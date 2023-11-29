/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "ws2812.pio.h"
#include "color_table.h"

#define IS_RGBW false
#define NUM_PIXELS 24

#define WS2812_PIN 21
#define UP_PIN 16
#define DOWN_PIN 17
#define LEFT_PIN 18
#define RIGHT_PIN 19
#define MID_PIN 20

#define UP_MSK (1 << UP_PIN)
#define DOWN_MSK (1 << DOWN_PIN)
#define LEFT_MSK (1 << LEFT_PIN)
#define RIGHT_MSK (1 << RIGHT_PIN)
#define MID_MSK (1 << MID_PIN)

#define COLOR_SELECTED (1 << 25)
#define COLOR_DELETING (1 << 26)

#define B_SHIFT 0
#define R_SHIFT 8
#define G_SHIFT 16
#define COLOR_SHIFT 24

#define B_MSK (0xff << B_SHIFT)
#define R_MSK (0xff << R_SHIFT)
#define G_MSK (0xff << G_SHIFT)
#define COLOR_MSK (R_MSK ^ G_MSK ^ B_MSK)

#define COLOR_RAND_MAX 255
#define COLOR_RAND_MIN 0

#define DEBOUNCE_PERIOD_US (200 * 1000)
#define FADE_STEP_US (50 * 1000)
#define DELETE_DELAY_US (7 * FADE_STEP_US)
#define LOOP_DELAY_US 1000

static int event_msk = 0;
static int colors[NUM_PIXELS] = {0};
static int color_state[NUM_PIXELS] = {0};
static uint64_t last_callback_timestamp = 0;
static uint joystick_pins[5] = {UP_PIN, DOWN_PIN, LEFT_PIN, RIGHT_PIN, MID_PIN};

static inline void put_pixel(uint32_t pixel_grb) {
  pixel_grb &= COLOR_MSK;
  pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline void put_pixels() {
  for (int i = 0; i < NUM_PIXELS; i++) {
    put_pixel(color_state[i]);
  }
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)(r) << R_SHIFT) ^ ((uint32_t)(g) << G_SHIFT) ^
         ((uint32_t)(b) << B_SHIFT);
}

static inline uint32_t fade_color(uint32_t color_in, int count) {
  int r = (color_in & R_MSK) >> R_SHIFT;
  int g = (color_in & G_MSK) >> G_SHIFT;
  int b = (color_in & B_MSK) >> B_SHIFT;

  // change the colors, but not the mask
  return urgb_u32(r >> count, g >> count, b >> count) ^
         ((color_in >> COLOR_SHIFT) << COLOR_SHIFT);
}

static_assert(RAND_MAX > (1 << 24));
// Get a random number in [low, high] (both inclusive)
static inline int get_rand(const int low, const int high) {
  // I don't care about the minuscule radix bias
  const int diff = (high - low) + 1;
  if (diff <= 0) {
    return low;
  }
  return (rand() % diff) + low;
}

static inline int get_random_color() {
  uint8_t r = get_rand(0, COLOR_TABLE_LEN - 1);
  uint8_t g = get_rand(0, COLOR_TABLE_LEN - 1);
  uint8_t b = get_rand(0, COLOR_TABLE_LEN - 1);
  return urgb_u32(color_table[r], color_table[g], color_table[b]);
}

// We know that the event is EDGE_RISE
static void joystick_callback(uint gpio, uint32_t _event __unused) {
  uint64_t ts = get_absolute_time();
  if ((ts - last_callback_timestamp) < DEBOUNCE_PERIOD_US) {
    // Debounce
    return;
  }
  last_callback_timestamp = ts;
  event_msk |= 1 << gpio;
  return;
}

static void delete_position(int pos) {
  for (int i = pos + 1; i < NUM_PIXELS; i++) {
    colors[i - 1] = colors[i];
    color_state[i - 1] = colors[i];
  }
  colors[NUM_PIXELS - 1] = 0;
  color_state[NUM_PIXELS - 1] = 0;
}

static void add_position(int pos) {
  colors[pos] = get_random_color();
  color_state[pos] = colors[pos];
}

static void core1_entry() {
  int prev = 0;

  while (1) {
    if (event_msk != prev) {
      prev = event_msk;
      printf("new mask: %08X\n", prev);
      if(prev == 0) {
        for(int i = 0; i < NUM_PIXELS; i++) {
          printf("color_state[%02d] = %08X\n", i, color_state[i]);
        }
      }
    }
    busy_wait_us(1);
  }
}

int main() {
  // set_sys_clock_48();
  stdio_init_all();
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, true);
  gpio_put(PICO_DEFAULT_LED_PIN, true);
  sleep_ms(1000);
  gpio_put(PICO_DEFAULT_LED_PIN, false);
  printf("--- WS2812 RGB Tasker ---\n"
         "- Using pins:\n"
         "LED=    %d\n"
         "UP=     %d\n"
         "DOWN=   %d\n"
         "LEFT=   %d\n"
         "RIGHT=  %d\n"
         "MID=    %d\n",
         WS2812_PIN, UP_PIN, DOWN_PIN, LEFT_PIN, RIGHT_PIN, MID_PIN);

  srand((uint32_t) get_absolute_time());

  bi_decl(bi_program_description("RGB Tasker"));
  bi_decl(bi_1pin_with_name(LEFT_PIN, "WS2812 pin"));
  bi_decl(bi_1pin_with_name(UP_PIN, "Joystick UP (first pin)"));
  bi_decl(bi_1pin_with_name(MID_PIN, "Joystick MID (last pin)"));

  for (int i = 0; i < 5; i++) {
    gpio_init(joystick_pins[i]);
    gpio_set_dir(joystick_pins[i], false);
    gpio_set_pulls(joystick_pins[i], true, false);
  }
  for (int i = 0; i < 5; i++) {
    gpio_set_irq_enabled_with_callback(joystick_pins[i], GPIO_IRQ_EDGE_FALL, true,
                                       joystick_callback);
  }

  multicore_launch_core1(core1_entry);

  uint64_t start_time = 0;
  uint64_t selection_change = 0;
  uint64_t delete_start = 0;
  uint64_t locked_until = 0;

  int task_count = 0;
  int selection_pos = 0;

  bool lock_ack = true;

  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, WS2812_PIN, 800 * 1000, IS_RGBW);
  put_pixels(); // clear everything

  while (true) {
    start_time = get_absolute_time();
    if (start_time > locked_until) {
      if (!lock_ack) {
        lock_ack = true;
        delete_position(selection_pos);
        task_count--;
        if (selection_pos == task_count) {
          selection_pos = MAX(0, task_count - 1);
        }
        if (selection_pos >= 0 && selection_pos < task_count) {
          color_state[selection_pos] |= COLOR_SELECTED;
          selection_change = start_time;
        }
      }
      if (event_msk & UP_MSK) {
        event_msk &= ~UP_MSK;
        if (task_count < NUM_PIXELS) {
          add_position(task_count);
          task_count++;
        }
      }
      if (event_msk & DOWN_MSK) {
        event_msk &= ~DOWN_MSK;
        if (task_count > 0) {
          task_count--;
          delete_position(task_count);
          if (selection_pos >= task_count) {
            selection_pos = MAX(0, task_count - 1);
            if (selection_pos >= 0 && selection_pos < task_count) {
              color_state[selection_pos] |= COLOR_SELECTED;
              selection_change = start_time;
            }
          }
        }
      }
      if (event_msk & RIGHT_MSK) {
        event_msk &= ~RIGHT_MSK;
        if ((selection_pos + 1) < task_count) {
          color_state[selection_pos] &= ~COLOR_SELECTED;
          color_state[selection_pos] = (color_state[selection_pos] & (~COLOR_MSK)) | colors[selection_pos];
          selection_pos++;
          color_state[selection_pos] |= COLOR_SELECTED;
          selection_change = start_time;
        }
      }
      if (event_msk & LEFT_MSK) {
        event_msk &= ~LEFT_MSK;
        if (selection_pos > 0) {
          color_state[selection_pos] &= ~COLOR_SELECTED;
          color_state[selection_pos] = (color_state[selection_pos] & (~COLOR_MSK)) | colors[selection_pos];
          selection_pos--;
          color_state[selection_pos] |= COLOR_SELECTED;
          selection_change = start_time;
        }
      }
      if (event_msk & MID_MSK) {
        event_msk &= ~MID_MSK;
        if (selection_pos >= 0 && selection_pos < NUM_PIXELS && task_count > 0) {
          color_state[selection_pos] |= COLOR_DELETING;
          delete_start = start_time;
          locked_until = start_time + DELETE_DELAY_US;
          lock_ack = false;
        }
      }
    }

    if (locked_until >= start_time) {
      // deleting something
      // calculate ceil
      int steps_after =
          (start_time - delete_start + FADE_STEP_US - 1) / FADE_STEP_US;
      for (int i = 0; i < NUM_PIXELS; i++) {
        if (color_state[i] & COLOR_DELETING) {
          color_state[i] = (color_state[i] & (~COLOR_MSK)) |
                           fade_color(colors[i], steps_after);
        }
      }
    } else {
      // just do selection animation
      // breating effect
      if(task_count == 1) {
        color_state[0] |= COLOR_SELECTED;
      } 
      int steps_after = (start_time - selection_change) / FADE_STEP_US;
      steps_after %= 8;
      int fade_count = steps_after > 4 ? 7 - steps_after : steps_after;
      for (int i = 0; i < NUM_PIXELS; i++) {
        if (color_state[i] & COLOR_SELECTED) {
          color_state[i] = (color_state[i] & (~COLOR_MSK)) |
                           fade_color(colors[i], fade_count);
        }
      }
    }

    put_pixels();
    busy_wait_until(start_time + LOOP_DELAY_US);
  }
}
