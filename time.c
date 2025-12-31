/**
 * This file is part of the Titan Project.
 * Copyright (c) 2025 UW SARP
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * 
 * @file modules/kernel/src/time.c
 * @authors Aaron McBride
 * @brief Implementation of time related utilities.
 */

#include "kernel/include/time.h" 
#include "internal/include/mmio.h"
#include "kernel/include/thread.h"
#include "kernel_util.h"
#include "util/include/math.h"
#include "util/include/core.h"
#include "ti_config.h"

// Union type for assembling 64 bit time variable
union _time_uni_t {
  int64_t full_value; // Full 64 bit time value
  struct {
    uint32_t lo; // Low 32 bits of time value
    uint32_t hi; // High 32 bits of time value
  } parts;       // Individual 32-bit "parts" of full time value
};

// Multiplication factors for converting between units of time.
static const int64_t _TIME_MILLIS_MUL  = 1000;
static const int64_t _TIME_SECONDS_MUL = 1000000;
static const int64_t _TIME_MINUTES_MUL = 60000000;
static const int64_t _TIME_HOURS_MUL   = 3600000000;
static const int64_t _TIME_DAYS_MUL    = 86400000000;

// Global current time variable (in microseconds).
static int64_t _current_time = 0;

// Sequence counter for lock-free access to above _current_time variable.
static int32_t _current_time_seq = 0;

// Function to update the current time, called periodically by systick.
// Cannot be blocked by critical sections -> thus, we require use of sequence lock.
void _ti_update_time(void) {
  ti_atomic_add((uint32_t*)&_current_time_seq, 1U);
  _current_time += TI_CFG_KERNEL_TICK_FREQ / _TIME_SECONDS_MUL;
  ti_atomic_add((uint32_t*)&_current_time_seq, 1U);
}

int64_t ti_get_time(enum ti_errc_t* const errc_out) {
  *errc_out = TI_ERRC_NONE;
  int32_t seq_start, seq_end;
  union _time_uni_t time_uni;
  int32_t loop_count = 0;
  do {
    if (loop_count++ > TI_CFG_TIME_LOCK_ATTEMPTS) {
      *errc_out = TI_ERRC_TIMEOUT;
      return -1;
    }
    // If lo/hi are modified during read, seq value will change triggering retry
    seq_start = (int32_t)ti_atomic_load((uint32_t*)&_current_time_seq);
    time_uni.parts.lo = ti_atomic_load((uint32_t*)&_current_time);
    time_uni.parts.hi = ti_atomic_load((uint32_t*)&_current_time + 1);
    seq_end = (int32_t)ti_atomic_load((uint32_t*)&_current_time_seq);
  } while ((seq_start != seq_end) || (seq_start & 1));
  return time_uni.full_value;
}

void ti_sleep(const int64_t duration, enum ti_errc_t* const errc_out) {
  *errc_out = TI_ERRC_NONE;
  if (duration < 0) {
    *errc_out = TI_ERRC_INVALID_ARG;
    return;
  }
  enum ti_errc_t int_errc;
  const int64_t start_time = ti_get_time(&int_errc);
  if (int_errc != TI_ERRC_NONE) {
    *errc_out = TI_ERRC_INTERNAL;
    return;
  }
  while ((ti_get_time(&int_errc) - start_time) < duration) {
    if (int_errc != TI_ERRC_NONE) {
      *errc_out = TI_ERRC_INTERNAL;
      return;
    }
    ti_yield();
  }
}

void ti_sleep_until(int64_t time, enum ti_errc_t* errc_out) {
  *errc_out = TI_ERRC_NONE;
  enum ti_errc_t int_errc;
  const int64_t current_time = ti_get_time(&int_errc);
  if (int_errc != TI_ERRC_NONE) {
    *errc_out = TI_ERRC_INTERNAL;
    return;
  }
  if (time < current_time) {
    *errc_out = TI_ERRC_INVALID_ARG;
    return;
  }
  while (ti_get_time(&int_errc) < time) {
    if (int_errc != TI_ERRC_NONE) {
      *errc_out = TI_ERRC_INTERNAL;
      return;
    }
    ti_yield();
  }
}

int64_t ti_micros_to_time(int64_t micros, enum ti_errc_t* errc_out) {
  *errc_out = TI_ERRC_NONE;
  if (micros < 0) {
    *errc_out = TI_ERRC_INVALID_ARG;
    return -1;
  }
  return micros;
}

int64_t ti_time_to_micros(int64_t time, enum ti_errc_t* errc_out) {
  *errc_out = TI_ERRC_NONE;
  if (time < 0) {
    *errc_out = TI_ERRC_INVALID_ARG;
    return -1;
  }
  return time;
}

// Generic implementation of time_to_x functions
#define _TIME_TO_IMPL(unit_name, time_mul) \
  int64_t ti_time_to_##unit_name(const int64_t time, enum ti_errc_t* const errc_out) { \
    *errc_out = TI_ERRC_NONE; \
    if (time < 0) { \
      *errc_out = TI_ERRC_INVALID_ARG; \
      return -1; \
    } \
    return (time == 0) ? 0 : (time / time_mul); \
  }

// Generic implementation of x_to_time functions
#define _TIME_FROM_IMPL(unit_name, time_mul) \
  int64_t ti_##unit_name##_to_time(const int64_t unit_name, enum ti_errc_t* const errc_out) { \
    *errc_out = TI_ERRC_NONE; \
    if (unit_name < 0) { \
      *errc_out = TI_ERRC_INVALID_ARG; \
      return -1; \
    } \
    bool overflow_flag = false; \
    const int64_t result = TI_MUL(unit_name, time_mul, &overflow_flag); \
    if (overflow_flag) { \
      *errc_out = TI_ERRC_OVERFLOW; \
      return -1; \
    } \
    return result; \
  }

// Declare implementation of millis conversion functions
_TIME_FROM_IMPL(millis, _TIME_MILLIS_MUL)
_TIME_TO_IMPL(millis, _TIME_MILLIS_MUL)

// Declare implementation of seconds conversion functions 
_TIME_FROM_IMPL(seconds, _TIME_SECONDS_MUL)
_TIME_TO_IMPL(seconds, _TIME_SECONDS_MUL)

// Declare implementation of minutes conversion functions
_TIME_FROM_IMPL(minutes, _TIME_MINUTES_MUL)
_TIME_TO_IMPL(minutes, _TIME_MINUTES_MUL)

// Declare implementation of hours conversion functions
_TIME_FROM_IMPL(hours, _TIME_HOURS_MUL)
_TIME_TO_IMPL(hours, _TIME_HOURS_MUL)

// Declare implementation of days conversion functions
_TIME_FROM_IMPL(days, _TIME_DAYS_MUL)
_TIME_TO_IMPL(days, _TIME_DAYS_MUL)