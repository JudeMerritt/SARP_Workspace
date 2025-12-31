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
 * @file modules/kernel/src/sys.c
 * @authors Aaron McBride
 * @brief Implementation of general system utilities.
 */

#include "kernel/include/sys.h"
#include "internal/include/mmio.h"
#include "util/include/atomic.h"
#include "kernel/include/time.h"
#include "ti_config.h"

// "Key" value for VECTKEY field to allow writes to AIRCR
const static uint32_t _RESET_VECTKEY_VALUE = 0x5FA;

// Part number for CM7 core (stored in CPUID)
const static uint32_t _CM7_PARTNO = 0xC27;

// Critical section nesting counters for CM7/CM4 core
static int32_t _cm7_critical_count = 0;
static int32_t _cm4_critical_count = 0;

// Lock variable for accessing exclusive section info
static int32_t _exclusive_lock = 0;

// Exclusive section nesting counters for CM7/CM4 core
static int32_t _exclusive_count = 0;

// Exclusive section acknowledgment variables for CM7/CM4 core
static int32_t _cm7_exclusive_ack = 0;
static int32_t _cm4_exclusive_ack = 0;

// Shutdown acknowledgment variables for CM7/CM4 core
static int32_t _cm7_shutdown_flag = 0;
static int32_t _cm4_shutdown_flag = 0;

// Gets the critical count for the core this function is running on.
static int32_t* _get_critical_count(void) {
  return ti_get_core() == TI_CORE_ID_CM7 ? &_cm7_critical_count : &_cm4_critical_count;
}

// Gets the exclusive section tag for the core this function is running on.
static int32_t _get_this_exclusive_tag(void) {
  return (ti_get_core() == TI_CORE_ID_CM7) ? 1 : -1;
}

// Gets the exclusive count for the core this function is running on.
static int32_t _get_alt_exclusive_tag(void) {
  return (ti_get_core() == TI_CORE_ID_CM7) ? -1 : 1;
}

// Gets the exclusive acknowledgment variable for the core this function is running on.
static int32_t* _get_this_exclusive_ack(void) {
  return (ti_get_core() == TI_CORE_ID_CM7) ? &_cm7_exclusive_ack : &_cm4_exclusive_ack;
}

// Gets the shutdown flag for the core this function is running on.
static int32_t* _get_this_shutdown_flag(void) {
  return ti_get_core() == TI_CORE_ID_CM7 ? &_cm7_shutdown_flag : &_cm4_shutdown_flag;
}

// Gets the shutdown flag for the core this function is NOT running on.
static int32_t* _get_alt_shutdown_flag(void) {
  return ti_get_core() == TI_CORE_ID_CM7 ? &_cm4_shutdown_flag : &_cm7_shutdown_flag;
}

// Executes shutdown sequence for the CM7 core
__attribute__((noreturn))
static void _exec_cm7_shutdown(void) {
  // Disable interrupts/faults to ensure shutdown sequence proceeds even if fault.
  asm volatile ("cpsid f");
  typedef void (*kernel_exit_fn_t)(void);
  extern kernel_exit_fn_t* __ti_kernel_cm7_exit_array_start;
  extern kernel_exit_fn_t* __ti_kernel_cm7_exit_array_end;
  kernel_exit_fn_t* cur_kernel_fn = __ti_kernel_cm7_exit_array_start;
  while (cur_kernel_fn < __ti_kernel_cm7_exit_array_end) {
    (*cur_kernel_fn)();
    cur_kernel_fn++;
  }
  typedef void (*mcu_exit_fn_t)(void);
  extern mcu_exit_fn_t* __mcu_exit_array_start;
  extern mcu_exit_fn_t* __mcu_exit_array_end;
  mcu_exit_fn_t* cur_mcu_fn = __mcu_exit_array_start;
  while (cur_mcu_fn < __mcu_exit_array_end) {
    (*cur_mcu_fn)();
    cur_mcu_fn++;
  }
  SET_FIELD(SCB_SCR, SCB_SCR_SLEEPDEEP);
  asm volatile (
    "dsb \n\t"
    "isb \n\t"
  );
  while (true) {
    asm volatile ("wfe");
  }
}

// Executes shutdown sequence for the CM4 core
__attribute__((noreturn))
static void _exec_cm4_shutdown(void) {
  // Disable interrupts/faults to ensure shutdown sequence proceeds even if fault.
  asm volatile ("cpsid f");
  typedef void (*kernel_exit_fn_t)(void);
  extern kernel_exit_fn_t* __ti_kernel_cm4_exit_array_start;
  extern kernel_exit_fn_t* __ti_kernel_cm4_exit_array_end;
  kernel_exit_fn_t* cur_fn = __ti_kernel_cm4_exit_array_start;
  while (cur_fn < __ti_kernel_cm4_exit_array_end) {
    (*cur_fn)();
    cur_fn++;
  }
  SET_FIELD(SCB_SCR, SCB_SCR_SLEEPDEEP);
  asm volatile (
    "dsb \n\t"
    "isb \n\t"
  );
  while (true) {
    asm volatile ("wfe");
  }
}

// Gets the shutdown function for the core this function is running on.
typedef void (*_shutdown_fn_t)(void);
static _shutdown_fn_t _get_this_shutdown_fn(void) {
  return ti_get_core() == TI_CORE_ID_CM7 ? &_exec_cm7_shutdown : &_exec_cm4_shutdown;
}

// Generic implementation of exclusive section acknowledgment system for update handlers
#define _EXCLUSIVE_ACK_IMPL(this_core_name, alt_core_name) \
  ti_enter_critical(); \
  enum ti_errc_t int_errc; \
  const int32_t alt_tag = _get_alt_exclusive_tag(); \
  const int64_t start_time = ti_get_time(&int_errc); \ 
  if (int_errc == TI_ERRC_NONE) { \
    while (((int32_t)ti_atomic_load((uint32_t*)&_exclusive_lock) == alt_tag) && \
          ((ti_get_time(&int_errc) - start_time) < TI_CFG_EXCLUSIVE_SECTION_TIMEOUT)) { \
      if (int_errc != TI_ERRC_NONE) { \
        break; \
      } \
      ti_atomic_store((uint32_t*)&_##this_core_name##_exclusive_ack, 1U); \
    } \
  } \
  ti_atomic_store((uint32_t*)&_##this_core_name##_exclusive_ack, 0U); \
  ti_exit_critical(&(enum ti_errc_t){0});

// System update handler for CM7 core (triggered by SEV instruction from CM4 core)
void cpu1_sev_irq_handler(void) {
  ti_enter_critical();
  if ((int32_t)ti_atomic_load((uint32_t*)&_cm4_shutdown_flag) != 0) {
    ti_atomic_store((uint32_t*)&_cm7_shutdown_flag, 1U);
    _exec_cm7_shutdown();
  }
  ti_exit_critical(&(enum ti_errc_t){0});
  _EXCLUSIVE_ACK_IMPL(cm7, cm4)
}

// System update handler for CM4 core (triggered by SEV instruction on CM7 core)
void cpu2_sev_irq_handler(void) {
  ti_enter_critical();
  if ((int32_t)ti_atomic_load((uint32_t*)&_cm7_shutdown_flag) != 0) {
    ti_atomic_store((uint32_t*)&_cm4_shutdown_flag, 1U);
    _exec_cm4_shutdown();
  }
  ti_exit_critical(&(enum ti_errc_t){0});
  _EXCLUSIVE_ACK_IMPL(cm4, cm7)
}

// Non-static internal function to reset critical sections
void _ti_reset_critical(void) {
  int32_t* const critical_count = _get_critical_count();
  *critical_count = 0;
  asm volatile (
    "msr basepri, #0 \n\t"
    "isb             \n\t"
  );
}

// Non-static internal function to reset exclusive sections
void _ti_reset_exclusive(void) {
  ti_enter_critical();
  const int32_t this_tag = _get_this_exclusive_tag();
  if ((int32_t)ti_atomic_load((uint32_t*)&_exclusive_lock) == this_tag) {
    _exclusive_count = 0;
    ti_atomic_store((uint32_t*)&_exclusive_lock, 0U);
  }
  ti_exit_critical(&(enum ti_errc_t){0});
}

__attribute__((noreturn))
void ti_sys_restart(void) {
  // Disable interrupts/faults to ensure reset sequence proceeds even on fault.
  asm volatile ("cpsid f");
  uint32_t reg_value = *SCB_AIRCR;
  WRITE_FIELD(&reg_value, SCB_AIRCR_VECTKEYSTAT, _RESET_VECTKEY_VALUE);
  SET_FIELD(&reg_value, SCB_AIRCR_SYSRESETREQ);
  *SCB_AIRCR = reg_value;
  asm volatile (
    "dsb \n\t"
    "isb \n\t"
  );
  while (true) {
    asm volatile ("wfe");
  }
}

__attribute__((noreturn))
void ti_sys_shutdown(void) {
  _shutdown_fn_t const this_shutdown_fn = _get_this_shutdown_fn();
  int32_t* const this_shutdown_flag = _get_this_shutdown_flag();
  int32_t* const alt_shutdown_flag = _get_alt_shutdown_flag();
  ti_atomic_store((uint32_t*)this_shutdown_flag, 1U);
  // SEV instruction runs above handler on other core to start its shutdown sequence
  asm volatile (
    "dsb \n\t"
    "sev \n\t"
  );
  // Dont start shutdown sequence until other core acknowledges request and starts shutdown itself
  while ((int32_t)ti_atomic_load((uint32_t*)alt_shutdown_flag) != 1);
  this_shutdown_fn();
}

void ti_sys_sleep(void) {
  if (!ti_is_critical()) {
    asm volatile (
      "dsb \n\t"
      "isb \n\t"
      "wfi \n\t"
    );
  }
}

enum ti_core_id_t ti_get_core(void) {
  const uint32_t partno = READ_FIELD(SCB_CPUID, SCB_CPUID_PARTNO);
  return (partno == _CM7_PARTNO) ? TI_CORE_ID_CM7 : TI_CORE_ID_CM4;
}

bool ti_is_interrupt(void) {
  uint32_t ipsr_value;
  asm volatile ("mrs %[ipsr_value], ipsr" : [ipsr_value] "=r" (ipsr_value));
  return ipsr_value != 0;
}

void ti_enter_critical(void) {
  int32_t* const critical_count = _get_critical_count();
  if (*critical_count == 0) {
    asm volatile (
      "msr basepri, #1 \n\t"
      "isb             \n\t"
    );
  }
  (*critical_count)++;
}

void ti_exit_critical(enum ti_errc_t* const errc_out) {
  *errc_out = TI_ERRC_NONE;
  int32_t* const critical_count = _get_critical_count();
  if (*critical_count == 0) {
    *errc_out = TI_ERRC_INVALID_STATE;
    return;
  }
  (*critical_count)--;
  if (*critical_count == 0) {
    asm volatile (
      "msr basepri, #0 \n\t"
      "isb             \n\t"
    );
  }
}

bool ti_is_critical(void) {
  int32_t* const critical_count = _get_critical_count();
  return *critical_count > 0;
}

void ti_enter_exclusive(enum ti_errc_t* errc_out) {
  *errc_out = TI_ERRC_NONE;
  ti_enter_critical();
  enum ti_errc_t int_errc;
  const int32_t this_tag = _get_this_exclusive_tag();
  const int32_t alt_tag = _get_alt_exclusive_tag();
  const int32_t* const this_ack = _get_this_exclusive_ack();
  if ((int32_t)ti_atomic_load((uint32_t*)&_exclusive_lock) != this_tag) {
    int32_t cur_tag = 0;
    const int64_t start_time = ti_get_time(&int_errc);
    if (int_errc != TI_ERRC_NONE) {
      ti_exit_critical(&(enum ti_errc_t){0});
      *errc_out = TI_ERRC_INTERNAL;
      return;
    }
    // Must acknowledge other core's exclusive section because this must be a critical section and we would deadlock otherwise.
    while (!ti_atomic_cmp_exchange((uint32_t*)&_exclusive_lock, (uint32_t*)&cur_tag, (uint32_t)this_tag)) {
      const int64_t current_time = ti_get_time(&int_errc);
      if (int_errc != TI_ERRC_NONE) {
        ti_exit_critical(&(enum ti_errc_t){0});
        *errc_out = TI_ERRC_INTERNAL;
        return;
      }
      if ((current_time - start_time) > TI_CFG_EXCLUSIVE_SECTION_TIMEOUT) {
        ti_exit_critical(&int_errc);
        if (int_errc != TI_ERRC_NONE) {
          *errc_out = TI_ERRC_INTERNAL;
          return;
        }
        *errc_out = TI_ERRC_TIMEOUT;
        return;
      }
      if (cur_tag == alt_tag) {
        ti_atomic_store((uint32_t*)this_ack, 1U);
      }
      cur_tag = 0;
    }
  }
  ti_atomic_store((uint32_t*)this_ack, 0U);
  _exclusive_count++;
  const int64_t start_time = ti_get_time(&int_errc);
  if (int_errc != TI_ERRC_NONE) {
    _exclusive_count--;
    if (_exclusive_count == 0) {
      ti_atomic_store((uint32_t*)&_exclusive_lock, 0U);
    }
    ti_exit_critical(&(enum ti_errc_t){0});
    *errc_out = TI_ERRC_INTERNAL;
    return;
  }
  const int32_t* const alt_ack = _get_alt_exclusive_ack();
  while (ti_atomic_load((uint32_t*)alt_ack) != 1U) {
    const int64_t current_time = ti_get_time(&int_errc);
    if (int_errc != TI_ERRC_NONE) {
      _exclusive_count--;
      if (_exclusive_count == 0) {
        ti_atomic_store((uint32_t*)&_exclusive_lock, 0U);
      }
      ti_exit_critical(&(enum ti_errc_t){0});
      *errc_out = TI_ERRC_INTERNAL;
      return;
    }
    if ((current_time - start_time) > TI_CFG_EXCLUSIVE_SECTION_ACK_TIMEOUT) {
      _exclusive_count--;
      if (_exclusive_count == 0) {
        ti_atomic_store((uint32_t*)&_exclusive_lock, 0U);
      }
      ti_exit_critical(&int_errc);
      if (int_errc != TI_ERRC_NONE) {
        *errc_out = TI_ERRC_INTERNAL;
        return;
      }
      *errc_out = TI_ERRC_TIMEOUT;
      return;
    }
  }
  ti_exit_critical(&int_errc);
  if (int_errc != TI_ERRC_NONE) {
    *errc_out = TI_ERRC_INTERNAL;
  }
}

void ti_exit_exclusive(enum ti_errc_t* errc_out) {
  *errc_out = TI_ERRC_NONE;
  ti_enter_critical();
  enum ti_errc_t int_errc;
  const int32_t this_tag = _get_this_exclusive_tag();
  if ((int32_t)ti_atomic_load((uint32_t*)&_exclusive_lock) != this_tag) {
    ti_exit_critical(&int_errc);
    if (int_errc != TI_ERRC_NONE) {
      *errc_out = TI_ERRC_INTERNAL;
      return;
    }
    *errc_out = TI_ERRC_INVALID_STATE;
    return;
  }
  int32_t* const alt_ack = _get_alt_exclusive_ack();
  if (ti_atomic_load((uint32_t*)alt_ack) == 0U) {
    ti_exit_critical(&int_errc);
    if (int_errc != TI_ERRC_NONE) {
      *errc_out = TI_ERRC_INTERNAL;
      return;
    }
    *errc_out = TI_ERRC_TIMEOUT;
    return;
  }
  _exclusive_count--;
  if (_exclusive_count == 0) {
    ti_atomic_store((uint32_t*)&_exclusive_lock, 0U);
  }
  ti_exit_critical(&int_errc);
  if (int_errc != TI_ERRC_NONE) {
    *errc_out = TI_ERRC_INTERNAL;
  }
}

bool ti_is_exclusive(void) {
  const int32_t this_tag = _get_this_exclusive_tag();
  const int32_t lock_value = (int32_t)ti_atomic_load((uint32_t*)&_exclusive_lock);
  return lock_value == this_tag;
}