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
 * @file modules/kernel/include/kernel/sys.h
 * @authors Aaron McBride
 * @brief General system utilities.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "util/include/errc.h"

/** @breif Denotes identity of a core in the MCU. */
enum ti_core_id_t {
  TI_CORE_ID_CM7, /** @brief Denotes the cortex-M7 core. */
  TI_CORE_ID_CM4, /** @brief Denotes the cortex-M4 core. */
};

/**
 * @brief Triggers a system reset (both cores).
 */
void ti_sys_restart(void);

/**
 * @brief Shuts down the system (both cores).
 * @note - The system will stay in a low power state until it recieves an external reset event.
 */
void ti_sys_shutdown(void);

/**
 * @brief Puts the core this function is called on into a low power state until an event/interrupt occurs.
 * @note - This function does nothing if called from within a critical section.
 */
void ti_sys_sleep(void);

/**
 * @brief Determines the identity of the core this function is called from.
 * @returns (enum ti_core_id_t) Denotes the identity of the core this function is called from.
 */
enum ti_core_id_t ti_get_core(void);

/**
 * @brief Determines if the caller is currently executing within an interrupt context.
 * @returns (bool) True if the caller is executing within an interrupt context, or false otherwise.
 */
bool ti_is_interrupt(void);

/**
 * @brief Enters a critical section.
 * @note - Critical sections can be nested (that is entered/exited multiple times) so long as each entry is matched with an exit.
 * @note - When within a critical section, all interrupts on the core this function is called from are disabled.
 * @note - When within a critical section, the thread scheduler cannot run to switch threads on the core this function is called from.
 * @note - Any interrupt triggered from within a critical section will wait until the critical section is exited to be serviced.
 * @remark - Critical sections should be used sparingly and kept as short as possible to avoid negatively impacting system responsiveness.
 */
void ti_enter_critical(void);

/**
 * @brief Exits a critical section.
 * @param (enum ti_errc_t*) Out argument for error code.
 * @note - The state of the critical section system is unchanged if any error occurs.
 * @warning - TI_ERRC_INVALID_STATE is raised if this function is called when not within a critical section.
 */
void ti_exit_critical(enum ti_errc_t* errc_out);

/**
 * @brief Determines if the calling thread/interrupt is within a critical section.
 * @returns (bool) True if the calling thread/interrupt is within a critical section, or false otherwise.
 */
bool ti_is_critical(void);

/**
 * @brief Enters an exclusive section.
 * @param errc_out (enum ti_errc_t*) Out argument for error code.
 * @note - Exclusive sections can be nested (that is entered/exited multiple times) so long as each entry is matched with an exit.
 * @note - When within an exclusive section, no threads/interrupts can execute on any other core.
 * @note - The state of the exclusive section system is unchanged if any error other than TI_ERRC_INTERNAL occurs.
 * @note - The state of the exclusive section system is undefined if TI_ERRC_INTERNAL occurs.
 * @warning - TI_ERRC_TIMEOUT is raised if the exclusive section cannot be entered within the configured timeout duration.
 * @warning - TI_ERRC_INTERNAL can be raised by this function if an internal error occurs.
 * @remark - Exclusive section should be used VERY sparingly as they are very intensive to enter/exit and can severely impact system responsiveness.
 */
void ti_enter_exclusive(enum ti_errc_t* errc_out);

/**
 * @brief Exits an exclusive section.
 * @param errc_out (enum ti_errc_t*) Out argument for error code.
 * @note - Exclusive sections can be nested (that is entered/exited multiple times) so long as each entry is matched with an exit.
 * @note - The state of the exclusive section system is unchanged if any error other than TI_ERRC_INTERNAL occurs.
 * @note - The state of the exclusive section system is undefined if TI_ERRC_INTERNAL is raised.
 * @warning - TI_ERRC_INVALID_STATE is raised if this function is called when not within an exclusive section.
 * @warning - TI_ERRC_INTERNAL can be raised by this function if an internal error occurs.
 */
void ti_exit_exclusive(enum ti_errc_t* errc_out);

/**
 * @brief Determines if the calling thread/interrupt is within an exclusive section.
 * @returns (bool) True if the calling thread/interrupt is within an exclusive section, or false otherwise.
 */
bool ti_is_exclusive(void);