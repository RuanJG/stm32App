/**
 * @file    swd_host.h
 * @brief   Host driver for accessing the DAP
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SWDHOST_CM_H
#define SWDHOST_CM_H

#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "swd_bsp.h"

#ifdef TARGET_MCU_CORTEX_A
#include "swd_debug_ca.h"
#else
#include "swd_debug_cm.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONNECT_NORMAL,
    CONNECT_UNDER_RESET,
} SWD_CONNECT_TYPE;

typedef struct __attribute__((__packed__)) {
    uint32_t breakpoint;
    uint32_t static_base;
    uint32_t stack_pointer;
} program_syscall_t;

typedef struct __attribute__((__packed__)) {
    const uint32_t  init;
    const uint32_t  uninit;
    const uint32_t  erase_chip;
    const uint32_t  erase_sector;
    const uint32_t  program_page;
    const uint32_t  verify;
    const program_syscall_t sys_call_s;
    const uint32_t  program_buffer;
    const uint32_t  algo_start;
    const uint32_t  algo_size;
    const uint32_t *algo_blob;
    const uint32_t  program_buffer_size;
} program_target_t;

typedef struct __attribute__((__packed__)) {
    const uint32_t start;
    const uint32_t size;
} sector_info_t;

typedef enum _target_state {
    RESET_HOLD,              //!< Hold target in reset
    RESET_PROGRAM,           //!< Reset target and setup for flash programming
    RESET_RUN,               //!< Reset target and run normally
    NO_DEBUG,                //!< Disable debug on running target
    DEBUG,                   //!< Enable debug on running target
    HALT,                    //!< Halt the target without resetting it
    RUN,                     //!< Resume the target without resetting it
    POST_FLASH_RESET,        //!< Reset target after flash programming
    POWER_ON,                //!< Poweron the target
    SHUTDOWN,                //!< Poweroff the target
} target_state_t;

typedef enum _reset_type {
    kHardwareReset = 1,
    kSoftwareReset,
} reset_type_t;

uint8_t swd_init(void);
uint8_t swd_off(void);
uint8_t swd_init_debug(void);
uint8_t swd_clear_errors(void);
uint8_t swd_read_dp(uint8_t adr, uint32_t *val);
uint8_t swd_write_dp(uint8_t adr, uint32_t val);
uint8_t swd_read_ap(uint32_t adr, uint32_t *val);
uint8_t swd_write_ap(uint32_t adr, uint32_t val);
uint8_t swd_read_word(uint32_t addr, uint32_t *val);
uint8_t swd_write_word(uint32_t addr, uint32_t val);
uint8_t swd_read_byte(uint32_t addr, uint8_t *val);
uint8_t swd_write_byte(uint32_t addr, uint8_t val);
uint8_t swd_read_memory(uint32_t address, uint8_t *data, uint32_t size);
uint8_t swd_write_memory(uint32_t address, uint8_t *data, uint32_t size);
uint8_t swd_read_core_register(uint32_t n, uint32_t *val);
uint8_t swd_write_core_register(uint32_t n, uint32_t val);
uint8_t swd_flash_syscall_exec(const program_syscall_t *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4);
uint8_t swd_set_target_state_hw(target_state_t state);
uint8_t swd_set_target_state_sw(target_state_t state);
uint8_t swd_transfer_retry(uint32_t req, uint32_t *data);
void int2array(uint8_t *res, uint32_t data, uint8_t len);
void swd_set_reset_connect(SWD_CONNECT_TYPE type);
void swd_set_soft_reset(uint32_t soft_reset_type);
void swd_set_target_reset_type( reset_type_t rstType );

//JK: add
int swd_set_target_state(target_state_t state);
//JK: mark
//int target_before_init_debug();   //should define if it is in need , be used in swd_init_debug()

#ifdef __cplusplus
}
#endif

#endif
