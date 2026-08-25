/**
 * Copyright (c) 2026 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    common.h
 * @brief   Common header file for the BHI360 examples
 *
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdbool.h>
#include <inttypes.h>
#include "bhi360.h"
#include "bhi360_defs.h"
#include "bhi360_activity_param.h"
#include "bhi360_bsx_algo_param.h"
#include "bhi360_klio_param.h"
#include "bhi360_multi_tap_param.h"
#include "bhi360_phy_sensor_ctrl_param.h"
#include "bhi360_generic_features_param.h"
#include "bhi360_system_param.h"
#include "bhi360_virtual_sensor_conf_param.h"
#include "bhi360_virtual_sensor_info_param.h"
#include "bhi360_head_feature_param.h"
#include "coines.h"

#define BHY360_APP20_CS_PIN     COINES_SHUTTLE_PIN_7
#define BHY360_APP20_INT_PIN    COINES_SHUTTLE_PIN_21
#define BHY360_APP20_RESET_PIN  COINES_SHUTTLE_PIN_8
#define BHY360_APP30_CS_PIN     COINES_MINI_SHUTTLE_PIN_2_1
#define BHY360_APP30_INT_PIN    COINES_MINI_SHUTTLE_PIN_1_6
#define BHY360_APP30_RESET_PIN  COINES_MINI_SHUTTLE_PIN_1_5

#ifdef PC
#ifdef COINES_BRIDGE
#define BHI360_RD_WR_LEN        256      /* Coines bridge maximum read write length */
#else
#define BHI360_RD_WR_LEN        44       /* USB maximum read write length(DD firmware) */
#endif
#else
#define BHI360_RD_WR_LEN        256      /* MCU maximum read write length */
#endif

#define WORK_BUFFER_SIZE        UINT16_C(2048)

#define BHI360_ODR_0_HZ         (0.0f)
#define BHI360_ODR_1_5625_HZ    (1.5625f)
#define BHI360_ODR_3_125_HZ     (3.125f)
#define BHI360_ODR_6_25_HZ      (6.25f)
#define BHI360_ODR_12_5_HZ      (12.5f)
#define BHI360_ODR_25_HZ        (25.0f)
#define BHI360_ODR_50_HZ        (50.0f)
#define BHI360_ODR_100_HZ       (100.0f)
#define BHI360_ODR_200_HZ       (200.0f)
#define BHI360_ODR_400_HZ       (400.0f)
#define BHI360_ODR_800_HZ       (800.0f)

char *get_coines_error(int16_t rslt);
char *get_api_error(int8_t error_code);
char *get_sensor_error_text(uint8_t sensor_error);
char *get_sensor_name(uint8_t sensor_id);
char *get_physical_sensor_name(uint8_t sensor_id);
uint8_t get_physical_sensor_id(uint8_t virt_sensor_id);
float get_sensor_dynamic_range_scaling(uint8_t sensor_id, float dynamic_range);
char *get_sensor_si_unit(uint8_t sensor_id);
char *get_sensor_parse_format(uint8_t sensor_id);
char *get_sensor_axis_names(uint8_t sensor_id);
char *get_klio_error(bhi360_klio_param_driver_error_state_t error);
void print_api_error(int8_t rslt, struct bhi360_dev *dev);
void upload_firmware(const uint8_t fw[], uint32_t length, struct bhi360_dev *dev);

void setup_interfaces(bool reset_power, enum bhi360_intf intf);
void setup_interfaces_with_port(bool reset_power, enum bhi360_intf intf, const char *com_port);
void init_sensor(struct bhi360_dev *dev, enum bhi360_intf intf);
void setup_host_int_ctrl(struct bhi360_dev *dev);
void close_interfaces(enum bhi360_intf intf);
int8_t bhi360_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhi360_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhi360_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhi360_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bhi360_delay_us(uint32_t us, void *private_data);
bool get_interrupt_status(void);

#endif /* _COMMON_H_ */
