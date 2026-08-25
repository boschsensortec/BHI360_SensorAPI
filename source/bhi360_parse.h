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
* @file       bhi360_parse.h
* @date       2026-08-14
* @version    v2.3.1
*
*/
#ifndef __BHI360_PARSE_H__
#define __BHI360_PARSE_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdbool.h>

#include "bhi360.h"

#include "bhi360_logbin.h"
#include "bhi360_klio_param.h"

#define PARSE_FLAG_NONE                 UINT8_C(0x00)
#define PARSE_FLAG_STREAM               UINT8_C(0x01)
#define PARSE_FLAG_LOG                  UINT8_C(0x02)
#define PARSE_FLAG_HEXSTREAM            UINT8_C(0x04)

#define PARSE_SET_FLAG(var, flag)       (var | flag)
#define PARSE_CLEAR_FLAG(var, flag)     (var & ~flag)

#define MAX_DEBUG_MESSAGE_PAYLOAD_SIZE  UINT8_C(16)

struct bhi360_parse_sensor_details
{
    uint8_t id;
    uint8_t accuracy;
    float scaling_factor;
    uint8_t parse_flag;
};

/* *INDENT-OFF* */
/* Contains information about klio capabilities and current state, as well as runtime configuration */
typedef struct
{
    uint8_t max_patterns;
    uint8_t max_pattern_blob_size;
    uint8_t auto_load_pattern_write_index;
    uint8_t auto_load_pattern;
} klio_info;
/* *INDENT-ON* */

struct bhi360_parse_ref
{
    struct bhi360_parse_sensor_details sensor[BHI360_MAX_SIMUL_SENSORS];
    struct bhi360_dev *bhy;
    struct bhi360_logbin_dev logdev;
};

/**
* @brief Function to get sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_get_sensor_details(uint8_t id, struct bhi360_parse_ref *ref);

/**
* @brief Function to add sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_add_sensor_details(uint8_t id, struct bhi360_parse_ref *ref);

/**
* @brief Function to parse meta event (wake-up and non-wake-up)
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse 3-axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse euler format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_euler(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse quaternion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_quaternion(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse 16-bit signed format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_s16_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse 32-bit scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u32(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse scalar event format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse activity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_activity(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse 24-bit unsigned format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_u24_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse 8-bit unsigned scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u8(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse generic format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_generic(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse debug message
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_debug_message(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Multi-tap
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_multitap(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Wrist Gesture Detector
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_wrist_gesture_detect(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Head Misalignment Calibration
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_hmc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Head Orientation Quaternion
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_oc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse for Head Orientation Euler
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_ec(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to set down sampling flag
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling value
*/
void bhi360_set_downsampling_flag(uint8_t sen_id, bool enable);

/**
* @brief Function to get down sampling flag
* @param[in] sen_id  : Virtual sensor ID
* @return Down sampling value
*/
bool bhi360_get_downsampling_flag(uint8_t sen_id);

/**
* @brief Function to set down sampling ratio
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling ratio
*/
void bhi360_set_downsampling_odr(uint8_t sen_id, int16_t odr);

/**
 * @brief Function to parse FIFO frame data into temperature
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] temperature  : Reference to the data buffer to store temperature in degree C
 */
void bhi360_parse_temperature_celsius(const uint8_t *data, bhi360_float *temperature);

/**
 * @brief Function to parse FIFO frame data into humidity
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] humidity : Reference to the data buffer to store humidity in %
 */
void bhi360_parse_humidity(const uint8_t *data, bhi360_float *humidity);

/**
 * @brief Function to parse FIFO frame data into barometric pressure
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] pressure : Reference to the data buffer to store pressure in Pascals
 */
void bhi360_parse_pressure(const uint8_t *data, bhi360_float *pressure);

/**
 * @brief Function to parse FIFO frame data into altitude
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] altitude : Reference to the data buffer to store altitude
 */
void bhi360_parse_altitude(const uint8_t *data, bhi360_float *altitude);

/**
* @brief Function to parse for Wrist wear wakeup data
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
 */
void bhi360_parse_wrist_wear_wakeup_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
 * @brief Function to parse for head gesture detector data
 * @param[in] callback_info : Pointer to callback information
 * @param[in] callback_ref  : Pointer to callback reference
 */

void bhi360_parse_head_gesture_detector_data(const struct bhi360_fifo_parse_data_info *callback_info,
                                             void *callback_ref);

/**
* @brief Function to parse Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_klio(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

void bhi360_parse_klio_handle_learnt_pattern(const struct bhi360_fifo_parse_data_info *callback_info,
                                             uint32_t s,
                                             uint32_t ns,
                                             struct bhi360_dev *bhy);

/**
* @brief Function to parse log Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_klio_log(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to set Klio information (capabilities, state and runtime configuration)
* @param[in] klio_info : Klio information
*/
void bhi360_set_klio_info(const klio_info* info);

/**
* @brief Function to get Klio information (capabilities, state and runtime configuration)
* @return Klio information
*/
klio_info* bhi360_get_klio_info(void);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_PARSE_H__ */
