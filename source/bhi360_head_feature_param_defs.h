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
* @file       bhi360_head_feature_param_defs.h
* @date       2026-08-14
* @version    v2.3.1
*
*/

#ifndef _BHI360_HEAD_FEATURE_PARAM_DEFS_H_
#define _BHI360_HEAD_FEATURE_PARAM_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdlib.h>
#include <stdlib.h>

#include "bhi360_defs.h"

/*! Virtual Sensor Macros */
#define BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG                               UINT8_C(120)                   /*Head FEATURE
                                                                                                     * Misalignment*/
#define BHI360_SENSOR_ID_IMU_HEAD_ORI_Q                                 UINT8_C(121)                   /*IMU Head
                                                                                                     * FEATURE
                                                                                                     * Quaternion*/
#define BHI360_SENSOR_ID_NDOF_HEAD_ORI_Q                                UINT8_C(122)                   /*NDOF Head
                                                                                                     * FEATURE
                                                                                                     * Quaternion*/
#define BHI360_SENSOR_ID_IMU_HEAD_ORI_E                                 UINT8_C(123)                   /*IMU Head
                                                                                                     * FEATURE
                                                                                                     * Euler*/
#define BHI360_SENSOR_ID_NDOF_HEAD_ORI_E                                UINT8_C(124)                   /*NDOF Head
                                                                                                     * FEATURE
                                                                                                     * Euler*/

#define BHI360_SENSOR_ID_HEAD_GESTURE_DETECTOR                          UINT8_C(125)                   /*Head Gesture
                                                                                                     * Detector*/

#define BHI360_HEAD_FEATURE_PARAM_PAGE_BASE                             UINT16_C(0xC00)

#define BHI360_HMC_PARAM_HMC_TRIGGER_ID                                 UINT8_C(1)
#define BHI360_HMC_PARAM_HMC_TRIGGER_LENGTH                             UINT8_C(4)
#define BHI360_HMC_PARAM_HMC_TRIGGER_CALIB_SET                          UINT8_C(1)

#define BHI360_HMC_PARAM_HMC_CONFIG_ID                                  UINT8_C(2)
#define BHI360_HMC_PARAM_HMC_CONFIG_LENGTH                              UINT8_C(8)

#define BHI360_HMC_PARAM_HMC_SET_CONFIG_TO_DEFAULT_ID                   UINT8_C(3)
#define BHI360_HMC_PARAM_HMC_SET_CONFIG_TO_DEFAULT_LENGTH               UINT8_C(4)
#define BHI360_HMC_PARAM_HMC_SET_CONFIG_TO_DEFAULT_SET                  UINT8_C(1)

#define BHI360_HMC_PARAM_GET_ALGO_VERSION_ID                            UINT8_C(4)
#define BHI360_HMC_PARAM_GET_ALGO_VERSION_LENGTH                        UINT8_C(4)

#define BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_ID                   UINT8_C(5)
#define BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_WLENGTH              UINT8_C(20)
#define BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_RLENGTH              UINT8_C(20)

#define BHI360_HMC_PARAM_SET_MODE_ID                                    UINT8_C(6)
#define BHI360_HMC_PARAM_SET_MODE_LENGTH                                UINT8_C(13)

#define BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ID                   UINT8_C(10)
#define BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_LENGTH               UINT8_C(4)
#define BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_DISABLE              UINT8_C(0)
#define BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ENABLE               UINT8_C(1)

#define BHI360_OC_PARAM_GET_ALGO_VERSION_ID                             UINT8_C(11)
#define BHI360_OC_PARAM_GET_ALGO_VERSION_LENGTH                         UINT8_C(4)

#define BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_ID              UINT8_C(13)
#define BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_LENGTH          UINT8_C(4)
#define BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_DISABLE         UINT8_C(0)
#define BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_ENABLE          UINT8_C(1)

#define bhi360_head_feature_param_hgd_get_algo_version_ID               UINT8_C(14)
#define bhi360_head_feature_param_hgd_get_algo_version_LENGTH           UINT8_C(4)

#define BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_ID           UINT8_C(15)
#define BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_LENGTH       UINT8_C(4)

#define BHI360_HGD_PARAM_CONF_SET_TO_DEFAULT_ID                         UINT8_C(16)
#define BHI360_HGD_PARAM_CONF_SET_TO_DEFAULT_LENGTH                     UINT8_C(4)

#define BHI360_HGD_PARAM_CONF_THRES_PRIMARY_MOTION_ANGULAR_RATE_ID      UINT8_C(17)
#define BHI360_HGD_PARAM_CONF_THRES_PRIMARY_MOTION_ANGULAR_RATE_LENGTH  UINT8_C(4)

#define BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_TILT_ID      UINT8_C(18)
#define BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_TILT_LENGTH  UINT8_C(4)

/*! Structure for Head Misalignment Configuration */
typedef struct
{
    uint8_t still_phase_max_dur;
    uint8_t still_phase_min_dur;
    uint8_t still_phase_max_samples;
    int32_t acc_diff_threshold;
    uint8_t reserved;
} BHI360_PACKED bhi360_head_feature_param_misalignment_config;

/*! Structure for Head Feature /Head Misalignment version */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t reserved;
} BHI360_PACKED bhi360_head_feature_param_ver;

/*! Structure for Head Misalignment Quaternion Correction */
typedef struct
{
    union bhi360_float_conv quaternion_x;
    union bhi360_float_conv quaternion_y;
    union bhi360_float_conv quaternion_z;
    union bhi360_float_conv quaternion_w;
    union bhi360_float_conv accuracy;
} BHI360_PACKED bhi360_head_feature_param_misalignment_quat_corr;

/*! Structure for Head Misalignment Mode and Vector X value */
typedef struct
{
    uint8_t mode;
    union bhi360_float_conv vector_x_0;
    union bhi360_float_conv vector_x_1;
    union bhi360_float_conv vector_x_2;
    uint8_t reserved[3];
} BHI360_PACKED bhi360_head_feature_param_misalignment_mode_vector_x;

typedef struct
{
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t major_bug_fix;
    uint8_t reserved;
} BHI360_PACKED bhi360_head_gesture_algo_version_t;

typedef struct
{
    union bhi360_float_conv time_duration_of_one_gesture;
} BHI360_PACKED bhi360_hgd_time_duration_of_one_gesture_t;

typedef struct
{
    union bhi360_float_conv time_duration_of_one_gesture_tilt;
} BHI360_PACKED bhi360_hgd_time_duration_of_one_gesture_tilt_t;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHI360_HEAD_FEATURE_PARAM_DEFS_H_ */
