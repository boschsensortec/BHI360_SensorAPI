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
* @file       bhi360_phy_sensor_ctrl_param_defs.h
* @date       2026-08-14
* @version    v2.3.1
*
*/

#ifndef __BHI360_PHY_SENSOR_CTRL_PARAM_DEFS_H__
#define __BHI360_PHY_SENSOR_CTRL_PARAM_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_defs.h"

/*! Physical Sensor Control Parameter Page Base Address*/
#define BHI360_PARAM_PHY_SENSOR_CTRL_PAGE_BASE                           UINT16_C(0xE00)

#define BHI360_PARAM_PHY_SENSOR_CTRL_READ                                UINT8_C(0x80)
#define BHI360_PARAM_PHY_SENSOR_CTRL_WRITE                               UINT8_C(0x00)

/* Control code */
#define BHI360_PHY_PARAM_ACCEL_FAST_OFFSET_CALIB_COD                     UINT8_C(0x01)
#define BHI360_PHY_PARAM_ACCEL_LOW_POWER_MODE_COD                        UINT8_C(0x05)
#define BHI360_PHY_PARAM_ACCEL_AXIS_REMAP_COD                            UINT8_C(0x07)
#define BHI360_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_COD                     UINT8_C(0x08)
#define BHI360_PHY_PARAM_GYRO_FAST_OFFSET_CALIB_COD                      UINT8_C(0x01)
#define BHI360_PHY_PARAM_GYRO_OIS_ENABLE_COD                             UINT8_C(0x02)
#define BHI360_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_COD                    UINT8_C(0x03)
#define BHI360_PHY_PARAM_GYRO_COMP_RETRIM_COD                            UINT8_C(0x04)
#define BHI360_PHY_PARAM_GYRO_PERF_MODE_COD                              UINT8_C(0x05)
#define BHI360_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_COD                        UINT8_C(0x06)
#define BHI360_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_COD                      UINT8_C(0x08)
#define BHI360_PHY_PARAM_GYRO_MANUAL_SENSITIVITY_COMP_COD                UINT8_C(0x09)
#define BHI360_PHY_PARAM_MAGNET_POWER_MODE_COD                           UINT8_C(0x05)
#define BHI360_PHY_PARAM_WRIST_WEAR_WAKEUP_COD                           UINT8_C(0x01)
#define BHI360_PHY_PARAM_ANY_MOTION_COD                                  UINT8_C(0x01)
#define BHI360_PHY_PARAM_NO_MOTION_COD                                   UINT8_C(0x01)
#define BHI360_PHY_PARAM_WRIST_GESTURE_DETECTOR_COD                      UINT8_C(0x07)
#define BHI360_PHY_PARAM_BARO_PRESSURE_COD                               UINT8_C(0x01)
#define BHI360_PHY_PARAM_STEP_COUNTER_COD                                UINT8_C(0x01)

/* Control length : payload + control code */
#define BHI360_PHY_PARAM_ACCEL_FAST_OFFSET_CALIB_CTRL_LENGTH             UINT8_C(7)
#define BHI360_PHY_PARAM_ACCEL_LOW_POWER_MODE_CTRL_LENGTH                UINT8_C(2)
#define BHI360_PHY_PARAM_ACCEL_AXIS_REMAP_CTRL_LENGTH                    UINT8_C(7)
#define BHI360_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_CTRL_LENGTH             UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_FAST_OFFSET_CALIB_CTRL_LENGTH              UINT8_C(7)
#define BHI360_PHY_PARAM_GYRO_OIS_ENABLE_CTRL_LENGTH                     UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE_CTRL_LENGTH            UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_COMP_RETRIM_CTRL_LENGTH                    UINT8_C(5)
#define BHI360_PHY_PARAM_GYRO_PERF_MODE_CTRL_LENGTH                      UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_CTRL_LENGTH                UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_CTRL_LENGTH              UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_MANUAL_SENSITIVITY_COMP_CTRL_LENGTH        UINT8_C(7)
#define BHI360_PHY_PARAM_MAGNET_POWER_MODE_CTRL_LENGTH                   UINT8_C(2)
#define BHI360_PHY_PARAM_WRIST_WEAR_WAKEUP_CTRL_LENGTH                   UINT8_C(17)
#define BHI360_PHY_PARAM_ANY_MOTION_CTRL_LENGTH                          UINT8_C(5)
#define BHI360_PHY_PARAM_NO_MOTION_CTRL_LENGTH                           UINT8_C(5)
#define BHI360_PHY_PARAM_WRIST_GESTURE_DETECTOR_CTRL_LENGTH              UINT8_C(20)
#define BHI360_PHY_PARAM_BARO_PRESSURE_TYPE_1_CTRL_LENGTH                UINT8_C(3)
#define BHI360_PHY_PARAM_BARO_PRESSURE_TYPE_2_CTRL_LENGTH                UINT8_C(4)
#define BHI360_PHY_PARAM_STEP_COUNTER_CTRL_LENGTH                        UINT8_C(51)

/* Accelerometer control parameter */
#define BHI360_PHY_PARAM_ACCEL_NORMAL_MODE                               UINT8_C(0)
#define BHI360_PHY_PARAM_ACCEL_LOW_POWER_MODE                            UINT8_C(2)
#define BHI360_PHY_PARAM_ACCEL_NVM_WRITE_TRIGGER_ENABLE                  UINT8_C(2)
#define BHI360_PHY_PARAM_ACCEL_NVM_WRITE_STATUS_DONE                     UINT8_C(0)
#define BHI360_PHY_PARAM_ACCEL_NVM_WRITE_STATUS_IN_PROGRESS              UINT8_C(1)

/* Gyroscope control parameter */
#define BHI360_PHY_PARAM_GYRO_OIS_DISABLE                                UINT8_C(0)
#define BHI360_PHY_PARAM_GYRO_OIS_ENABLE                                 UINT8_C(1)
#define BHI360_PHY_PARAM_GYRO_FAST_STARTUP_DISABLE                       UINT8_C(0)
#define BHI360_PHY_PARAM_GYRO_FAST_STARTUP_ENABLE                        UINT8_C(1)
#define BHI360_PHY_PARAM_GYRO_COMP_RETRIM_SUCCESS                        UINT8_C(0)
#define BHI360_PHY_PARAM_GYRO_COMP_RETRIM_START                          UINT8_C(1)
#define BHI360_PHY_PARAM_GYRO_NORMAL_MODE                                UINT8_C(0)
#define BHI360_PHY_PARAM_GYRO_PERF_MODE                                  UINT8_C(1)
#define BHI360_PHY_PARAM_GYRO_LOW_POWER_MODE                             UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_DISABLE                    UINT8_C(0)
#define BHI360_PHY_PARAM_GYRO_TIMER_AUTO_TRIM_START                      UINT8_C(1)
#define BHI360_PHY_PARAM_GYRO_NVM_WRITE_TRIGGER_ENABLE                   UINT8_C(2)
#define BHI360_PHY_PARAM_GYRO_NVM_WRITE_STATUS_DONE                      UINT8_C(0)
#define BHI360_PHY_PARAM_GYRO_NVM_WRITE_STATUS_IN_PROGRESS               UINT8_C(1)

/* Magnetometer control parameter */
#define BHI360_PHY_PARAM_MAGNET_NORMAL_MODE                              UINT8_C(0)
#define BHI360_PHY_PARAM_MAGNET_LOW_POWER_MODE                           UINT8_C(2)

/* Baro control parameter */
#define BHI360_PHY_PARAM_BARO_NO_OVERSAMPLING                            UINT8_C(0)
#define BHI360_PHY_PARAM_BARO_X2_OVERSAMPLING                            UINT8_C(1)
#define BHI360_PHY_PARAM_BARO_X4_OVERSAMPLING                            UINT8_C(2)
#define BHI360_PHY_PARAM_BARO_X8_OVERSAMPLING                            UINT8_C(3)
#define BHI360_PHY_PARAM_BARO_X16_OVERSAMPLING                           UINT8_C(4)
#define BHI360_PHY_PARAM_BARO_X32_OVERSAMPLING                           UINT8_C(5)

#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_0                             UINT8_C(0)
#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_1                             UINT8_C(1)
#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_3                             UINT8_C(2)
#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_7                             UINT8_C(3)
#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_15                            UINT8_C(4)
#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_31                            UINT8_C(5)
#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_63                            UINT8_C(6)
#define BHI360_PHY_PARAM_BARO_FILTER_COEFF_127                           UINT8_C(7)

#define BHI360_PHY_SENSOR_CTRL_PARAM_READ_LENGTH                         UINT8_C(4)

#define BHI360_PHY_SENSOR_CTRL_PARAM_CODE_LENGTH                         UINT8_C(1)

#define BHI360_PHY_PARAM_ANY_MOTION_DURATION_UPPER_MASK                  UINT16_C(0x1F)
#define BHI360_PHY_PARAM_ANY_MOTION_AXIS_SEL_SHIFT                       UINT16_C(0x05)
#define BHI360_PHY_PARAM_ANY_MOTION_AXIS_SEL_MASK                        UINT16_C(0x07)
#define BHI360_PHY_PARAM_ANY_MOTION_THRESHOLD_UPPER_MASK                 UINT16_C(0x07)

#define BHI360_PHY_PARAM_NO_MOTION_DURATION_UPPER_MASK                   UINT16_C(0x1F)
#define BHI360_PHY_PARAM_NO_MOTION_AXIS_SEL_SHIFT                        UINT16_C(0x05)
#define BHI360_PHY_PARAM_NO_MOTION_AXIS_SEL_MASK                         UINT16_C(0x07)
#define BHI360_PHY_PARAM_NO_MOTION_THRESHOLD_UPPER_MASK                  UINT16_C(0x07)

#define BHI360_PHY_PARAM_BARO_OSR_PRESSURE_SHIFT                         UINT16_C(0x00)
#define BHI360_PHY_PARAM_BARO_OSR_PRESSURE_MASK                          UINT16_C(0x07)
#define BHI360_PHY_PARAM_BARO_OSR_TEMP_SHIFT                             UINT16_C(0x03)
#define BHI360_PHY_PARAM_BARO_OSR_TEMP_MASK                              UINT16_C(0x07)
#define BHI360_PHY_PARAM_BARO_IIR_FILTER_SHIFT                           UINT16_C(0x00)
#define BHI360_PHY_PARAM_BARO_IIR_FILTER_MASK                            UINT16_C(0x07)
#define BHI360_PHY_PARAM_BARO_IIR_FILTER_PRESSURE_SHIFT                  UINT16_C(0x00)
#define BHI360_PHY_PARAM_BARO_IIR_FILTER_PRESSURE_MASK                   UINT16_C(0x07)
#define BHI360_PHY_PARAM_BARO_IIR_FILTER_TEMP_SHIFT                      UINT16_C(0x03)
#define BHI360_PHY_PARAM_BARO_IIR_FILTER_TEMP_MASK                       UINT16_C(0x07)

#define BHI360_PHY_PARAM_GYRO_MANUAL_SENSITIVITY_COMP_GAIN_UPDATE_MASK   UINT16_C(0x07FF)
#define BHI360_PHY_PARAM_GYRO_MANUAL_SENSITIVITY_COMP_GAIN_UPDATE_SHIFT  UINT8_C(8U)

#define BHI360_PHY_PARAM_MASK_8BIT                                       UINT16_C(0xFFU)
#define BHI360_PHY_PARAM_MASK_3BIT                                       UINT8_C(0x7U)

/*!
 *
 * @brief bhy accel fast offset calibration
 *
 */
typedef struct
{
    int16_t x_offset;
    int16_t y_offset;
    int16_t z_offset;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_accel_fast_offset_calib;

/*!
 *
 * @brief bhy accel axis remapping for internal imu features
 *
 */
typedef struct
{
    uint8_t map_x_axis;
    uint8_t map_x_axis_sign;
    uint8_t map_y_axis;
    uint8_t map_y_axis_sign;
    uint8_t map_z_axis;
    uint8_t map_z_axis_sign;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_accel_axis_remap;

/*!
 *
 * @brief bhy gyro fast offset calibration
 *
 */
typedef struct
{
    int16_t x_offset;
    int16_t y_offset;
    int16_t z_offset;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_gyro_fast_offset_calib;

/*!
 *
 * @brief bhy Gyroscope Component Retrim
 *
 */
typedef struct
{
    uint8_t status;
    uint8_t x;
    uint8_t y;
    uint8_t z;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_gyro_crt_data;

typedef struct
{
    uint16_t gain_update_x : 11;
    uint16_t gain_update_y : 11;
    uint16_t gain_update_z : 11;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_gyro_manual_sensitivity_comp;

/*!
 *
 * @brief bhy wrist wear wakeup control
 *
 */
typedef struct
{
    uint16_t min_angle_focus; /* Cosine of minimum expected attitude change within \
                                  1 second time window when moving with focus position */
    uint16_t min_angle_non_focus; /* Cosine of minimum expected attitude change within 1 second time window \
                                        when moving from non-focus to focus position */
    uint8_t angle_landscape_right; /* Sine of maximum allowed tilt angle in landscape right direction */
    uint8_t angle_landscape_left; /* Sine of maximum allowed tilt angle in landscape left direction */
    uint8_t angle_portrait_down; /* Sine of maximum allowed backward tilt angle in portrait down direction */
    uint8_t angle_portrait_up; /* Sine of maximum allowed forward tilt angle in portrait up direction */
    uint8_t min_dur_moved; /* Minimum duration the arm should be moved while performing gesture */
    uint8_t min_dur_quite; /* Minimum duration the arm should be static between 2 consecutive gestures */
    uint8_t angle_landscape_right_drop; /* Sine of the minimum allowed tilt angle in landscape right direction of the
                                         * device for drop detection */
    uint8_t angle_landscape_left_drop; /* Sine of minimum expected tilt angle in landscape left direction of the device
                                        * for drop detection */
    uint8_t angle_portrait_down_drop; /* Sine of minimum expected backward tilt angle in portrait down direction of the
                                       * device for drop detection */
    uint8_t angle_portrait_up_drop; /* Sine of minimum expected forward tilt angle in portrait up direction of the
                                     * device for drop detection */
    uint8_t mac_acc_z_drop; /* Cosine of minimum allowed forward tilt angle in portrait up direction of the device for
                             * drop detection */
    uint8_t reserved; /* Reserved for future use, should be set to 0 */
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_wrist_wear_wakeup;

/*!
 *
 * @brief bhy any motion control
 *
 */
typedef struct
{
    uint16_t duration : 13; /* Number of consecutive dat points the threshold condition must be respected */
    uint16_t axis_sel : 3; /* Axis selection */
    uint16_t threshold : 11; /* Slope threshold for any motion detection */
    uint16_t reserved : 5;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_any_motion;

/*!
 *
 * @brief bhy no motion control
 *
 */
typedef struct
{
    uint16_t duration : 13; /* Number of consecutive dat points the threshold condition must be respected */
    uint16_t axis_sel : 3; /* Axis selection */
    uint16_t threshold : 11; /* Slope threshold for no motion detection */
    uint16_t reserved : 5;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_no_motion;

/*!
 *
 * @brief bhy barometer pressure type 1
 *
 */
typedef struct
{
    uint8_t osr_p : 3; /* Oversampling setting pressure measurement */
    uint8_t osr_t : 3; /* Oversampling setting temperature measurement */
    uint8_t reserved_1 : 2;
    uint8_t iir_filter : 3; /* Filter coefficient for IIR filter */
    uint8_t reserved_2 : 5;
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_baro_type_1; /* For BMP390 */

/*!
 *
 * @brief bhy barometer pressure type 2
 *
 */
typedef struct
{
    uint8_t osr_p : 3; /* Oversampling setting pressure measurement */
    uint8_t osr_t : 3; /* Oversampling setting temperature measurement */
    uint8_t reserved_1 : 2;
    uint8_t iir_filter_p : 3; /* Filter coefficient for pressure IIR filter */
    uint8_t iir_filter_t : 3; /* Filter coefficient for temperature IIR filter */
    uint8_t reserved_2 : 2;
    uint8_t dsp_config; /* DSP configuration */
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_baro_type_2; /* For BMP581 */

/*!
 *
 * @brief bhy step counter control
 *
 */
typedef struct
{
    uint16_t env_min_dist_up; /* threshold to up envelope curve */
    uint16_t env_coef_up; /* decay filter coefficient to up envelope curve */
    uint16_t env_min_dist_down; /* threshold to down envelope curve */
    uint16_t env_coef_down; /* decay filter coefficient to down envelope curve */
    uint16_t step_buffer_size; /* buffer size to step count, step can be count only buffer is full */
    uint16_t mean_val_decay; /* decay filter coefficient to mean value */
    uint16_t mean_step_dur; /* decay filter coefficient to step duration */
    int16_t filter_coeff_b2; /* filter coefficient to input signal */
    int16_t filter_coeff_b1; /* filter coefficient to input signal */
    int16_t filter_coeff_b0; /* filter coefficient to input signal */
    int16_t filter_coeff_a2; /* filter coefficient to input signal */
    int16_t filter_coeff_a1; /* filter coefficient to input signal */
    uint16_t filter_cascade_enabled; /* enable cascade filter for input signal */
    uint16_t peak_duration_min_walking; /* minimal peak duration for walking activity */
    uint16_t peak_duration_min_running; /* minimal peak duration for running activity */
    uint16_t step_duration_max; /* maximal samples since last step to reset step buffer and recognize activity as still
                                 * */
    uint16_t step_duration_window; /* step duration threshold to determine if one step is missed */
    uint16_t half_step_enabled; /* enable half step compensation function */
    uint16_t activity_detection_factor; /* gain for signal magnitude variance in activity identify function */
    uint16_t activity_detection_thres; /* threshold for signal magnitude variance in activity identify function */
    uint16_t step_counter_increment; /* scale factor of step counter */
    uint16_t step_duration_pp_enabled; /* enable post-process of duration between steps */
    uint16_t step_dur_thres; /* gain threshold of mean step duration in post-processing check */
    uint16_t en_mcr_pp; /* enable post-process of mean crossings */
    uint16_t mcr_thres; /* threshold of mean crossings in post-processing check */
} BHI360_PACKED bhi360_phy_sensor_ctrl_param_step_counter;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_PHY_SENSOR_CTRL_PARAM_DEFS_H__ */
