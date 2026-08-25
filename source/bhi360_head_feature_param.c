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
* @file       bhi360_head_feature_param.c
* @date       2026-08-14
* @version    v2.3.1
*
*/

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <string.h>
#include <stdlib.h>

/*********************************************************************/
/* BHY SensorAPI header files */
/*********************************************************************/
#include "bhi360.h"

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "bhi360_head_feature_param.h"

#define BHI360_ROUND_UP_4_MUL(x)  (((x) % 4) ? (uint16_t)((((x) / 4) + 1) * \
                                                          4) : (uint16_t)(x))

/*lint -e506 */

/*!
 * @brief To trigger the Head Misalignment Calibration
 *
 * @param[in] dev  Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_trigger_hmc_calibration(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_HMC_TRIGGER_ID;
    uint8_t trig_set[BHI360_HMC_PARAM_HMC_TRIGGER_LENGTH] = { 0 };

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        trig_set[0] = BHI360_HMC_PARAM_HMC_TRIGGER_CALIB_SET;
        rslt = bhi360_set_parameter(param_id, (const uint8_t*)trig_set, BHI360_HMC_PARAM_HMC_TRIGGER_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Calibration Configuration
 *
 * @param[in] config  Reference to hold the Head Misalignment Calibration Configuration
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_set_hmc_configuration(const bhi360_head_feature_param_misalignment_config *config,
                                                       struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_HMC_CONFIG_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_set_parameter(param_id, (const uint8_t*)config, BHI360_HMC_PARAM_HMC_CONFIG_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Calibration Configuration
 *
 * @param[out] config  Reference to hold the Head Misalignment Calibration Configuration
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_get_hmc_configuration(bhi360_head_feature_param_misalignment_config *config,
                                                       struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t act_len;
    uint8_t buffer[BHI360_HMC_PARAM_HMC_CONFIG_LENGTH] = { 0U };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_HMC_CONFIG_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id, buffer, BHI360_HMC_PARAM_HMC_CONFIG_LENGTH, &act_len, dev);
        if (rslt == BHI360_OK)
        {
            config->still_phase_max_dur = buffer[0];
            config->still_phase_min_dur = buffer[1];
            config->still_phase_max_samples = buffer[2];
            config->acc_diff_threshold = BHI360_LE2S32(&buffer[3]);
        }
    }

    return rslt;
}

/*!
 * @brief To set Head Misalignment Calibration Configuration to default
 *
 * @param[in] dev  Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_set_default_hmc_cfg(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_HMC_SET_CONFIG_TO_DEFAULT_ID;
    uint8_t default_set[BHI360_HMC_PARAM_HMC_SET_CONFIG_TO_DEFAULT_LENGTH] = { 0 };

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        default_set[0] = BHI360_HMC_PARAM_HMC_SET_CONFIG_TO_DEFAULT_SET;
        rslt = bhi360_set_parameter(param_id,
                                    (const uint8_t*)default_set,
                                    BHI360_HMC_PARAM_HMC_SET_CONFIG_TO_DEFAULT_LENGTH,
                                    dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Calibrator Version
 *
 * @param[out] hmc_version  Reference to hold the Head Misalignment Calibrator Version
 * @param[in] dev           Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_get_hmc_version(bhi360_head_feature_param_ver *hmc_version, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t act_len;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_GET_ALGO_VERSION_ID;

    if ((hmc_version == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    (uint8_t*)hmc_version,
                                    BHI360_HMC_PARAM_GET_ALGO_VERSION_LENGTH,
                                    &act_len,
                                    dev);
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[in] config  Reference to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_set_hmc_quat_cal_cor_cfg(
    const bhi360_head_feature_param_misalignment_quat_corr *config,
    struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhi360_set_parameter(param_id, (uint8_t*)config, BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_WLENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[out] config  Reference to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_get_hmc_quat_cal_cor_cfg(bhi360_head_feature_param_misalignment_quat_corr *config,
                                                          struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t act_len;
    uint8_t buffer[BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_RLENGTH] = { 0U };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhi360_get_parameter(param_id, buffer, BHI360_HMC_PARAM_CALI_CORRECT_QUARTER_CONF_RLENGTH, &act_len, dev);
        if (rslt == BHI360_OK)
        {
            config->quaternion_x.u32_val = BHI360_LE2U32(&buffer[0]);
            config->quaternion_y.u32_val = BHI360_LE2U32(&buffer[4]);
            config->quaternion_z.u32_val = BHI360_LE2U32(&buffer[8]);
            config->quaternion_w.u32_val = BHI360_LE2U32(&buffer[12]);
            config->accuracy.u32_val = BHI360_LE2U32(&buffer[16]);
        }
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Mode and Vector X value
 *
 * @param[in] config  Reference to hold the Head Misalignment Mode and Vector X value
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_set_hmc_mode_vector_x(
    const bhi360_head_feature_param_misalignment_mode_vector_x *config,
    struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_SET_MODE_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhi360_set_parameter(param_id,
                                 (const uint8_t*)config,
                                 BHI360_ROUND_UP_4_MUL(BHI360_HMC_PARAM_SET_MODE_LENGTH),
                                 dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Mode and Vector X value
 *
 * @param[out] config  Reference to hold the Head Misalignment Mode and Vector X value
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_get_hmc_mode_vector_x(bhi360_head_feature_param_misalignment_mode_vector_x *config,
                                                       struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t act_len;
    uint8_t buffer[BHI360_ROUND_UP_4_MUL(BHI360_HMC_PARAM_SET_MODE_LENGTH)] = { 0U };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HMC_PARAM_SET_MODE_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    buffer,
                                    BHI360_ROUND_UP_4_MUL(BHI360_HMC_PARAM_SET_MODE_LENGTH),
                                    &act_len,
                                    dev);
        if (rslt == BHI360_OK)
        {
            config->mode = buffer[0];
            config->vector_x_0.u32_val = BHI360_LE2U32(&buffer[1]);
            config->vector_x_1.u32_val = BHI360_LE2U32(&buffer[5]);
            config->vector_x_2.u32_val = BHI360_LE2U32(&buffer[9]);
        }
    }

    return rslt;
}

/*!
 * @brief To set the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[in] config  Reference to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_set_quat_init_head_corr(const uint8_t *config, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_set_parameter(param_id, config, BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[out] config  Reference to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_get_quat_init_head_corr(uint8_t *config, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t act_len;
    uint8_t buffer[BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_LENGTH] = { 0U };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id, buffer, BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_LENGTH, &act_len, dev);
        if (rslt == BHI360_OK)
        {
            *config = buffer[0];
        }
    }

    return rslt;
}

/*!
 * @brief To get the IMU/NDOF Head Orientation Version
 *
 * @param[out] ho_version Reference to hold the IMU/NDOF Head Orientation Version
 * @param[in] dev         Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_get_ho_version(bhi360_head_feature_param_ver *ho_version, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t act_len;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_OC_PARAM_GET_ALGO_VERSION_ID;

    if ((ho_version == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    (uint8_t*)ho_version,
                                    BHI360_OC_PARAM_GET_ALGO_VERSION_LENGTH,
                                    &act_len,
                                    dev);
    }

    return rslt;
}

/*!
 * @brief To get the Head Gesture Detector Algorithm Version
 *
 * @param[out] algo_version Reference to hold the Head Gesture Detector Algorithm Version
 * @param[in] dev           Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_get_algo_version(bhi360_head_gesture_algo_version_t *algo_version,
                                                      struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[4] = { 0 };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + bhi360_head_feature_param_hgd_get_algo_version_ID;

    if ((algo_version == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    buffer,
                                    bhi360_head_feature_param_hgd_get_algo_version_LENGTH,
                                    &bytes_read,
                                    dev);

        if (rslt == BHI360_OK)
        {
            algo_version->major_version = buffer[0]; /* Head Gesture Detector major version */
            algo_version->minor_version = buffer[1]; /* Head Gesture Detector minor version */
            algo_version->major_bug_fix = buffer[2]; /* Head Gesture Detector major bug fix version */
        }
    }

    return rslt;
}

/*!
 * @brief To get the time duration of one gesture
 *
 * @param[out] time_dur Reference to hold the time duration of one gesture
 * @param[in] dev       Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_get_time_duration_of_one_gesture(
    bhi360_hgd_time_duration_of_one_gesture_t *time_dur,
    struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[16] = { 0 };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_ID;

    if ((time_dur == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    buffer,
                                    BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_LENGTH,
                                    &bytes_read,
                                    dev);

        if (rslt == BHI360_OK)
        {
            time_dur->time_duration_of_one_gesture.u32_val = BHI360_LE2U32(buffer);
        }
    }

    return rslt;
}

/*!
 * @brief To set the time duration of one gesture
 *
 * @param[in] time_dur Reference to hold the time duration of one gesture
 * @param[in] dev       Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_set_time_duration_of_one_gesture(
    const bhi360_hgd_time_duration_of_one_gesture_t *time_dur,
    struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_ID;

    if ((time_dur == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_set_parameter(param_id,
                                    (uint8_t*)time_dur,
                                    BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_LENGTH,
                                    dev);
    }

    return rslt;
}

/*!
 * @brief To set the Head Gesture Detector parameters to default
 *
 * @param[in] value  Value to set the parameters to default
 * @param[in] dev    Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_set_to_default(uint8_t value, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_HGD_PARAM_CONF_SET_TO_DEFAULT_ID;
    uint8_t buffer[4] = { 0 };

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        buffer[0] = value; /* Set to default for all the Head Gesture Detector parameters */
        rslt = bhi360_set_parameter(param_id, buffer, BHI360_HGD_PARAM_CONF_SET_TO_DEFAULT_LENGTH, dev);
    }

    return rslt;
}

/*!
 * @brief To get the threshold of primary motion angular rate for head gesture detection
 *
 * @param[out] thres Reference to hold the threshold of primary motion angular rate
 * @param[in] dev    Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_get_thres_primary_motion_angular_rate(uint8_t *thres, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[4] = { 0 };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE +
                        BHI360_HGD_PARAM_CONF_THRES_PRIMARY_MOTION_ANGULAR_RATE_ID;

    if ((thres == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    buffer,
                                    BHI360_HGD_PARAM_CONF_THRES_PRIMARY_MOTION_ANGULAR_RATE_LENGTH,
                                    &bytes_read,
                                    dev);

        if (rslt == BHI360_OK)
        {
            *thres = buffer[0]; /* Threshold of primary motion angular rate for head gesture detection */
        }
    }

    return rslt;
}

/*!
 * @brief To set the threshold of primary motion angular rate for head gesture detection
 *
 * @param[in] thres Threshold of primary motion angular rate for head gesture detection
 * @param[in] dev    Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_set_thres_primary_motion_angular_rate(uint8_t thres, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE +
                        BHI360_HGD_PARAM_CONF_THRES_PRIMARY_MOTION_ANGULAR_RATE_ID;
    uint8_t buffer[4] = { 0 };

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        buffer[0] = thres; /* Threshold of primary motion angular rate for head gesture detection */
        rslt = bhi360_set_parameter(param_id,
                                    buffer,
                                    BHI360_HGD_PARAM_CONF_THRES_PRIMARY_MOTION_ANGULAR_RATE_LENGTH,
                                    dev);
    }

    return rslt;
}

/*!
 * @brief To get the time duration of one gesture tilt
 *
 * @param[out] time_dur_tilt Reference to hold the time duration of one gesture tilt
 * @param[in] dev            Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_get_time_duration_of_one_gesture_tilt(union bhi360_float_conv *time_dur_tilt,
                                                                           struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[4] = { 0 };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE +
                        BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_TILT_ID;

    if ((time_dur_tilt == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    buffer,
                                    BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_TILT_LENGTH,
                                    &bytes_read,
                                    dev);

        if (rslt == BHI360_OK)
        {
            time_dur_tilt->u32_val = BHI360_LE2U32(buffer);
        }
    }

    return rslt;
}

/*!
 * @brief To set the time duration of one gesture tilt
 *
 * @param[in] time_dur_tilt Reference to hold the time duration of one gesture tilt
 * @param[in] dev            Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_hgd_set_time_duration_of_one_gesture_tilt(union bhi360_float_conv *time_dur_tilt,
                                                                           struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE +
                        BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_TILT_ID;

    if ((time_dur_tilt == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_set_parameter(param_id,
                                    (uint8_t*)&time_dur_tilt->u32_val,
                                    BHI360_HGD_PARAM_CONF_TIME_DURATION_OF_ONE_GESTURE_TILT_LENGTH,
                                    dev);
    }

    return rslt;
}

/*!
 * @brief To set the Initial heading correction setting
 *
 * @param[in] config  Reference to hold the Initial heading correction setting
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_set_ndof_init_head_corr(const uint8_t *config, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_set_parameter(param_id,
                                    (const uint8_t*)config,
                                    BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_LENGTH,
                                    dev);
    }

    return rslt;
}

/*!
 * @brief To get the Initial heading correction setting
 *
 * @param[out] config  Reference to hold the Initial heading correction setting
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_feature_param_get_ndof_init_head_corr(uint8_t *config, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint32_t act_len;
    uint8_t buffer[BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_LENGTH] = { 0U };
    uint16_t param_id = BHI360_HEAD_FEATURE_PARAM_PAGE_BASE + BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_ID;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_parameter(param_id,
                                    buffer,
                                    BHI360_OC_NDOF_PARAM_INITIAL_HEADING_CORRECTION_LENGTH,
                                    &act_len,
                                    dev);
        if (rslt == BHI360_OK)
        {
            *config = buffer[0];
        }
    }

    return rslt;
}