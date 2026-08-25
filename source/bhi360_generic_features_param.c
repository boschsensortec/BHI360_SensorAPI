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
* @file       bhi360_generic_features_param.c
* @date       2026-08-14
* @version    v2.3.1
*
*/

#include "bhi360_generic_features_param.h"
#include "bhi360_hif.h"
#include "bhi360.h"

static int8_t bhi360_get_generic_feature_ctrl_info(uint8_t sensor_id,
                                                   uint8_t ctrl_code,
                                                   uint8_t *payload,
                                                   uint32_t payload_len,
                                                   uint32_t *actual_len,
                                                   struct bhi360_hif_dev *hif);

static int8_t bhi360_get_generic_feature_ctrl_info(uint8_t sensor_id,
                                                   uint8_t ctrl_code,
                                                   uint8_t *payload,
                                                   uint32_t payload_len,
                                                   uint32_t *actual_len,
                                                   struct bhi360_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status;
    uint8_t tmp_buf[BHI360_GENERIC_FEATURES_PARAM_READ_LENGTH] = { 0 };
    uint8_t buf = 0;
    uint32_t length;
    int8_t rslt;
    uint16_t param_id = BHI360_PARAM_GENERIC_FEATURE_PAGE_BASE | sensor_id;

    /* Change Status FIFO to Sync mode */
    rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    old_status = buf;
    buf &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL));
    rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Execute 'Set parameter' command to request data */
    tmp_buf[0] = ctrl_code | BHI360_GENERIC_FEATURES_PARAM_READ;
    length = BHI360_GENERIC_FEATURES_PARAM_READ_LENGTH;
    rslt = bhi360_hif_set_parameter(param_id, tmp_buf, length, hif);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Execute 'Get parameter' command to read data */
    length = 0U;
    rslt = bhi360_hif_set_parameter(param_id | BHI360_PARAM_READ_MASK, NULL, length, hif);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Wait and process the return data */
    rslt = bhi360_hif_wait_status_ready(hif);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    rslt = bhi360_hif_get_status_fifo(&code, payload, payload_len, actual_len, hif);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Revert previous Status FIFO mode */
    buf = old_status;
    rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &buf, 1, hif);
    if (rslt == BHI360_OK)
    {
        if (code != param_id)
        {
            rslt = BHI360_E_INVALID_PARAM;
        }
    }

    return rslt;
}

/**
 * @brief Function to set Wrist Gesture Detector configuration
 * @param[in] config  : Reference to Wrist Gesture Detector configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_generic_features_param_set_wrist_gesture_cfg(
    const bhi360_generic_features_param_wrist_gesture_detector* config,
    struct bhi360_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI360_PARAM_GENERIC_FEATURE_PAGE_BASE | BHI360_GENERIC_FEATURES_WRIST_GESTURE_SENSOR_ID;
        command = (uint8_t)BHI360_GENERIC_FEATURES_PARAM_WRIST_GESTURE_DETECTOR_CONFIG_COD |
                  BHI360_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi360_hif_exec_cmd_generic(param_id,
                                           (const uint8_t *)config,
                                           BHI360_PHY_PARAM_WRIST_GESTURE_DETECTOR_CTRL_LENGTH,
                                           &command,
                                           1,
                                           0,
                                           &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get Wrist Gesture Detector configuration
 * @param[out] config  : Reference to Wrist Gesture Detector configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_generic_features_param_get_wrist_gesture_cfg(bhi360_generic_features_param_wrist_gesture_detector* config,
                                                           struct bhi360_dev *dev)
{
    int8_t rslt;
    uint8_t command;
    uint32_t act_len;
    uint8_t payload[BHI360_GENERIC_FEATURES_PARAM_WRIST_GESTURE_DETECTOR_PAYLOAD_LENGTH] = { 0U };

    if ((config == NULL) || (dev == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        command = (uint8_t)BHI360_GENERIC_FEATURES_PARAM_WRIST_GESTURE_DETECTOR_CONFIG_COD;
        rslt = bhi360_get_generic_feature_ctrl_info(BHI360_GENERIC_FEATURES_WRIST_GESTURE_SENSOR_ID,
                                                    command,
                                                    payload,
                                                    BHI360_GENERIC_FEATURES_PARAM_WRIST_GESTURE_DETECTOR_PAYLOAD_LENGTH,
                                                    &act_len,
                                                    &dev->hif);
        if (rslt == BHI360_OK)
        {
            config->min_flick_peak_y_thres = BHI360_LE2U16(&payload[0]);
            config->min_flick_peak_z_thres = BHI360_LE2U16(&payload[2]);
            config->gravity_bounds_x_pos = BHI360_LE2U16(&payload[4]);
            config->gravity_bounds_x_neg = BHI360_LE2U16(&payload[6]);
            config->gravity_bounds_y_neg = BHI360_LE2U16(&payload[8]);
            config->gravity_bounds_z_neg = BHI360_LE2U16(&payload[10]);
            config->flick_peak_decay_coeff = BHI360_LE2U16(&payload[12]);
            config->lp_mean_filter_coeff = BHI360_LE2U16(&payload[14]);
            config->max_duration_jiggle_peaks = BHI360_LE2U16(&payload[16]);
            config->device_pos = BHI360_LE2U16(&payload[18]);
        }
    }

    return rslt;
}

/**
 * @brief Function to reset Wrist Gesture Detector configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_generic_features_param_reset_wrist_gesture_cfg(struct bhi360_dev *dev)
{
    int8_t rslt;
    uint16_t param_id;
    uint8_t command;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        param_id = (uint16_t)BHI360_PARAM_GENERIC_FEATURE_PAGE_BASE | BHI360_GENERIC_FEATURES_WRIST_GESTURE_SENSOR_ID;
        command = (uint8_t)BHI360_GENERIC_FEATURES_PARAM_WRIST_GESTURE_DETECTOR_RESET_COD |
                  BHI360_PARAM_PHY_SENSOR_CTRL_WRITE;
        rslt = bhi360_hif_exec_cmd_generic(param_id, NULL, 0, &command, 1, 0, &dev->hif);
    }

    return rslt;
}
