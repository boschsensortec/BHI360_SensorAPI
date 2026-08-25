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
* @file       bhi360_generic_features_ctrl_param.h
* @date       2026-08-14
* @version    v2.3.1
*
*/

#ifndef __BHI360_GENERIC_FEATURES_CTRL_PARAM_H__
#define __BHI360_GENERIC_FEATURES_CTRL_PARAM_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_generic_features_param_defs.h"

/*
* @brief Function to set Wrist Gesture Detector configuration
* @param[in] config  : Reference to Wrist Gesture Detector configuration
* @param[in] dev     : Device instance
* @return API error codes
*/
int8_t bhi360_generic_features_param_set_wrist_gesture_cfg(
    const bhi360_generic_features_param_wrist_gesture_detector* config,
    struct bhi360_dev *dev);

/*
 * @brief Function to get Wrist Gesture Detector configuration
* @param[out] config  : Reference to Wrist Gesture Detector configuration
* @param[in]  dev     : Device instance
* @return API error codes
*/
int8_t bhi360_generic_features_param_get_wrist_gesture_cfg(bhi360_generic_features_param_wrist_gesture_detector* config,
                                                           struct bhi360_dev *dev);

/**
 * @brief Function to reset Wrist Gesture Detector configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_generic_features_param_reset_wrist_gesture_cfg(struct bhi360_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_GENERIC_FEATURES_CTRL_PARAM_H__ */
