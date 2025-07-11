/**
* Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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
* @file       bhi360_multi_tap_param.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef _BHI360_MULTI_TAP_PARAM_H_
#define _BHI360_MULTI_TAP_PARAM_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>

#include "bhi360_multi_tap_param_defs.h"

/*!
 * @brief To set the MULTI_TAP configuration parameters
 * @param[in] conf  Reference to store MULTI_TAP configuration
 * @param[in] dev   Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_multi_tap_param_set_config(const bhi360_event_data_multi_tap *conf, struct bhi360_dev *dev);

/*!
 * @brief To get the MULTI_TAP configuration parameters
 *
 * @param[out] conf  Reference to store MULTI_TAP configuration
 * @param[in]  dev   Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_multi_tap_param_get_config(bhi360_event_data_multi_tap *conf, struct bhi360_dev *dev);

/*!
 * @brief To set the MULTI_TAP Detector configuration parameters
 * @param[in] conf  Reference to store MULTI_TAP Detector configuration
 * @param[in] dev   Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_multi_tap_param_detector_set_config(const bhi360_multi_tap_param_detector *conf, struct bhi360_dev *dev);

/*!
 * @brief To get the MULTI_TAP Detector configuration parameters
 *
 * @param[out] conf  Reference to store MULTI_TAP Detector configuration
 * @param[in]  dev   Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_multi_tap_param_detector_get_config(bhi360_multi_tap_param_detector *conf, struct bhi360_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHI360_MULTI_TAP_PARAM_H_ */
