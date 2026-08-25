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
* @file       bhi360_event_data_defs.h
* @date       2026-08-14
* @version    v2.3.1
*
*/

#ifndef __BHI360_EVENT_DATA_DEFS_H__
#define __BHI360_EVENT_DATA_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdio.h>
#include <stdint.h>

#ifndef BHI360_PACKED
#define BHI360_PACKED  __attribute__ ((__packed__))
#endif

struct bhi360_event_data_xyz
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct bhi360_event_data_quaternion
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
    uint16_t accuracy;
};

struct bhi360_event_data_orientation
{
    int16_t heading;
    int16_t pitch;
    int16_t roll;
};

/*! Sensor Structure for Head Orientation Quaternion */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
} BHI360_PACKED bhi360_event_data_head_orientation_quat;

/*! Sensor Structure for Head Orientation Euler */
typedef struct
{
    int16_t heading;
    int16_t pitch;
    int16_t roll;
} BHI360_PACKED bhi360_event_data_head_orientation_eul;

/*!
 * Multi Tap Setting.
 */
typedef enum {
    BHI360_NO_TAP,
    BHI360_SINGLE_TAP,
    BHI360_DOUBLE_TAP,
    BHI360_DOUBLE_SINGLE_TAP,
    BHI360_TRIPLE_TAP,
    BHI360_TRIPLE_SINGLE_TAP,
    BHI360_TRIPLE_DOUBLE_TAP,
    BHI360_TRIPLE_DOUBLE_SINGLE_TAP
} bhi360_event_data_multi_tap;

/*!
 * Multi Tap Output.
 */
static const char * const bhi360_event_data_multi_tap_string_out[] = {
    [BHI360_NO_TAP] = "NO_TAP", [BHI360_SINGLE_TAP] = "SINGLE_TAP", [BHI360_DOUBLE_TAP] = "DOUBLE_TAP",
    [BHI360_DOUBLE_SINGLE_TAP] = "DOUBLE_SINGLE_TAP", [BHI360_TRIPLE_TAP] = "TRIPLE_TAP",
    [BHI360_TRIPLE_SINGLE_TAP] = "TRIPLE_SINGLE_TAP", [BHI360_TRIPLE_DOUBLE_TAP] = "TRIPLE_DOUBLE_TAP",
    [BHI360_TRIPLE_DOUBLE_SINGLE_TAP] = "TRIPLE_DOUBLE_SINGLE_TAP" };                                                   /*lint -e528
                                                                                                           * */

typedef enum
{
    BHI360_HGD_NO_GESTURE,
    BHI360_HGD_SHAKING,
    BHI360_HGD_NODDING,
    BHI360_HGD_HEAD_TILTING
} bhi360_event_data_head_gesture_detector;

static const char * const bhi360_event_data_head_gesture_string_out[] = {
    [BHI360_HGD_NO_GESTURE] = "NO_GESTURE", [BHI360_HGD_SHAKING] = "SHAKING", [BHI360_HGD_NODDING] = "NODDING",
    [BHI360_HGD_HEAD_TILTING] = "HEAD_TILTING" };                                                                                                             /*lint
                                                                                                                                                               * -e528
                                                                                                                                                               * */

enum bhi360_event_data_wrist_gesture_activity {
    BHI360_NO_GESTURE,
    BHI360_WRIST_SHAKE_JIGGLE = 0x03,
    BHI360_FLICK_IN,
    BHI360_FLICK_OUT
};

typedef struct bhi360_event_data_wrist_gesture_detect
{
    enum bhi360_event_data_wrist_gesture_activity wrist_gesture;
} __attribute__ ((packed)) bhi360_event_data_wrist_gesture_detect_t;

static const char * const bhi360_event_data_wrist_gesture_detect_output[] = {
    [BHI360_NO_GESTURE] = "NO_GESTURE", [BHI360_WRIST_SHAKE_JIGGLE] = "WRIST_SHAKE_JIGGLE",
    [BHI360_FLICK_IN] = "FLICK_IN", [BHI360_FLICK_OUT] = "FLICK_OUT" }; /*lint -e528 */

typedef struct
{
    uint16_t iaq;
    uint16_t siaq;
    uint16_t voc;
    uint32_t co2;
    uint8_t iaq_accuracy;
    int16_t comp_temperature;
    uint16_t comp_humidity;
    uint32_t raw_gas;
} BHI360_PACKED bhi360_event_data_iaq_output_t;

typedef struct
{
    uint8_t instance;

    /*!
     * A value of -1 means no new learning has occurred. If the value is >= 0,
     * then a new pattern has been learnt, and reading of this pattern may be
     * performed.
     */
    int8_t index;

    /*!
     * While learning a new pattern, this field counts from 0 to 100. When 100 is
     * reached a new pattern will be learnt. If learning is interrupted, this
     * progress will return to 0, and change reason will be set to indicate
     * why learning was interrupted.
     */
    uint8_t progress;

    /*!
     * | Value | Description                                                            |
     * |:-----:|:-----------------------------------------------------------------------|
     * | 0     | Learning is progressing.                                               |
     * | 1     | Learning was interrupted by a non-repetitive activity.                 |
     * | 2     | Learning was interrupted because no significant movement was detected. |
     */
    uint8_t change_reason;
} BHI360_PACKED bhi360_event_data_klio_learning_t;

typedef struct
{
    uint8_t instance;

    /*! The index of the recognized activity. 255 means no activity was
     * recognized. */
    uint8_t index;

    /*! The current repetition count of the recognized activity. */
    float count;

    /*! The current score of the recognized activity. */
    float score;
} BHI360_PACKED bhi360_event_data_klio_recognition_t;

/*!
 *
 * @brief bhy klio combined data structure
 *
 * When the algorithm generates a new data frame, it is sent in this structure.
 *
 */
typedef struct
{
    bhi360_event_data_klio_learning_t learn;
    bhi360_event_data_klio_recognition_t recognize;
} BHI360_PACKED bhi360_event_data_klio_t;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_EVENT_DATA_DEFS_H__ */
