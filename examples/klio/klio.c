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
 * @file    klio.c
 * @brief   Klio example for the BHI360
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "bhi360.h"
#include "bhi360_parse.h"
#include "bhi360_klio_param_defs.h"
#include "common.h"
#include "bhi360_virtual_sensor_conf_param.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam_lite_Klio_cyclic.fw.h"

#define PARAM_BUF_LEN       UINT8_C(252)

#define EXECUTE_TIMEOUT_MS  UINT32_C(60000)

typedef struct klio_runtime
{
    struct bhi360_dev *bhy;
    bhi360_klio_param_sensor_state_t sensor_state;
    uint16_t max_patterns;
    uint16_t max_pattern_size;
    uint8_t ignore_insignificant_movement;
    uint8_t pattern_write_back_index;
    float* similarity_result_buf;
    uint8_t* similarity_idx_buf;
} klio_runtime_t;

static void parse_klio(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static float parse_klio_handle_learnt_pattern(const struct bhi360_fifo_parse_data_info *callback_info,
                                              uint32_t s,
                                              uint32_t ns,
                                              klio_runtime_t *klio_rt);
static void print_klio_error(bhi360_klio_param_driver_error_state_t status);
static void print_klio_status(struct bhi360_dev *bhy);

int main(void)
{
    enum bhi360_intf intf;

    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    uint8_t boot_status = 0;
    uint8_t accuracy; /* Accuracy is reported as a meta event. It is being printed alongside the data */

    klio_runtime_t klio_rt = {
        .bhy = &bhy, .sensor_state = /* Will be set by parameter write */
        {
            .learning_enabled = 1, .learning_reset = 1, .recognition_enabled = 1, .recognition_reset = 1
        }, .max_patterns = 0,               /* Will be retrieved by parameter read */
        .max_pattern_size = 0, /* Will be retrieved by parameter read */
        .ignore_insignificant_movement = 1, /* Will be set by parameter write */
        .pattern_write_back_index = 1, /* Used by callback routine */
        .similarity_result_buf = NULL, /* Will be allocated after we know how many patterns are supported */
        .similarity_idx_buf = NULL /* Will be allocated after we know how many patterns are supported */
    };

    /* Example pattern, BHI360 should be pointing upward, held level, and moved up and down at about 1.5Hz */
    uint8_t klio_example_pattern_id = 0;
    uint8_t klio_example_pattern[] = {

        0x52, 0x42, 0x31, 0x06, 0x03, 0x8b, 0xff, 0x3c, 0x40, 0x0a, 0xd7, 0x23, 0x3c, 0x73, 0xfe, 0xa7, 0xc0, 0x38,
        0x44, 0xbd, 0x40, 0xbb, 0x1f, 0xe5, 0x3e, 0x38, 0x6f, 0x30, 0x3f, 0x50, 0x89, 0x74, 0x3f, 0x4d, 0x2a, 0xf8,
        0x3c, 0x45, 0x61, 0xd9, 0x40, 0x6d, 0x21, 0x7f, 0x40, 0xd0, 0x80, 0x8f, 0x3d, 0x9e, 0x39, 0x33, 0xbd, 0x51,
        0xc5, 0x0e, 0x3f, 0x64, 0x94, 0x80, 0x3c, 0xba, 0x90, 0xd2, 0x3e, 0xf8, 0xd8, 0x37, 0xbc, 0xed, 0x50, 0xea,
        0x3d, 0xf4, 0x61, 0x16, 0x3f, 0x75, 0xc9, 0x9b, 0xbe, 0x24, 0x20, 0xf7, 0x3c, 0x91, 0x16, 0x5b, 0xbd, 0x0f,
        0x61, 0x21, 0xbc, 0x23, 0xce, 0x80, 0x3e, 0x46, 0x8c, 0x93, 0x3d, 0x0c, 0x70, 0x16, 0x3e, 0x02, 0xf9, 0x9b,
        0x3a, 0x12, 0x48, 0xbc, 0x3d, 0x2e, 0x1f, 0xba, 0x3d, 0xe9, 0x82, 0xf5, 0xbe, 0xb4, 0xbd, 0xe8, 0x3d, 0xc6,
        0x79, 0x02, 0xbd, 0x8a, 0x1a, 0x00, 0x3b, 0x87, 0x22, 0x81, 0x3e, 0x96, 0x57, 0x05, 0x3e, 0xcb, 0x03, 0xcb,
        0xbf, 0x34, 0x2d, 0x93, 0x3e, 0x26, 0x6c, 0xff, 0xbd, 0x52, 0xb0, 0x84, 0x3b
    };

    uint32_t curr_ts, start_ts;

#ifdef BHI360_USE_I2C
    intf = BHI360_I2C_INTERFACE;
#else
    intf = BHI360_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

    init_sensor(&bhy, intf);

    setup_host_int_ctrl(&bhy);

    /* Check if the sensor is ready to load firmware */
    rslt = bhi360_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI360_BST_HOST_INTERFACE_READY)
    {
        upload_firmware(bhi360_firmware_image, sizeof(bhi360_firmware_image), &bhy);
        rslt = bhi360_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI360_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT,
                                                   bhi360_parse_meta_event,
                                                   (void*)&accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU,
                                                   bhi360_parse_meta_event,
                                                   (void*)&accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_KLIO, parse_klio, (void*)&klio_rt, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
        print_api_error(rslt, &bhy);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    /* Get number of supported patterns */
    uint8_t param_buf[PARAM_BUF_LEN];
    uint16_t length = sizeof(param_buf);

    rslt = bhi360_klio_param_get_parameter(KLIO_PARAM_RECOGNITION_MAX_PATTERNS, param_buf, &length, &bhy);
    print_api_error(rslt, &bhy);
    print_klio_status(&bhy);
    memcpy(&klio_rt.max_patterns, param_buf, sizeof(klio_rt.max_patterns));
    klio_rt.similarity_result_buf = malloc(sizeof(float) * klio_rt.max_patterns);
    klio_rt.similarity_idx_buf = malloc(sizeof(uint8_t) * klio_rt.max_patterns);

    if (klio_rt.similarity_result_buf == NULL || klio_rt.similarity_idx_buf == NULL)
    {
        printf("Unable to allocate Klio buffers. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    /* Get maximum supported pattern size */
    length = sizeof(param_buf);
    rslt = bhi360_klio_param_get_parameter(KLIO_PARAM_PATTERN_BLOB_SIZE, param_buf, &length, &bhy);
    print_api_error(rslt, &bhy);
    print_klio_status(&bhy);
    memcpy(&klio_rt.max_pattern_size, param_buf, sizeof(klio_rt.max_pattern_size));

    /* Set klio state (learning/recognition enable/disable and reset) */
    rslt = bhi360_klio_param_set_state(&klio_rt.sensor_state, &bhy);
    print_api_error(rslt, &bhy);
    print_klio_status(&bhy);

    /* Prevent learning with small movements, parameter writes should be done after reset and before sensor enable */
    rslt = bhi360_klio_param_set_parameter(KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT,
                                           &klio_rt.ignore_insignificant_movement,
                                           sizeof(klio_rt.ignore_insignificant_movement),
                                           &bhy);
    print_api_error(rslt, &bhy);
    print_klio_status(&bhy);

    /* Write example pattern */
    rslt = bhi360_klio_param_write_pattern(klio_example_pattern_id,
                                           klio_example_pattern,
                                           sizeof(klio_example_pattern),
                                           &bhy);
    print_api_error(rslt, &bhy);
    print_klio_status(&bhy);

    /*Enables the pattern for recognition, enabling should be done after writing the pattern*/
    rslt = bhi360_klio_param_set_pattern_states(KLIO_PATTERN_STATE_ENABLE, &klio_example_pattern_id, 1, &bhy);
    print_api_error(rslt, &bhy);
    print_klio_status(&bhy);

    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    sensor_conf.sample_rate = BHI360_ODR_25_HZ; /* Read out data measured at 25Hz */
    sensor_conf.latency = 0; /* Report immediately */

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_KLIO, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_KLIO), sensor_conf.sample_rate);
    printf("Move device up and down / left to right in repetition in normal rate \r\n");

    start_ts = coines_get_millis();
    while (rslt == BHI360_OK)
    {
        curr_ts = coines_get_millis();
        if ((curr_ts - start_ts) >= EXECUTE_TIMEOUT_MS)
        {
            break;
        }

        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);
        }
    }

    free(klio_rt.similarity_result_buf);
    free(klio_rt.similarity_idx_buf);

    close_interfaces(intf);

    return rslt;
}

static void parse_klio(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    klio_runtime_t *klio_rt = (klio_runtime_t *)callback_ref;
    struct bhi360_dev *bhy = klio_rt->bhy;
    bhi360_event_data_klio_t data;
    uint32_t s, ns;
    uint64_t timestamp;

    if (callback_info->data_size != (sizeof(bhi360_event_data_klio_t) + 1)) /* Check for a valid payload size. Includes
                                                                             * sensor ID */
    {
        return;
    }

    timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    memcpy(&data, callback_info->data_ptr, sizeof(data));
    printf("\r\n");
#ifndef PC
    printf("SID: %u; T: %lu.%09lu; Learning [Id:%d Progress:%u Change:%u]; Recognition[Id:%d Count:%f Score:%f]\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.learn.index,
           data.learn.progress,
           data.learn.change_reason,
           data.recognize.index,
           data.recognize.count,
           data.recognize.score);
#else
    printf("SID: %u; T: %u.%09u; Learning [Id:%d Progress:%u Change:%u]; Recognition[Id:%d Count:%f Score:%f]\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.learn.index,
           data.learn.progress,
           data.learn.change_reason,
           data.recognize.index,
           data.recognize.count,
           data.recognize.score);
#endif
    printf("\r\n");

    if (data.learn.index != -1) /* -1 means nothing was learnt. */
    {
        float highest_similarity_score = parse_klio_handle_learnt_pattern(callback_info, s, ns, klio_rt);

        if (highest_similarity_score < 0.6) /* Enable if initial pattern or dissimilar */
        {
            int8_t rslt = bhi360_klio_param_set_pattern_states(KLIO_PATTERN_STATE_ENABLE,
                                                               &klio_rt->pattern_write_back_index,
                                                               1,
                                                               bhy);
            print_api_error(rslt, bhy);
            print_klio_status(bhy);

            klio_rt->pattern_write_back_index++;
        }
    }
}

static float parse_klio_handle_learnt_pattern(const struct bhi360_fifo_parse_data_info *callback_info,
                                              uint32_t s,
                                              uint32_t ns,
                                              klio_runtime_t *klio_rt)
{
    uint8_t tmp_buf[PARAM_BUF_LEN];
    uint16_t bufsize = sizeof(tmp_buf);
    struct bhi360_dev *bhy = klio_rt->bhy;
    float highest_similarity_score = 0.f;

    /* Read out learnt pattern */
    int8_t rslt = bhi360_klio_param_read_pattern(0, tmp_buf, &bufsize, bhy);

    print_api_error(rslt, bhy);
    print_klio_status(bhy);

    printf("\r\n");

#ifndef PC
    printf("SID: %u; T: %lu.%09lu; PATTERN LEARNT: ", callback_info->sensor_id, s, ns);
#else
    printf("SID: %u; T: %u.%09u; PATTERN LEARNT: ", callback_info->sensor_id, s, ns);
#endif

    for (uint16_t i = 0; i < bufsize; i++)
    {
        printf("%02x", tmp_buf[i]);
    }

    printf("\r\n");

    /* Write back learnt pattern for recognition */
    if (klio_rt->pattern_write_back_index < klio_rt->max_patterns)
    {
        /* Write pattern for recognition, note that this resets recognition statistics (and repetition counts) */
        rslt = bhi360_klio_param_write_pattern(klio_rt->pattern_write_back_index, tmp_buf, bufsize, bhy);
        print_api_error(rslt, bhy);
        print_klio_status(bhy);

        if (klio_rt->pattern_write_back_index > 0)
        {
            /* Compare current pattern against all previously stored ones */
            for (uint8_t i = 0; i < klio_rt->pattern_write_back_index; i++)
            {
                klio_rt->similarity_idx_buf[i] = i;
            }

            printf("\n");
            rslt = bhi360_klio_param_similarity_score_multiple(klio_rt->pattern_write_back_index,
                                                               klio_rt->similarity_idx_buf,
                                                               klio_rt->pattern_write_back_index,
                                                               klio_rt->similarity_result_buf,
                                                               klio_rt->bhy);
            print_api_error(rslt, bhy);
            print_klio_status(bhy);
#ifndef PC
            printf("SID: %u; T: %lu.%09lu; SIMILARITY SCORE TO ALREADY STORED PATTERNS: ",
                   callback_info->sensor_id,
                   s,
                   ns);
#else
            printf("SID: %u; T: %u.%09u; SIMILARITY SCORE TO ALREADY STORED PATTERNS: ", callback_info->sensor_id, s,
                   ns);
#endif
            for (uint8_t i = 0; i < klio_rt->pattern_write_back_index; i++)
            {
                float tmp_score = klio_rt->similarity_result_buf[i];

                printf("%d: %f ", i, tmp_score);

                if (tmp_score > highest_similarity_score)
                {
                    highest_similarity_score = tmp_score;
                }
            }

            printf("\r\n");
        }
    }

    /* If we have no stored patterns return will be 0.f */

    return highest_similarity_score;
}

static void print_klio_error(bhi360_klio_param_driver_error_state_t status)
{
    if (status != KLIO_DRIVER_ERROR_NONE)
    {
        printf("%s\r\n", get_klio_error(status));
    }
}

static void print_klio_status(struct bhi360_dev *bhy)
{
    uint32_t klio_status;
    int8_t rslt = bhi360_klio_param_read_reset_driver_status(&klio_status, bhy);

    print_api_error(rslt, bhy);
    print_klio_error((bhi360_klio_param_driver_error_state_t)klio_status);
}