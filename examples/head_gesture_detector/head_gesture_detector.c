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
 * @file    head_gesture_detector.c
 * @brief   Head gesture detector example for the BHY
 *
 */

#include "common.h"
#include "bhi360_parse.h"
#include "bhi360_event_data.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam_HeadGestureDetector.fw.h"

#define PARSE_DATA_WINDOW_SIZE  UINT16_C(60000)

/*!
 * @brief Output of the head gesture detector data is parsed for printing
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 */
static void parse_head_gesture_detector(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_quaternion(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt = BHI360_OK;
    struct bhi360_dev bhy;
    uint8_t boot_status;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t accuracy = 0;

    bhi360_head_gesture_algo_version_t hgd_ver = { 0 };

    bhi360_hgd_time_duration_of_one_gesture_t time_duration_of_one_gesture = { 0 };

    union bhi360_float_conv time_duration_of_one_gesture_tilt = { 0 };

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

        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG,
                                                   parse_quaternion,
                                                   (void*)&accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_HEAD_GESTURE_DETECTOR,
                                                   parse_head_gesture_detector,
                                                   NULL,
                                                   &bhy);
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

    /* Get the version numbers of hmc and ho algorithm */
    rslt = bhi360_head_feature_param_hgd_get_algo_version(&hgd_ver, &bhy);
    print_api_error(rslt, &bhy);

    printf("Head Gesture Detector Algorithm Version: %u.%u.%u\r\n",
           hgd_ver.major_version,
           hgd_ver.minor_version,
           hgd_ver.major_bug_fix);

    rslt = bhi360_head_feature_param_hgd_get_time_duration_of_one_gesture(&time_duration_of_one_gesture, &bhy);
    print_api_error(rslt, &bhy);
    printf("Time duration of one gesture: %.2fs\r\n", time_duration_of_one_gesture.time_duration_of_one_gesture.f_val);

    uint8_t thres_primary_motion_angular_rate = 0;
    rslt =
        bhi360_head_feature_param_hgd_get_thres_primary_motion_angular_rate(&thres_primary_motion_angular_rate, &bhy);
    print_api_error(rslt, &bhy);

    printf("Threshold of primary motion angular rate for head gesture detection: %u dps\r\n",
           thres_primary_motion_angular_rate);

    rslt =
        bhi360_head_feature_param_hgd_get_time_duration_of_one_gesture_tilt(&time_duration_of_one_gesture_tilt, &bhy);
    print_api_error(rslt, &bhy);

    printf("Time duration of one gesture tilt: %.2fs\r\n", time_duration_of_one_gesture_tilt.f_val);

    time_duration_of_one_gesture_tilt.f_val = 0.1f;
    rslt =
        bhi360_head_feature_param_hgd_set_time_duration_of_one_gesture_tilt(&time_duration_of_one_gesture_tilt, &bhy);
    print_api_error(rslt, &bhy);

    rslt =
        bhi360_head_feature_param_hgd_get_time_duration_of_one_gesture_tilt(&time_duration_of_one_gesture_tilt, &bhy);
    print_api_error(rslt, &bhy);

    printf("Time duration of one gesture tilt after changed: %.2fs\r\n", time_duration_of_one_gesture_tilt.f_val);

    uint8_t trigger_gesture_set_to_default = 1;
    rslt = bhi360_head_feature_param_hgd_set_to_default(trigger_gesture_set_to_default, &bhy);
    print_api_error(rslt, &bhy);

    rslt =
        bhi360_head_feature_param_hgd_get_time_duration_of_one_gesture_tilt(&time_duration_of_one_gesture_tilt, &bhy);
    print_api_error(rslt, &bhy);

    printf("Time duration of one gesture tilt after set to default: %.2fs\r\n",
           time_duration_of_one_gesture_tilt.f_val);

    /* Enabling head orientation misallignement virtual sensors */
    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* 100Hz */
    sensor_conf.latency = 0; /* Report immediately */

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG), sensor_conf.sample_rate);

    /* Trigger HMC calibration */
    rslt = bhi360_head_feature_param_trigger_hmc_calibration(&bhy);
    print_api_error(rslt, &bhy);

    while (true)
    {
        if (get_interrupt_status())
        {
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);
            if (accuracy == 1)
            {
                printf("\nPlease keep your head still.\r\n");
            }
            else if (accuracy == 2)
            {
                printf("\nPlease do nod movement.\r\n");
            }
            else if (accuracy == 3)
            {
                printf("\nCalibration successfully!\r\n");
                break;
            }
            else if (accuracy == 0)
            {
                printf("\nCalibration failed! Restart calibration!!!\r\n");
                rslt = bhi360_head_feature_param_trigger_hmc_calibration(&bhy);
                print_api_error(rslt, &bhy);
            }

            bhy.hif.delay_us(1000000, NULL);
        }
    }

    /* Stop 3 seconds*/
    bhy.hif.delay_us(3000000, NULL);

    /*! Setting the Sampling frequency and latency time */
    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /*! Read out data measured at 100Hz */
    sensor_conf.latency = 0; /*! Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_HEAD_GESTURE_DETECTOR, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    uint32_t curr_ts;
    uint32_t start_ts = coines_get_millis();
    curr_ts = coines_get_millis();
    do
    {
        curr_ts = coines_get_millis();

        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);
        }
    } while (rslt == BHI360_OK && (curr_ts - start_ts) < PARSE_DATA_WINDOW_SIZE);

    /* Disabling head gesture detector virtual sensor */
    sensor_conf.sample_rate = BHI360_ODR_0_HZ; /*! Disable the sensor by setting sample rate to 0Hz */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_HEAD_GESTURE_DETECTOR, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Disable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_HEAD_GESTURE_DETECTOR));

    close_interfaces(intf);

    return rslt;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void parse_head_gesture_detector(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    bhi360_event_data_head_gesture_detector hgd_data = BHI360_HGD_NO_GESTURE;
    uint32_t s, ns;
    uint64_t tns;

    if (callback_info->data_size != 2) /*! Check for a valid payload size. Includes sensor ID */
    {
        printf("Invalid payload size for head gesture detector data\r\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    hgd_data = (bhi360_event_data_head_gesture_detector)(*callback_info->data_ptr);

#ifndef PC
    printf("SID: %lu; T: %lu.%09lu; %s; \r\n",
           callback_info->sensor_id,
           s,
           ns,
           bhi360_event_data_head_gesture_string_out[hgd_data]);
#else
    printf("SID: %u; T: %u.%09u; %s; \r\n", callback_info->sensor_id, s, ns,
           bhi360_event_data_head_gesture_string_out[hgd_data]);
#endif
}

static void parse_quaternion(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhi360_event_data_quaternion data;
    uint32_t s, ns;
    uint64_t tns;

    if (callback_info->data_size < 8) /*! Check for a valid payload size. Includes sensor ID */
    {
        printf("Invalid payload size for quaternion data\r\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
    bhi360_event_data_parse_quaternion(callback_info->data_ptr, &data);

    uint8_t *accuracy = (uint8_t*)callback_ref;

#ifndef PC
    printf("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %u\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x / 16384.0f,
           data.y / 16384.0f,
           data.z / 16384.0f,
           data.w / 16384.0f,
           *accuracy);
#else
    printf("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f, w: %f; acc: %u\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x / 16384.0f,
           data.y / 16384.0f,
           data.z / 16384.0f,
           data.w / 16384.0f,
           *accuracy);
#endif
}

static void parse_meta_event_extended(uint8_t meta_event_type, uint8_t byte1, uint8_t byte2, char*event_text)
{
    switch (meta_event_type)
    {
        case BHI360_META_EVENT_INITIALIZED:
            printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHI360_META_TRANSFER_CAUSE:
            printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI360_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI360_META_EVENT_RESET:
            printf("%s Reset event\r\n", event_text);
            break;
        case BHI360_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;

    uint8_t *accuracy = (uint8_t*)callback_ref;

    if (callback_info->sensor_id == BHI360_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHI360_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHI360_META_EVENT_FLUSH_COMPLETE:
            printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI360_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI360_META_EVENT_POWER_MODE_CHANGED:
            printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI360_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
            break;
        case BHI360_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            if (accuracy)
            {
                uint8_t sensor_id = byte1;
                switch (sensor_id)
                {
                    case BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG:
                        *accuracy = byte2;
                        break;
                    default:
                        break;
                }
            }

            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHI360_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHI360_META_EVENT_SENSOR_ERROR:
            printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHI360_META_EVENT_FIFO_OVERFLOW:
            printf("%s FIFO overflow\r\n", event_text);
            break;
        case BHI360_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI360_META_EVENT_FIFO_WATERMARK:
            printf("%s FIFO watermark reached\r\n", event_text);
            break;
        default:
            parse_meta_event_extended(meta_event_type, byte1, byte2, event_text);
            break;
    }
}