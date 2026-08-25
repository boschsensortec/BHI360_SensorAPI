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
 * @file    wrist_gesture_detector.c
 * @brief   Example for wrist gesture detector usage
 *
 */

#include <stdio.h>
#include "common.h"
#include "bhi360_parse.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam_lite.fw.h"

/**
* @brief Function to parse any motion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_wrist_gesture_detector(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

#define MAX_EXECUTE_TIME  UINT8_C(5)

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;

    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t loop = 0;
    bhi360_generic_features_param_wrist_gesture_detector wgd_config = { 0 };

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
        printf("Loading firmware.\r\n");

        upload_firmware(bhi360_firmware_image, sizeof(bhi360_firmware_image), &bhy);

        rslt = bhi360_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI360_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        /* Update the callback table to enable parsing of sensor data */
        rslt = bhi360_update_virtual_sensor_list(&bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT, bhi360_parse_meta_event, NULL, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU, bhi360_parse_meta_event, NULL, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU,
                                                   parse_wrist_gesture_detector,
                                                   NULL,
                                                   &bhy);
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

    /*Get wrist gesture detector configuration */
    rslt = bhi360_generic_features_param_get_wrist_gesture_cfg(&wgd_config, &bhy);
    print_api_error(rslt, &bhy);

    printf("Wrist gesture detector configuration:\r\n");
    printf("min_flick_peak_y_thres: %X\r\n", wgd_config.min_flick_peak_y_thres);
    printf("min_flick_peak_z_thres: %X\r\n", wgd_config.min_flick_peak_z_thres);
    printf("gravity_bounds_x_pos: %X\r\n", wgd_config.gravity_bounds_x_pos);
    printf("gravity_bounds_x_neg: %X\r\n", wgd_config.gravity_bounds_x_neg);
    printf("gravity_bounds_y_neg: %X\r\n", wgd_config.gravity_bounds_y_neg);
    printf("gravity_bounds_z_neg: %X\r\n", wgd_config.gravity_bounds_z_neg);
    printf("flick_peak_decay_coeff: %X\r\n", wgd_config.flick_peak_decay_coeff);
    printf("lp_mean_filter_coeff: %X\r\n", wgd_config.lp_mean_filter_coeff);
    printf("max_duration_jiggle_peaks: %X\r\n", wgd_config.max_duration_jiggle_peaks);
    printf("device_pos: %X\r\n", wgd_config.device_pos);

    /* Set wrist gesture detector configuration */
    wgd_config.min_flick_peak_y_thres = 0x900; /* Minimum threshold for flick peak y-axis */
    wgd_config.min_flick_peak_z_thres = 0x800; /* Minimum threshold for flick peak z-axis */

    rslt = bhi360_generic_features_param_set_wrist_gesture_cfg(&wgd_config, &bhy);
    print_api_error(rslt, &bhy);

    /*Get wrist gesture detector configuration */
    rslt = bhi360_generic_features_param_get_wrist_gesture_cfg(&wgd_config, &bhy);
    print_api_error(rslt, &bhy);

    printf("Wrist gesture detector configuration after setting:\r\n");
    printf("min_flick_peak_y_thres: %X\r\n", wgd_config.min_flick_peak_y_thres);
    printf("min_flick_peak_z_thres: %X\r\n", wgd_config.min_flick_peak_z_thres);
    printf("gravity_bounds_x_pos: %X\r\n", wgd_config.gravity_bounds_x_pos);
    printf("gravity_bounds_x_neg: %X\r\n", wgd_config.gravity_bounds_x_neg);
    printf("gravity_bounds_y_neg: %X\r\n", wgd_config.gravity_bounds_y_neg);
    printf("gravity_bounds_z_neg: %X\r\n", wgd_config.gravity_bounds_z_neg);
    printf("flick_peak_decay_coeff: %X\r\n", wgd_config.flick_peak_decay_coeff);
    printf("lp_mean_filter_coeff: %X\r\n", wgd_config.lp_mean_filter_coeff);
    printf("max_duration_jiggle_peaks: %X\r\n", wgd_config.max_duration_jiggle_peaks);
    printf("device_pos: %X\r\n", wgd_config.device_pos);

    /* Reset wrist gesture detector configuration */
    rslt = bhi360_generic_features_param_reset_wrist_gesture_cfg(&bhy);
    print_api_error(rslt, &bhy);

    /*Get wrist gesture detector configuration */
    rslt = bhi360_generic_features_param_get_wrist_gesture_cfg(&wgd_config, &bhy);
    print_api_error(rslt, &bhy);

    printf("Wrist gesture detector configuration after reseting:\r\n");
    printf("min_flick_peak_y_thres: %X\r\n", wgd_config.min_flick_peak_y_thres);
    printf("min_flick_peak_z_thres: %X\r\n", wgd_config.min_flick_peak_z_thres);
    printf("gravity_bounds_x_pos: %X\r\n", wgd_config.gravity_bounds_x_pos);
    printf("gravity_bounds_x_neg: %X\r\n", wgd_config.gravity_bounds_x_neg);
    printf("gravity_bounds_y_neg: %X\r\n", wgd_config.gravity_bounds_y_neg);
    printf("gravity_bounds_z_neg: %X\r\n", wgd_config.gravity_bounds_z_neg);
    printf("flick_peak_decay_coeff: %X\r\n", wgd_config.flick_peak_decay_coeff);
    printf("lp_mean_filter_coeff: %X\r\n", wgd_config.lp_mean_filter_coeff);
    printf("max_duration_jiggle_peaks: %X\r\n", wgd_config.max_duration_jiggle_peaks);
    printf("device_pos: %X\r\n", wgd_config.device_pos);

    /* Enable wrist gesture detector virtual sensor */
    sensor_conf.sample_rate = 1.0f; /* Read out data measured at 1Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("\r\nEnable %s at %.2fHz.\r\n",
           get_sensor_name(BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU),
           sensor_conf.sample_rate);

    while (rslt == BHI360_OK && loop < MAX_EXECUTE_TIME)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            loop++;
            print_api_error(rslt, &bhy);
        }
    }

    /* Disabling wrist gesture detector virtual sensors */
    sensor_conf.sample_rate = BHI360_ODR_0_HZ; /*! Disable the sensor by setting sample rate to 0Hz */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("\nDisable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU));

    close_interfaces(intf);

    return rslt;
}

void parse_wrist_gesture_detector(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_wrist_gesture_detect_t wrist_gesture_detect_data;
    uint32_t s, ns;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    (void)bhi360_event_data_wrist_gesture_detect_parsing(callback_info->data_ptr, &wrist_gesture_detect_data);

#ifndef PC
    printf("SID: %u; T: %lu.%09lu; wrist_gesture: %s\r\n",
           callback_info->sensor_id,
           s,
           ns,
           bhi360_event_data_wrist_gesture_detect_output[wrist_gesture_detect_data.wrist_gesture]);
#else
    printf("SID: %u; T: %u.%09u; wrist_gesture: %s\r\n",
           callback_info->sensor_id,
           s,
           ns,
           bhi360_event_data_wrist_gesture_detect_output[wrist_gesture_detect_data.wrist_gesture]);
#endif
}