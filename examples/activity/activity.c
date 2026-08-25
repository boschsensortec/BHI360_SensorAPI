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
 * @file    activity.c
 * @brief   Activity example for the BHI360
 *
 */

#include <stdio.h>
#include "bhi360.h"
#include "bhi360_parse.h"
#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam.fw.h"

#define MAX_ACTIVITY_LOOP  UINT8_C(5)

/**
* @brief Function to parse activity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_activity_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0U;
    int8_t rslt = BHI360_OK;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0U;
    uint8_t count = 0U;
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };

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

        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT, bhi360_parse_meta_event, NULL, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU, bhi360_parse_meta_event, NULL, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_AR_WEAR_WU, parse_activity_data, NULL, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
        print_api_error(rslt, &bhy);

        /* Update the callback table to enable parsing of sensor data */
        rslt = bhi360_update_virtual_sensor_list(&bhy);
        print_api_error(rslt, &bhy);

        bhi360_activity_param_wearable set_wearable_cfg = { 0 };
        bhi360_activity_param_wearable get_wearable_cfg = { 0 };

        /* get activity parameters of Wearable */
        rslt = bhi360_activity_param_get_wearable_config(&get_wearable_cfg, &bhy);
        print_api_error(rslt, &bhy);

        printf("\n");
        printf("Get wearable activity parameters\r\n");
        printf("post_process_en: %u\r\n", get_wearable_cfg.post_process_en);
        printf("min_gdi_thre: %u\r\n", get_wearable_cfg.min_gdi_thre);
        printf("max_gdi_thre: %u\r\n", get_wearable_cfg.max_gdi_thre);
        printf("out_buff_size: %u\r\n", get_wearable_cfg.out_buff_size);
        printf("min_seg_moder_confg: %u\r\n", get_wearable_cfg.min_seg_moder_conf);

        set_wearable_cfg.post_process_en = 1;
        set_wearable_cfg.min_gdi_thre = 4095;
        set_wearable_cfg.max_gdi_thre = 4095;
        set_wearable_cfg.out_buff_size = 10;
        set_wearable_cfg.min_seg_moder_conf = 10;

        /* set activity parameters for Wearable */
        rslt = bhi360_activity_param_set_wearable_config(&set_wearable_cfg, &bhy);
        print_api_error(rslt, &bhy);

        printf("\n");
        printf("Set wearable activity parameters\r\n");
        printf("post_process_en: %u\r\n", set_wearable_cfg.post_process_en);
        printf("min_gdi_thre: %u\r\n", set_wearable_cfg.min_gdi_thre);
        printf("max_gdi_thre: %u\r\n", set_wearable_cfg.max_gdi_thre);
        printf("out_buff_size: %u\r\n", set_wearable_cfg.out_buff_size);
        printf("min_seg_moder_confg: %u\r\n", set_wearable_cfg.min_seg_moder_conf);

        /* get activity parameters of Wearable */
        rslt = bhi360_activity_param_get_wearable_config(&get_wearable_cfg, &bhy);
        print_api_error(rslt, &bhy);

        printf("\n");
        printf("Get wearable activity parameters\r\n");
        printf("post_process_en: %u\r\n", get_wearable_cfg.post_process_en);
        printf("min_gdi_thre: %u\r\n", get_wearable_cfg.min_gdi_thre);
        printf("max_gdi_thre: %u\r\n", get_wearable_cfg.max_gdi_thre);
        printf("out_buff_size: %u\r\n", get_wearable_cfg.out_buff_size);
        printf("min_seg_moder_confg: %u\r\n", get_wearable_cfg.min_seg_moder_conf);

        /* Enable Activity recognition sensor*/
        sensor_conf.sample_rate = 1.0f; /* Read out data measured at 1Hz */
        sensor_conf.latency = 0; /* Report immediately */

        rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_AR_WEAR_WU, &sensor_conf, &bhy);
        print_api_error(rslt, &bhy);
        printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_AR_WEAR_WU), sensor_conf.sample_rate);

        while (rslt == BHI360_OK && count < MAX_ACTIVITY_LOOP)
        {
            if (get_interrupt_status())
            {
                /* Data from the FIFO is read and the relevant callbacks if registered are called */
                rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
                count++;
                print_api_error(rslt, &bhy);
            }
        }

        /* Disable Activity recognition sensor*/
        sensor_conf.sample_rate = BHI360_ODR_0_HZ; /* Read out data measured at 0Hz */
        sensor_conf.latency = 0; /* Report immediately */

        rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_AR_WEAR_WU, &sensor_conf, &bhy);
        print_api_error(rslt, &bhy);
        printf("Disable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_AR_WEAR_WU));

    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    close_interfaces(intf);

    return rslt;
}

void parse_activity_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t activity;
    uint32_t s, ns;
    uint64_t timestamp;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    activity = BHI360_LE2U16(callback_info->data_ptr);

    timestamp = *callback_info->time_stamp; /* Store the last timestamp */
    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));
#ifndef PC
    printf("SID: %u; T: %lu.%09lu", callback_info->sensor_id, s, ns);
#else
    printf("SID: %u; T: %u.%09u", callback_info->sensor_id, s, ns);
#endif

    if (activity & BHI360_STILL_ACTIVITY_ENDED)
    {
        printf(" Still activity ended,\r\n");
    }

    if (activity & BHI360_WALKING_ACTIVITY_ENDED)
    {
        printf(" Walking activity ended,\r\n");
    }

    if (activity & BHI360_RUNNING_ACTIVITY_ENDED)
    {
        printf(" Running activity ended,\r\n");
    }

    if (activity & BHI360_ON_BICYCLE_ACTIVITY_ENDED)
    {
        printf(" On bicycle activity ended,\r\n");
    }

    if (activity & BHI360_IN_VEHICLE_ACTIVITY_ENDED)
    {
        printf(" In vehicle ended,\r\n");
    }

    if (activity & BHI360_STILL_ACTIVITY_STARTED)
    {
        printf(" Still activity started,\r\n");
    }

    if (activity & BHI360_WALKING_ACTIVITY_STARTED)
    {
        printf(" Walking activity started,\r\n");
    }

    if (activity & BHI360_RUNNING_ACTIVITY_STARTED)
    {
        printf(" Running activity started,\r\n");
    }

    if (activity & BHI360_ON_BICYCLE_ACTIVITY_STARTED)
    {
        printf(" On bicycle activity started,\r\n");
    }

    if (activity & BHI360_IN_VEHICLE_ACTIVITY_STARTED)
    {
        printf(" In vehicle activity started,\r\n");
    }
}