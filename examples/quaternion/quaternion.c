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
 * @file    quaternion.c
 * @brief   Quaternion data stream example for the BHI360
 *
 */

#include <stdio.h>

#include "bhi360.h"
#include "bhi360_parse.h"
#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam.fw.h"

static void parse_quaternion_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    uint8_t boot_status = 0;
    uint8_t loop = 0;
    uint8_t limit = 50;

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
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_GAMERV, parse_quaternion_data, NULL, &bhy);
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

    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_GAMERV, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_GAMERV), sensor_conf.sample_rate);

    while (rslt == BHI360_OK && loop < limit)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            loop++;
            print_api_error(rslt, &bhy);
        }
    }

    close_interfaces(intf);

    return rslt;
}

static void parse_quaternion_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhi360_event_data_quaternion data;
    uint32_t s, ns;
    uint64_t timestamp;

    if (callback_info->data_size != 11) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhi360_event_data_parse_quaternion(callback_info->data_ptr, &data);

    timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

#ifndef PC
    printf("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %.2f\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x / 16384.0f,
           data.y / 16384.0f,
           data.z / 16384.0f,
           data.w / 16384.0f,
           ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
#else
    printf("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f, w: %f; acc: %.2f\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x / 16384.0f,
           data.y / 16384.0f,
           data.z / 16384.0f,
           data.w / 16384.0f,
           ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
#endif
}
