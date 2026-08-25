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
 * @file    virtual_sensor_conf_param.c
 * @brief   Virtual sensor configuration/information example with sensor id
 *
 */

#include <stdio.h>
#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam.fw.h"

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    struct bhi360_virtual_sensor_info_param_info info;

    /*! Selecting the SPI interface for sensor communication */
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

    /*! Check if the sensor is ready to load firmware */
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
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    /*! Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_virtual_sensor_conf_param_get_cfg(BHI360_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    printf("Sensor ID=%d, rate=%.2fHz, latency=%u, range=%d\r\n",
           BHI360_SENSOR_ID_GYRO_PASS,
           sensor_conf.sample_rate,
           sensor_conf.latency,
           sensor_conf.range);

    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /*! Read out data measured at 100Hz */
    sensor_conf.latency = 0; /*! Report immediately */

    /*! Setting the Sampling frequency and latency time */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_virtual_sensor_conf_param_get_cfg(BHI360_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    printf("Sensor ID=%d, rate=%.2fHz, latency=%u, range=%d\r\n",
           BHI360_SENSOR_ID_GYRO_PASS,
           sensor_conf.sample_rate,
           sensor_conf.latency,
           sensor_conf.range);

    printf("Get virtual sensor information.\r\n");
    rslt = bhi360_virtual_sensor_info_param_get_info(BHI360_SENSOR_ID_ACC_PASS, &info, &bhy);
    print_api_error(rslt, &bhy);
    printf("    Sensor ID: %u\r\n", info.sensor_type);
    printf("    Driver ID: %u\r\n", info.driver_id);
    printf("    Driver version: %u\r\n", info.driver_version);
    printf("    Power: %u\r\n", info.power);
    printf("    Max range: %u\r\n", info.max_range.u16_val);
    printf("    Resolution: %u\r\n", info.resolution.u16_val);
    printf("    Max rate: %f\r\n", info.max_rate.f_val);
#ifdef PC
    printf("    FIFO reserved: %u\r\n", info.fifo_reserved.u32_val);
    printf("    FIFO max: %u\r\n", info.fifo_max.u32_val);
#else
    printf("    FIFO reserved: %lu\r\n", info.fifo_reserved.u32_val);
    printf("    FIFO max: %lu\r\n", info.fifo_max.u32_val);
#endif
    printf("    Event size: %u\r\n", info.event_size);
    printf("    Min rate: %f\r\n", info.min_rate.f_val);

    /*! Close all the active communication */
    close_interfaces(intf);

    return rslt;
}