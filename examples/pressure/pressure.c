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
 * @file    pressure.c
 * @brief   Example for pressure sensor usage
 *
 */

#include <stdio.h>
#include "common.h"
#include "bhi360_parse.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMP58X_bsxsam.fw.h"

static void run_pressure_param(struct bhi360_dev *dev);

/**
* @brief Function to parse pressure temperature format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_pressure_temperature_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

#define PARSE_DATA_WINDOW_SIZE  UINT16_C(5000)

/**
* @brief Function to parse any motion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_bmp_pressure_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;

    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

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

        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_PRESSURE, parse_bmp_pressure_data, NULL, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_BMP_TEMPERATURE,
                                                   parse_pressure_temperature_data,
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

    run_pressure_param(&bhy);

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    /* Enable pressure virtual sensor */
    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_PRESSURE, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_PRESSURE), sensor_conf.sample_rate);

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_BMP_TEMPERATURE, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_BMP_TEMPERATURE), sensor_conf.sample_rate);

    uint32_t curr_ts;
    uint32_t start_ts = coines_get_millis();
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

    /* Disabling pressure virtual sensors */
    sensor_conf.sample_rate = BHI360_ODR_0_HZ; /*! Disable the sensor by setting sample rate to 0Hz */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_PRESSURE, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("\nDisable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_PRESSURE));

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_BMP_TEMPERATURE, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("\nDisable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_BMP_TEMPERATURE));

    close_interfaces(intf);

    return rslt;
}

static void run_pressure_param(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    bhi360_phy_sensor_ctrl_param_baro_type_2 conf = { 0 };

    printf("Get barometer pressure type 2 configuration.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_baro_get_press_type_2_cfg(&conf, dev);
    print_api_error(rslt, dev);
    printf("    Barometer pressure type 2 configuration:\r\n");
    printf("     - <osr_p>: %u\r\n", conf.osr_p);
    printf("     - <osr_t>: %u\r\n", conf.osr_t);
    printf("     - <iir_filter_p>: %u\r\n", conf.iir_filter_p);
    printf("     - <iir_filter_t>: %u\r\n", conf.iir_filter_t);
    printf("     - <dsp_config>: %u\r\n", conf.dsp_config);
    printf("Change barometer pressure type 2 configuration.\r\n");
    conf.osr_t = 1;
    rslt = bhi360_phy_sensor_ctrl_param_baro_set_press_type_2_cfg(&conf, dev);
    print_api_error(rslt, dev);
    printf("Get barometer pressure type 2 configuration again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_baro_get_press_type_2_cfg(&conf, dev);
    print_api_error(rslt, dev);
    printf("    Barometer pressure type 2 configuration:\r\n");
    printf("     - <osr_p>: %u\r\n", conf.osr_p);
    printf("     - <osr_t>: %u\r\n", conf.osr_t);
    printf("     - <iir_filter_p>: %u\r\n", conf.iir_filter_p);
    printf("     - <iir_filter_t>: %u\r\n", conf.iir_filter_t);
    printf("     - <dsp_config>: %u\r\n", conf.dsp_config);
}

void parse_bmp_pressure_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t timestamp;
    float scaling_factor;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    scaling_factor = 1.0f / 128.0f;
    timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    data = BHI360_LE2U24(callback_info->data_ptr);

#ifndef PC
    printf("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
#else
    printf("SID: %u; T: %u.%09u; %f\r\n", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
#endif
}

void parse_pressure_temperature_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;

    float scaling_factor;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    scaling_factor = 1.0f / 128.0f;
    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    data = BHI360_LE2U24(callback_info->data_ptr);
#ifndef PC
    printf("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
#else
    printf("SID: %u; T: %u.%09u; %f\r\n", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
#endif
}