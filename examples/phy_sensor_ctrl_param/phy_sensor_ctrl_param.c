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
 * @file    bhi360_phy_sensor_ctrl_param.c
 * @brief   Example to set/get physical sensor control parameters
 *
 */

#include <stdio.h>
#include "common.h"
#include "bhi360_parse.h"
#include "bhi360_phy_sensor_ctrl_param.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350_bsxsam_ndof.fw.h"

static void run_phy_accel_sample(struct bhi360_dev *dev);
static void run_phy_gyro_sample(struct bhi360_dev *dev);
static void run_phy_magnet_sample(struct bhi360_dev *dev);

#define PARSE_DATA_WINDOW_SIZE  UINT16_C(3000)

/**
* @brief Function to parse 3axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
static void parse_3axis_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

#define ACC_ACCURACY_INDEX      UINT8_C(0)
#define GYRO_ACCURACY_INDEX     UINT8_C(1)
#define MAG_ACCURACY_INDEX      UINT8_C(2)

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt = BHI360_OK;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t accuracy[3] = { 0 }; /* Accuracy is reported as a meta event. It is being printed alongside the data */

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

        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT, parse_meta_event, (void*)accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)accuracy, &bhy);
        print_api_error(rslt, &bhy);

        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_ACC,
                                                parse_3axis_data,
                                                (void*)&accuracy[ACC_ACCURACY_INDEX],
                                                &bhy);
        print_api_error(rslt, &bhy);

        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_GYRO,
                                                parse_3axis_data,
                                                (void*)&accuracy[GYRO_ACCURACY_INDEX],
                                                &bhy);
        print_api_error(rslt, &bhy);

        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_MAG,
                                                parse_3axis_data,
                                                (void*)&accuracy[MAG_ACCURACY_INDEX],
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

    run_phy_accel_sample(&bhy);
    run_phy_gyro_sample(&bhy);
    run_phy_magnet_sample(&bhy);

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    /* Enable accelerometer, gyroscope, and magnetometer virtual sensors */
    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_ACC, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("\r\nEnable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_ACC), sensor_conf.sample_rate);

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_GYRO, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_GYRO), sensor_conf.sample_rate);

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_MAG, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_MAG), sensor_conf.sample_rate);

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

    /* Disable accelerometer, gyroscope, and magnetometer virtual sensors */
    sensor_conf.sample_rate = BHI360_ODR_0_HZ; /*! Disable the sensor by setting sample rate to 0Hz */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_ACC, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("\nDisable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_ACC));

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_GYRO, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Disable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_GYRO));

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_MAG, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Disable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_MAG));

    close_interfaces(intf);

    return rslt;
}

static void run_phy_accel_sample(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t mode = 0;
    uint8_t status = 0;
    bhi360_phy_sensor_ctrl_param_accel_fast_offset_calib calib = { 0 };
    bhi360_phy_sensor_ctrl_param_accel_axis_remap remap = { 0 };

    printf("Get accelerometer calibration.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_accel_get_foc_calibration(&calib, dev);
    print_api_error(rslt, dev);
    printf("    Accelerometer calibration X-axis: %d\r\n", calib.x_offset);
    printf("    Accelerometer calibration Y-axis: %d\r\n", calib.y_offset);
    printf("    Accelerometer calibration Z-axis: %d\r\n", calib.z_offset);

    printf("Get accelerometer power mode.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_accel_get_power_mode(&mode, dev);
    print_api_error(rslt, dev);
    printf("    Accelerometer power mode: %d\r\n", mode);
    printf("Change accelerometer power mode.\r\n");
    mode = 2;
    rslt = bhi360_phy_sensor_ctrl_param_accel_set_power_mode(mode, dev);
    print_api_error(rslt, dev);
    printf("Get accelerometer power mode again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_accel_get_power_mode(&mode, dev);
    print_api_error(rslt, dev);
    printf("    Accelerometer power mode: %d\r\n", mode);

    printf("Get accelerometer axis remapping.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_accel_get_axis_remapping(&remap, dev);
    print_api_error(rslt, dev);
    printf("    Accelerometer remapping X-axis: %d\r\n", remap.map_x_axis);
    printf("    Accelerometer remapping X-axis sign: %d\r\n", remap.map_x_axis_sign);
    printf("    Accelerometer remapping Y-axis: %d\r\n", remap.map_y_axis);
    printf("    Accelerometer remapping Y-axis sign: %d\r\n", remap.map_y_axis_sign);
    printf("    Accelerometer remapping Z-axis: %d\r\n", remap.map_z_axis);
    printf("    Accelerometer remapping Z-axis sign: %d\r\n", remap.map_z_axis_sign);
    printf("Change accelerometer axis remapping.\r\n");
    remap.map_x_axis = 0;
    remap.map_x_axis_sign = 1;
    remap.map_y_axis = 1;
    remap.map_y_axis_sign = 1;
    remap.map_z_axis = 2;
    remap.map_z_axis_sign = 1;
    rslt = bhi360_phy_sensor_ctrl_param_accel_set_axis_remapping(&remap, dev);
    print_api_error(rslt, dev);
    printf("Get accelerometer axis remapping again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_accel_get_axis_remapping(&remap, dev);
    print_api_error(rslt, dev);
    printf("    Accelerometer remapping X-axis: %d\r\n", remap.map_x_axis);
    printf("    Accelerometer remapping X-axis sign: %d\r\n", remap.map_x_axis_sign);
    printf("    Accelerometer remapping Y-axis: %d\r\n", remap.map_y_axis);
    printf("    Accelerometer remapping Y-axis sign: %d\r\n", remap.map_y_axis_sign);
    printf("    Accelerometer remapping Z-axis: %d\r\n", remap.map_z_axis);
    printf("    Accelerometer remapping Z-axis sign: %d\r\n", remap.map_z_axis_sign);

    printf("Get accelerometer NVM writing status.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_accel_get_nvm_status(&status, dev);
    print_api_error(rslt, dev);
    printf("    Accelerometer NVM writing status: %d\r\n", status);
}

static void run_phy_gyro_sample(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t mode = 0;
    uint8_t status = 0;
    uint8_t ois_conf = 0;
    uint8_t fast_startup_conf = 0;
    uint8_t auto_trim_conf = 0;
    bhi360_phy_sensor_ctrl_param_gyro_fast_offset_calib calib = { 0 };
    bhi360_phy_sensor_ctrl_param_gyro_manual_sensitivity_comp manual_sens_comp = { 0 };

    printf("Get gyroscope manual sensitivity compensation.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_manual_sensitivity_comp(&manual_sens_comp, dev);
    print_api_error(rslt, dev);

    printf("    Gyroscope manual sensitivity compensation X-axis: 0x%X\r\n", manual_sens_comp.gain_update_x);
    printf("    Gyroscope manual sensitivity compensation Y-axis: 0x%X\r\n", manual_sens_comp.gain_update_y);
    printf("    Gyroscope manual sensitivity compensation Z-axis: 0x%X\r\n", manual_sens_comp.gain_update_z);

    manual_sens_comp.gain_update_x = 0x300;
    manual_sens_comp.gain_update_y = 0x400;
    manual_sens_comp.gain_update_z = 0x500;

    rslt = bhi360_phy_sensor_ctrl_param_gyro_set_manual_sensitivity_comp(&manual_sens_comp, dev);
    print_api_error(rslt, dev);

    memset(&manual_sens_comp, 0, sizeof(manual_sens_comp));

    printf("Get gyroscope manual sensitivity compensation again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_manual_sensitivity_comp(&manual_sens_comp, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope manual sensitivity compensation X-axis: 0x%X\r\n", manual_sens_comp.gain_update_x);
    printf("    Gyroscope manual sensitivity compensation Y-axis: 0x%X\r\n", manual_sens_comp.gain_update_y);
    printf("    Gyroscope manual sensitivity compensation Z-axis: 0x%X\r\n", manual_sens_comp.gain_update_z);

    printf("Get gyroscope calibration.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_foc_calibration(&calib, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope calibration X-axis: %d\r\n", calib.x_offset);
    printf("    Gyroscope calibration Y-axis: %d\r\n", calib.y_offset);
    printf("    Gyroscope calibration Z-axis: %d\r\n", calib.z_offset);

    printf("Get gyroscope OIS configuration.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_ois_config(&ois_conf, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope OIS configuration: %d\r\n", ois_conf);
    printf("Change gyroscope OIS configuration.\r\n");
    ois_conf = 1;
    rslt = bhi360_phy_sensor_ctrl_param_gyro_set_ois_config(ois_conf, dev);
    print_api_error(rslt, dev);
    printf("Get gyroscope OIS configuration again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_ois_config(&ois_conf, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope OIS configuration: %d\r\n", ois_conf);

    printf("Get gyroscope Fast start up configuration.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg(&fast_startup_conf, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope Fast start up configuration: %d\r\n", fast_startup_conf);
    printf("Change gyroscope Fast start up configuration.\r\n");
    fast_startup_conf = 1;
    rslt = bhi360_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg(fast_startup_conf, dev);
    print_api_error(rslt, dev);
    printf("Get gyroscope Fast start up configuration again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg(&fast_startup_conf, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope Fast start up configuration: %d\r\n", fast_startup_conf);

    printf("Get gyroscope power mode.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_power_mode(&mode, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope power mode: %d\r\n", mode);
    printf("Change gyroscope power mode.\r\n");
    mode = 1;
    rslt = bhi360_phy_sensor_ctrl_param_gyro_set_power_mode(mode, dev);
    print_api_error(rslt, dev);
    printf("Get gyroscope power mode again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_power_mode(&mode, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope power mode: %d\r\n", mode);

    printf("Get gyroscope timer auto trim configuration.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg(&auto_trim_conf, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope timer auto trim configuration: %d\r\n", auto_trim_conf);
    printf("Change gyroscope timer auto trim configuration.\r\n");
    auto_trim_conf = 1;
    rslt = bhi360_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg(auto_trim_conf, dev);
    print_api_error(rslt, dev);
    printf("Get gyroscope timer auto trim configuration again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg(&auto_trim_conf, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope timer auto trim configuration: %d\r\n", auto_trim_conf);

    printf("Get gyroscope NVM writing status.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_gyro_get_nvm_status(&status, dev);
    print_api_error(rslt, dev);
    printf("    Gyroscope NVM writing status: %d\r\n", status);

}

static void run_phy_magnet_sample(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t mode = 0;

    printf("Get magnetometer power mode.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_magnet_get_power_mode(&mode, dev);
    print_api_error(rslt, dev);
    printf("    Magnetometer power mode: %d\r\n", mode);
    printf("Change magnetometer power mode.\r\n");
    mode = 1;
    rslt = bhi360_phy_sensor_ctrl_param_magnet_set_power_mode(mode, dev);
    print_api_error(rslt, dev);
    printf("Get magnetometer power mode again.\r\n");
    rslt = bhi360_phy_sensor_ctrl_param_magnet_get_power_mode(&mode, dev);
    print_api_error(rslt, dev);
    printf("    Magnetometer power mode: %d\r\n", mode);
}

static void parse_3axis_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_xyz data;
    uint32_t s, ns;
    uint64_t timestamp;

    float scaling_factor = 0.0f;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    if (callback_info->sensor_id >= BHI360_SENSOR_ID_ACC && callback_info->sensor_id <= BHI360_SENSOR_ID_ACC_RAW_WU)
    {
        scaling_factor = 1.0f / 4096.0f;
    }
    else if (callback_info->sensor_id >= BHI360_SENSOR_ID_GYRO &&
             callback_info->sensor_id <= BHI360_SENSOR_ID_GYRO_RAW_WU)
    {
        scaling_factor = 2000.0f / 32768.0f;
    }
    else if (callback_info->sensor_id >= BHI360_SENSOR_ID_MAG &&
             callback_info->sensor_id <= BHI360_SENSOR_ID_MAG_RAW_WU)
    {
        scaling_factor = 2000.0f / 32768.0f;
    }

    uint8_t *accuracy = (uint8_t*)callback_ref;

    bhi360_event_data_parse_xyz(callback_info->data_ptr, &data);

    timestamp = *callback_info->time_stamp; /* Store the last timestamp */
    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

#ifndef PC
    printf("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f; acc: %u\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x * scaling_factor,
           data.y * scaling_factor,
           data.z * scaling_factor,
           *accuracy);
#else
    printf("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f; acc: %u\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x * scaling_factor,
           data.y * scaling_factor,
           data.z * scaling_factor,
           *accuracy);
#endif
}

static void parse_meta_event_extended(uint8_t meta_event_type, uint8_t byte1, uint8_t byte2, char*event_text)
{
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
        case BHI360_META_EVENT_SYSTEM_ERROR:
            printf("%s System error event, Error Register: %u, Interrupt State Register: %u\r\n",
                   event_text,
                   byte1,
                   byte2);
            break;
        case BHI360_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
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
    }
}

static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint8_t *accuracy = (uint8_t*)callback_ref;
    char *event_text;

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
        case BHI360_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            if (accuracy)
            {
                uint8_t sensor_id = byte1;
                switch (sensor_id)
                {
                    case BHI360_SENSOR_ID_ACC_BIAS:
                    case BHI360_SENSOR_ID_ACC:
                        accuracy[ACC_ACCURACY_INDEX] = byte2;

                        break;
                    case BHI360_SENSOR_ID_GYRO_BIAS:
                    case BHI360_SENSOR_ID_GYRO:
                        accuracy[GYRO_ACCURACY_INDEX] = byte2;

                        break;
                    case BHI360_SENSOR_ID_MAG_BIAS:
                    case BHI360_SENSOR_ID_MAG:
                        accuracy[MAG_ACCURACY_INDEX] = byte2;
                        break;
                    default:
                        break;
                }
            }

            break;

        default:
            parse_meta_event_extended(meta_event_type, byte1, byte2, event_text);
            break;
    }
}