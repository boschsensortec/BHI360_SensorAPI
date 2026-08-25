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
 * @file    remap_physical_sensor.c
 * @brief   Remapping physical sensor example for the BHI360 sensor
 *
 */

#include <stdio.h>
#include "common.h"
#include "bhi360.h"
#include "bhi360_parse.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350_BMP58X_BME688_bsxsam_ndof.fw.h"

void get_orientation_matrix_from_psi(struct bhi360_system_param_phys_sensor_info *psi,
                                     struct bhi360_system_param_orient_matrix *ort_mtx);

void setting_orientation_matrix_with_new_values(struct bhi360_system_param_orient_matrix *ort_mtx, int8_t *value);

#define PARSE_DATA_WINDOW_SIZE  UINT16_C(3000)

#define PARSE_DATA_WINDOW_SIZE  UINT16_C(3000)

#define ACC_ACCURACY_INDEX      UINT8_C(0)
#define GYRO_ACCURACY_INDEX     UINT8_C(1)
#define MAG_ACCURACY_INDEX      UINT8_C(2)

/**
* @brief Function to parse 3axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
static void parse_3axis_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

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

    struct bhi360_system_param_phys_sensor_info accel_psi = { 0 }, gyro_psi = { 0 }, mag_psi = { 0 };
    struct bhi360_system_param_orient_matrix accel_ort_mtx = { { 0 } }, gyro_ort_mtx = { { 0 } },
                                             mag_ort_mtx = { { 0 } };

    /*! Get and set new orientation matrix for accelerometer */
    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, &accel_psi, &bhy);
    print_api_error(rslt, &bhy);
    printf("Getting default orientation matrix for accelerometer:\r\n");
    get_orientation_matrix_from_psi(&accel_psi, &accel_ort_mtx);

    int8_t arr[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };
    setting_orientation_matrix_with_new_values(&accel_ort_mtx, arr);
    rslt = bhi360_system_param_set_physical_sensor_info(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, &accel_ort_mtx, &bhy);
    print_api_error(rslt, &bhy);

    memset(&accel_psi, 0, sizeof(accel_psi));
    memset(&accel_ort_mtx, 0, sizeof(accel_ort_mtx));
    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, &accel_psi, &bhy);
    print_api_error(rslt, &bhy);
    printf("Getting again orientation matrix for accelerometer after changed:\r\n");
    get_orientation_matrix_from_psi(&accel_psi, &accel_ort_mtx);

    /*! Get and set new orientation matrix for gyroscope */
    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_GYROSCOPE, &gyro_psi, &bhy);
    print_api_error(rslt, &bhy);
    printf("Getting default orientation matrix for gyroscope:\r\n");
    get_orientation_matrix_from_psi(&gyro_psi, &gyro_ort_mtx);

    setting_orientation_matrix_with_new_values(&gyro_ort_mtx, arr);
    rslt = bhi360_system_param_set_physical_sensor_info(BHI360_PHYS_SENSOR_ID_GYROSCOPE, &gyro_ort_mtx, &bhy);
    print_api_error(rslt, &bhy);

    memset(&gyro_psi, 0, sizeof(gyro_psi));
    memset(&gyro_ort_mtx, 0, sizeof(gyro_ort_mtx));
    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_GYROSCOPE, &gyro_psi, &bhy);
    print_api_error(rslt, &bhy);
    printf("Getting again orientation matrix for gyroscope after changed:\r\n");
    get_orientation_matrix_from_psi(&gyro_psi, &gyro_ort_mtx);

    /*! Get and set new orientation matrix for magnetometer */
    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_MAGNETOMETER, &mag_psi, &bhy);
    print_api_error(rslt, &bhy);
    printf("Getting default orientation matrix for magnetometer:\r\n");
    get_orientation_matrix_from_psi(&mag_psi, &mag_ort_mtx);

    setting_orientation_matrix_with_new_values(&mag_ort_mtx, arr);
    rslt = bhi360_system_param_set_physical_sensor_info(BHI360_PHYS_SENSOR_ID_MAGNETOMETER, &mag_ort_mtx, &bhy);
    print_api_error(rslt, &bhy);

    memset(&mag_psi, 0, sizeof(mag_psi));
    memset(&mag_ort_mtx, 0, sizeof(mag_ort_mtx));
    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_MAGNETOMETER, &mag_psi, &bhy);
    print_api_error(rslt, &bhy);
    printf("Getting again orientation matrix for magnetometer after changed:\r\n");
    get_orientation_matrix_from_psi(&mag_psi, &mag_ort_mtx);

    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */

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

void get_orientation_matrix_from_psi(struct bhi360_system_param_phys_sensor_info *psi,
                                     struct bhi360_system_param_orient_matrix *ort_mtx)
{
    ort_mtx->c[0] = (int8_t)((psi->orientation_matrix[0] & 0x0F) << 4) >> 4;
    ort_mtx->c[1] = (int8_t)(psi->orientation_matrix[0] & 0xF0) >> 4;
    ort_mtx->c[2] = (int8_t)((psi->orientation_matrix[1] & 0x0F) << 4) >> 4;
    ort_mtx->c[3] = (int8_t)(psi->orientation_matrix[1] & 0xF0) >> 4;
    ort_mtx->c[4] = (int8_t)((psi->orientation_matrix[2] & 0x0F) << 4) >> 4;
    ort_mtx->c[5] = (int8_t)(psi->orientation_matrix[2] & 0xF0) >> 4;
    ort_mtx->c[6] = (int8_t)((psi->orientation_matrix[3] & 0x0F) << 4) >> 4;
    ort_mtx->c[7] = (int8_t)(psi->orientation_matrix[3] & 0xF0) >> 4;
    ort_mtx->c[8] = (int8_t)((psi->orientation_matrix[4] & 0x0F) << 4) >> 4;

    printf("%d %d %d\r\n", ort_mtx->c[0], ort_mtx->c[1], ort_mtx->c[2]);
    printf("%d %d %d\r\n", ort_mtx->c[3], ort_mtx->c[4], ort_mtx->c[5]);
    printf("%d %d %d\r\n", ort_mtx->c[6], ort_mtx->c[7], ort_mtx->c[8]);
    printf("\r\n");
}

void setting_orientation_matrix_with_new_values(struct bhi360_system_param_orient_matrix *ort_mtx, int8_t *value)
{
    for (uint8_t i = 0; i < 9; i++)
    {
        ort_mtx->c[i] = value[i];
    }
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

    if (callback_info->sensor_id >= BHI360_SENSOR_ID_ACC_PASS &&
        callback_info->sensor_id <= BHI360_SENSOR_ID_ACC_RAW_WU)
    {
        scaling_factor = 1.0f / 4096.0f;
    }
    else if (callback_info->sensor_id >= BHI360_SENSOR_ID_GYRO_PASS &&
             callback_info->sensor_id <= BHI360_SENSOR_ID_GYRO_RAW_WU)
    {
        scaling_factor = 2000.0f / 32768.0f;
    }
    else if (callback_info->sensor_id >= BHI360_SENSOR_ID_MAG_PASS &&
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

static void parse_meta_event_extended(uint8_t meta_event_type,
                                      uint8_t byte1,
                                      uint8_t byte2,
                                      char*event_text,
                                      uint32_t s,
                                      uint32_t ns)
{
    switch (meta_event_type)
    {
        case BHI360_META_EVENT_FLUSH_COMPLETE:
            printf("%s; T: %u.%09u; Flush complete for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s; T: %u.%09u; Sample rate changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_POWER_MODE_CHANGED:
            printf("%s; T: %u.%09u; Power mode changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SYSTEM_ERROR:
            printf("%s; T: %u.%09u; System error event, Error Register:%u, Interrupt State Register: %u\r\n",
                   event_text,
                   s,
                   ns,
                   byte1,
                   byte2);
            break;
        case BHI360_META_EVENT_ALGORITHM_EVENTS:
            printf("%s; T: %u.%09u; Algorithm event\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s; T: %u.%09u; BSX event (do steps main)\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s; T: %u.%09u; BSX event (do steps calib)\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s; T: %u.%09u; BSX event (get output signal)\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_SENSOR_ERROR:
            printf("%s; T: %u.%09u; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns, byte1, byte2);
            break;
        case BHI360_META_EVENT_FIFO_OVERFLOW:
            printf("%s; T: %u.%09u; FIFO overflow\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s; T: %u.%09u; Dynamic range changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_FIFO_WATERMARK:
            printf("%s; T: %u.%09u; FIFO watermark reached\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_INITIALIZED:
            printf("%s; T: %u.%09u; Firmware initialized. Firmware version %u\r\n", event_text, s, ns,
                   ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHI360_META_TRANSFER_CAUSE:
            printf("%s; T: %u.%09u; Transfer cause for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s; T: %u.%09u; Sensor framework event for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_RESET:
            printf("%s; T: %u.%09u; Reset event\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_SPACER:
            break;
    }
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint8_t *accuracy = (uint8_t*)callback_ref;
    char *event_text;
    uint32_t s, ns;
    uint64_t tns;

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

    (void)time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    switch (meta_event_type)
    {
        case BHI360_META_EVENT_SENSOR_STATUS:
            printf("%s; T: %u.%09u;  Accuracy for sensor id %u changed to %u\r\n", event_text, s, ns, byte1, byte2);
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
            parse_meta_event_extended(meta_event_type, byte1, byte2, event_text, s, ns);
            break;
    }
}