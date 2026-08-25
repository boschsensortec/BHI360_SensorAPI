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
 * @file    cal_seq.c
 * @brief   Example to do calibration process for the BHI360
 *
 */

#include <stdlib.h>
#include <stdio.h>

#include "bhi360.h"
#include "common.h"
#include "bhi360_phy_sensor_ctrl_param_defs.h"
#include "bhi360_parse.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350_bsxsam_ndof_turbo.fw.h"

#define SENSOR_FOC_SUCCESS       UINT8_C(0)
#define SENSOR_FOC_FAIL          UINT8_C(101)
#define SENSOR_FOC_UNKNOWN       UINT8_C(36)
#define SENSOR_CRT_SUCCESS       UINT8_C(0)
#define SENSOR_CRT_FAIL          UINT8_C(2)

#define BHI360_PHY_CRT_CTRL_LEN  UINT8_C(3)
#define CALIB_SEQ_FILE_NAME      "cal_seq.txt"

static void parse_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static int8_t bhi3_perform_foc_crt(struct bhi360_foc_resp *acc_foc_status,
                                   struct bhi360_foc_resp *gyro_foc_status,
                                   struct bhi360_dev *dev,
                                   uint8_t flag,
                                   enum bhi360_intf intf);

static int16_t convert_char_int(char *line, char *pattern1, char *pattern2);

static bhi360_phy_sensor_ctrl_param_gyro_fast_offset_calib foc_resp_gyro_backup;
static bhi360_phy_sensor_ctrl_param_accel_fast_offset_calib foc_resp_acc_backup;
static uint8_t crt_backup[3] = { 0 };

static bhi360_phy_sensor_ctrl_param_gyro_crt_data data_crt = { 0 };

int main(int argc, char *argv[])
{
    enum bhi360_intf intf;
    int8_t rslt;
    uint16_t version = 0;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;
    struct bhi360_foc_resp foc_resp_gyro, foc_resp_acc;
    uint8_t accuracy;
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    bool flag = true;
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    char line[256] = { 0 };
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
        printf("Loading firmware.\r\n");

        upload_firmware(bhi360_firmware_image, sizeof(bhi360_firmware_image), &bhy);

        rslt = bhi360_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);

        printf("Boot successful. Kernel version %u.\r\n", version);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");
        close_interfaces(intf);

        return 0;
    }

    while (true)
    {
        if (flag)
        {
            rslt = bhi3_perform_foc_crt(&foc_resp_acc, &foc_resp_gyro, &bhy, atoi(argv[1]), intf);
            print_api_error(rslt, &bhy);

            if (rslt != BHI360_OK)
            {
                close_interfaces(intf);

                return 0;
            }
        }
        else
        {
            printf("Loading firmware again.\r\n");

            rslt = bhi360_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), &bhy);
            print_api_error(rslt, &bhy);

            rslt = bhi360_get_kernel_version(&version, &bhy);
            print_api_error(rslt, &bhy);

            printf("Boot successful. Kernel version %u.\r\n", version);

            char *pattern1 = "x ";
            char *pattern2 = ", y ";
            char *pattern3 = ", z ";
            char *pattern4 = "\0";

            FILE *fp = fopen(CALIB_SEQ_FILE_NAME, "r");
            if (fp == NULL)
            {
                return -1;
            }

            while (fgets(line, sizeof(line), fp))
            {
                if (strstr(line, "Latest Accel FOC offsets"))
                {
                    foc_resp_acc_backup.x_offset = convert_char_int(line, pattern1, pattern2);
                    foc_resp_acc_backup.y_offset = convert_char_int(line, pattern2, pattern3);
                    foc_resp_acc_backup.z_offset = convert_char_int(line, pattern3, pattern4);
                }
            }

            rslt = bhi360_phy_sensor_ctrl_param_accel_set_foc_calibration(&foc_resp_acc_backup, &bhy);
            if (rslt != BHI360_OK)
            {
                printf("Set accel foc failed!\r\n");
                break;
            }

            printf("Perform Accel FOC successfully\r\n");
            printf("Acc  offsets back up: x %d, y %d, z %d\r\n",
                   foc_resp_acc_backup.x_offset,
                   foc_resp_acc_backup.y_offset,
                   foc_resp_acc_backup.z_offset);

            /* wait for load acc foc ready*/
            coines_delay_msec(10);

            while (fgets(line, sizeof(line), fp))
            {
                if (strstr(line, "Latest GYRO CRT status"))
                {
                    crt_backup[0] = convert_char_int(line, pattern1, pattern2);
                    crt_backup[1] = convert_char_int(line, pattern2, pattern3);
                    crt_backup[2] = convert_char_int(line, pattern3, pattern4);
                }
            }

            data_crt.x = crt_backup[0];
            data_crt.y = crt_backup[1];
            data_crt.z = crt_backup[2];
            rslt = bhi360_phy_sensor_ctrl_param_set_gyro_data(&data_crt, &bhy);
            if (rslt != BHI360_OK)
            {
                printf("load gyro crt failed!\r\n");
                break;
            }

            printf("crt  offsets back up: x %d, y %d, z %d status %d\r\n",
                   crt_backup[0],
                   crt_backup[1],
                   crt_backup[2],
                   rslt);

            while (fgets(line, sizeof(line), fp))
            {
                if (strstr(line, "Latest GYRO FOC offsets"))
                {
                    foc_resp_gyro_backup.x_offset = convert_char_int(line, pattern1, pattern2);
                    foc_resp_gyro_backup.y_offset = convert_char_int(line, pattern2, pattern3);
                    foc_resp_gyro_backup.z_offset = convert_char_int(line, pattern3, pattern4);
                }
            }

            rslt = bhi360_phy_sensor_ctrl_param_gyro_set_foc_calibration(&foc_resp_gyro_backup, &bhy);
            if (rslt != BHI360_OK)
            {
                printf("load gyro foc failed!\r\n");
                break;
            }

            fclose(fp);
            printf("gyro offsets back up: x %d, y %d, z %d\r\n",
                   foc_resp_gyro_backup.x_offset,
                   foc_resp_gyro_backup.y_offset,
                   foc_resp_gyro_backup.z_offset);

            /* wait for load gyro foc ready*/
            coines_delay_msec(10);

            printf("\r\nThe program will be exiting now ...\r\n");
            break;
        }

        /* register meta event */
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

        /* register sensor callback */
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_ACC_PASS, parse_3axis_s16, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_GYRO_PASS, parse_3axis_s16, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);

        /* process fifo */
        rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
        print_api_error(rslt, &bhy);

        /* Update the callback table to enable parsing of sensor data */
        rslt = bhi360_update_virtual_sensor_list(&bhy);
        print_api_error(rslt, &bhy);

        sensor_conf.sample_rate = BHI360_ODR_25_HZ; /* Read out data measured at 25Hz */
        sensor_conf.latency = 0;

        rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_ACC_PASS, &sensor_conf, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_GYRO_PASS, &sensor_conf, &bhy);
        print_api_error(rslt, &bhy);

        while (rslt == BHI360_OK)
        {
            if (loop >= limit)
            {
                flag = false;
                break;
            }

            if (get_interrupt_status())
            {
                /* Data from the FIFO is read and the relevant callbacks if registered are called */
                rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
                loop++;
                print_api_error(rslt, &bhy);
            }
        }
    }

    close_interfaces(intf);

    return rslt;
}

/*!
 * @brief To perform foc and crt
 */
static int8_t bhi3_perform_foc_crt(struct bhi360_foc_resp *acc_foc_status,
                                   struct bhi360_foc_resp *gyro_foc_status,
                                   struct bhi360_dev *dev,
                                   uint8_t flag,
                                   enum bhi360_intf intf)
{
    int8_t rslt = BHI360_OK;

    FILE *fp = fopen(CALIB_SEQ_FILE_NAME, "w");

    if (fp == NULL)
    {
        printf("Error to open file\r\n");

        return -1;
    }

    /* start acc foc process */
    if ((flag & 0x01) || flag == 0)
    {
        rslt = bhi360_perform_foc(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, acc_foc_status, dev);
        if (acc_foc_status->foc_status == SENSOR_FOC_FAIL)
        {
            printf("Acc foc process error, please make sure your device is available\r\n");
            rslt = SENSOR_FOC_FAIL;

            return rslt;
        }
        else if (acc_foc_status->foc_status == SENSOR_FOC_SUCCESS)
        {
            foc_resp_acc_backup.x_offset = acc_foc_status->x_offset;
            foc_resp_acc_backup.y_offset = acc_foc_status->y_offset;
            foc_resp_acc_backup.z_offset = acc_foc_status->z_offset;
            printf("Accel Status: %u\n", acc_foc_status->foc_status);
            printf("Accel FOC offsets: x %d, y %d, z %d\n",
                   acc_foc_status->x_offset,
                   acc_foc_status->y_offset,
                   acc_foc_status->z_offset);
            fprintf(fp,
                    "Latest Accel FOC offsets: x %d, y %d, z %d\r\n",
                    acc_foc_status->x_offset,
                    acc_foc_status->y_offset,
                    acc_foc_status->z_offset);
        }
    }

    /* start  gyro crt process */
    if ((flag & 0x02) || flag == 0)
    {

        rslt = bhi360_phy_sensor_ctrl_param_gyro_start_comp_retrim(dev);
        rslt = bhi360_phy_sensor_ctrl_param_gyro_get_crt_data(&data_crt, dev);
        if (rslt != BHI360_OK)
        {
            printf("CRT failed!\r\n");
            close_interfaces(intf);

            return 0;
        }

        if (data_crt.status == SENSOR_CRT_FAIL)
        {
            printf("CRT process error, please make sure your device is available\r\n");
            rslt = SENSOR_CRT_FAIL;

            return rslt;
        }
        else if (data_crt.status == SENSOR_CRT_SUCCESS)
        {
            printf("GYRO CRT status %d,  value x %d y %d z %d\r\n", data_crt.status, data_crt.x, data_crt.y,
                   data_crt.z);
            fprintf(fp,
                    "Latest GYRO CRT status %d, value: x %d, y %d, z %d\r\n",
                    data_crt.status,
                    data_crt.x,
                    data_crt.y,
                    data_crt.z);
        }
    }

    /* start  gyro foc process */
    if ((flag & 0x04) || flag == 0)
    {
        rslt = bhi360_perform_foc(BHI360_PHYS_SENSOR_ID_GYROSCOPE, gyro_foc_status, dev);
        if (gyro_foc_status->foc_status == SENSOR_FOC_FAIL)
        {
            printf("Gyro foc process error, please make sure your device is available\r\n");
            rslt = SENSOR_FOC_FAIL;

            return rslt;
        }
        else if (gyro_foc_status->foc_status == SENSOR_FOC_SUCCESS)
        {
            printf("GYRO Status: %u\n", gyro_foc_status->foc_status);
            printf("GYRO FOC offsets: x %d, y %d, z %d\n",
                   gyro_foc_status->x_offset,
                   gyro_foc_status->y_offset,
                   gyro_foc_status->z_offset);
            fprintf(fp,
                    "Latest GYRO FOC offsets: x %d, y %d, z %d",
                    gyro_foc_status->x_offset,
                    gyro_foc_status->y_offset,
                    gyro_foc_status->z_offset);
        }
    }

    fclose(fp);

    return rslt;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void parse_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
    float scaling_factor = 0.0f;

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

    uint8_t *accuracy = (uint8_t*)callback_ref;

    bhi360_event_data_parse_xyz(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);
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

static int16_t convert_char_int(char *line, char *pattern1, char *pattern2)
{
    char target[16] = { '\0' };
    char *start, *end;

    start = strstr(line, pattern1);
    if (start)
    {
        start += strlen(pattern1);
        if (pattern2 == NULL || strcmp(pattern2, "") == 0)
        {
            memcpy(target, start, strlen(start));
            target[strlen(start)] = '\0';
        }
        else if ((end = strstr(start, pattern2)) != NULL)
        {
            memcpy(target, start, end - start);
            target[end - start] = '\0';
        }
    }

    if (strcmp(target, "") != 0)
    {
        return (int16_t)atoi(target);
    }

    return INT16_MAX;
}