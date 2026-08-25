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
 * @file    bsx_algo_param.c
 * @brief   Bsx algorithm configuration example for the BHY
 *
 */
#include <stdio.h>
#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350_bsxsam_ndof_turbo.fw.h"

#define PARAMETER_ID             0X201

#define CALIB_ACC_FILE_NAME      "calib_acc.bin"
#define CALIB_GYRO_FILE_NAME     "calib_gyro.bin"
#define CALIB_MAG_FILE_NAME      "calib_mag.bin"

#define ACC_ACCURACY_INDEX       UINT8_C(0)
#define GYRO_ACCURACY_INDEX      UINT8_C(1)
#define MAGNET_ACCURACY_INDEX    UINT8_C(2)

#define OPEN_FILE_ERR            INT8_C(-1)
#define READ_FILE_ERR            INT8_C(-2)
#define WRITE_FILE_ERR           INT8_C(-2)
#define FILE_TOO_LARGE_ERR       INT8_C(-3)
#define READ_WRITE_SUCCESS       UINT8_C(0)

#define BASE_ADDR                UINT16_C(0x200)
#define MAX_LENGTH               UINT32_C(2048)

#define NEED_UPDATE_CALIBRATION  UINT8_C(1)
#define UPDATE_CALIBRATION_DONE  UINT8_C(0)

#define CALIBRATION_COMPLETED    UINT8_C(3)

static void get_calib_from_sensor(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event_calib(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static void read_calib_from_file_and_set_to_bhi(uint8_t phy_sensor_id);
static int8_t write_calib_file(const char* filename, const uint8_t * calib_prof, uint32_t actual_len);
static int8_t read_calib_file(const char* filename, uint8_t* calib_prof, uint32_t max_len, uint32_t* actual_len);

struct bhi360_dev bhy;

/* Flag to indicate if offset update is needed */
static uint8_t offset_update_flag[3] = { 0 };

/* Variable to ensure getting calibration profile happens only one time*/
uint8_t first_run[3] = { 1, 1, 1 };

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt = BHI360_OK;
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    uint8_t boot_status = 0;
    struct bhi360_bsx_algo_param_version get_bsx_ver = { 0 };
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t accuracy[3];

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
                                                   parse_meta_event_calib,
                                                   (void*)accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU,
                                                   parse_meta_event_calib,
                                                   (void*)accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_ACC, get_calib_from_sensor, (void*)accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_GYRO, get_calib_from_sensor, (void*)accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_MAG, get_calib_from_sensor, (void*)accuracy, &bhy);
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

    /* get the BSX version */
    rslt = bhi360_bsx_algo_param_get_bsx_version(&get_bsx_ver, &bhy);
    print_api_error(rslt, &bhy);

    printf("\n");
    printf("BSX version : %u.%u.%u.%u\r\n",
           get_bsx_ver.major_version,
           get_bsx_ver.minor_version,
           get_bsx_ver.major_bug_fix_version,
           get_bsx_ver.minor_bug_fix_version);

    bhi360_bsx_algo_param_sic_matrix bsx_sic_matrix_exg = { 0 };

    /* Get bsx states for BSX fusion parameters*/

    rslt = bhi360_bsx_algo_param_get_bsx_sic_matrix(&bsx_sic_matrix_exg, &bhy);
    print_api_error(rslt, &bhy);

    printf("Get SIC Matrix of BSX parameter id %X:\r\n", BHI360_BSX_SIC_MATRIX);

    for (uint8_t idx = 0; idx < 9; idx++)
    {
        printf("%f ", bsx_sic_matrix_exg.sic[idx].f_val);
        if ((idx + 1) % 3 == 0)
        {
            printf("\r\n");
        }
    }

    /* Below SIC matrix is just an example, the actual values should be provided by customer or request BST for further
     * support */
    bsx_sic_matrix_exg.sic[0].f_val = 0.908583f;
    bsx_sic_matrix_exg.sic[1].f_val = 0.0187744f;
    bsx_sic_matrix_exg.sic[2].f_val = 0.0688107f;
    bsx_sic_matrix_exg.sic[3].f_val = 0.0187744f;
    bsx_sic_matrix_exg.sic[4].f_val = 0.858548f;
    bsx_sic_matrix_exg.sic[5].f_val = 0.0496729f;
    bsx_sic_matrix_exg.sic[6].f_val = 0.0688107f;
    bsx_sic_matrix_exg.sic[7].f_val = 0.0496729f;
    bsx_sic_matrix_exg.sic[8].f_val = 0.918619f;

    rslt = bhi360_bsx_algo_param_set_bsx_sic_matrix(&bsx_sic_matrix_exg, &bhy);
    print_api_error(rslt, &bhy);

    bhi360_bsx_algo_param_sic_matrix bsx_sic_matrix = { 0 };
    rslt = bhi360_bsx_algo_param_get_bsx_sic_matrix(&bsx_sic_matrix, &bhy);
    print_api_error(rslt, &bhy);

    printf("\r\nGetting BSX SIC Matrix again:\r\n");
    for (uint8_t idx = 0; idx < 9; idx++)
    {
        printf("%f ", bsx_sic_matrix.sic[idx].f_val);
        if ((idx + 1) % 3 == 0)
        {
            printf("\r\n");
        }
    }

    printf("\r\n\r\n");

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    read_calib_from_file_and_set_to_bhi(BHI360_PHYS_SENSOR_ID_ACCELEROMETER);
    read_calib_from_file_and_set_to_bhi(BHI360_PHYS_SENSOR_ID_GYROSCOPE);
    read_calib_from_file_and_set_to_bhi(BHI360_PHYS_SENSOR_ID_MAGNETOMETER);

    /* Read out data measured at 100Hz */
    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_ACC, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_ACC), sensor_conf.sample_rate);

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_GYRO, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_GYRO), sensor_conf.sample_rate);

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_MAG, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_MAG), sensor_conf.sample_rate);

    while (rslt == BHI360_OK)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);
            if ((accuracy[GYRO_ACCURACY_INDEX] == CALIBRATION_COMPLETED) &&
                (accuracy[ACC_ACCURACY_INDEX] == CALIBRATION_COMPLETED) &&
                (accuracy[MAGNET_ACCURACY_INDEX] == CALIBRATION_COMPLETED))
            {
                printf("The program will be exiting now ...\r\n");
                break;
            }
        }
    }

    close_interfaces(intf);

    return rslt;
}

static int8_t write_calib_file(const char* filename, const uint8_t * calib_prof, uint32_t actual_len)
{
    FILE* file = fopen(filename, "wb");

    if (file == NULL)
    {
        printf("no file\r\n");

        return OPEN_FILE_ERR; /* Error opening file */
    }

    /* Write the calibration profile data to the file */
    size_t written = fwrite(calib_prof, sizeof(uint8_t), actual_len, file);
    fclose(file);

    if (written != actual_len)
    {
        return WRITE_FILE_ERR; /* Error writing to file */
    }

    return READ_WRITE_SUCCESS; /* Success */
}

/**
 * @brief Helper functions toread calibration profiles from files.
 *
 * @param filename File to read from.
 * @param calib_prof Calibration profile buffer to store data for read.
 * @param actual_len maximum length to read , pointer to store the actual length read.
 *
 * @return int Returns an integer indicating success or type of failure.
 *
 * @note For reading, the `calib_prof` and `actual_len` are updated with the file contents and its length, respectively.
 */
static int8_t read_calib_file(const char* filename, uint8_t* calib_prof, uint32_t max_len, uint32_t* actual_len)
{
    FILE* file = fopen(filename, "rb");

    if (file == NULL)
    {
        return OPEN_FILE_ERR; /* Error opening file */
    }

    fseek(file, 0, SEEK_END);
    uint32_t file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size > max_len)
    {
        fclose(file);

        return FILE_TOO_LARGE_ERR; /* File contents too large */
    }

    /* Read the calibration profile data from the file */
    size_t read = fread(calib_prof, sizeof(uint8_t), file_size, file);
    fclose(file);

    if (read != file_size)
    {
        return READ_FILE_ERR; /* Error reading from file */
    }

    *actual_len = file_size; /* Set the actual length read */
    return READ_WRITE_SUCCESS; /* Success */
}

/**
 * @brief Reads calibration data from a file for a specified physical sensor.
 *
 * @param phy_sensor_id ID of the physical sensor (accelerometer, gyroscope).
 *
 * @note This function reads calibration data into a global buffer and updates the sensor calibration profile in the BHI2/3 device.
 */
static void read_calib_from_file_and_set_to_bhi(uint8_t phy_sensor_id)
{
    int8_t rslt = 0;
    uint16_t param_id = BASE_ADDR | phy_sensor_id;
    uint8_t calib_prof[WORK_BUFFER_SIZE] = { 0 };
    uint32_t actual_len = 0;
    bhi360_bsx_algo_param_state_exg bsx_state_exg[BHI360_BSX_STATE_MAX_BLOCKS] = { { 0 } };

    switch (phy_sensor_id)
    {
        case BHI360_PHYS_SENSOR_ID_ACCELEROMETER:

            /* Read accelerometer calibration profile from file */
            rslt = read_calib_file(CALIB_ACC_FILE_NAME, calib_prof, MAX_LENGTH, &actual_len);
            break;
        case BHI360_PHYS_SENSOR_ID_GYROSCOPE:

            /* Read gyroscope calibration profile from file */
            rslt = read_calib_file(CALIB_GYRO_FILE_NAME, calib_prof, MAX_LENGTH, &actual_len);
            break;

        case BHI360_PHYS_SENSOR_ID_MAGNETOMETER:

            /* Read magnetometer calibration profile from file */
            rslt = read_calib_file(CALIB_MAG_FILE_NAME, calib_prof, MAX_LENGTH, &actual_len);
            break;
        default:
            break;
    }

    if (rslt == BHI360_OK)
    {
        uint16_t num = 0;
        uint32_t state_buf_size;

        /* Initialize the BSX state structure */
        memset(bsx_state_exg, 0, sizeof(bsx_state_exg));

        /* Traverse each block in the BSX states until finished */
        for (uint32_t pos = 0; pos < actual_len; pos += BHI360_BSX_STATE_BLOCK_LEN, num++)
        {
            bsx_state_exg[num].block_info = num & BHI360_BSX_STATE_BLOCK_NUMBER;
            state_buf_size = actual_len - pos;
            if (state_buf_size <= BHI360_BSX_STATE_BLOCK_LEN)
            {
                bsx_state_exg[num].block_info |= BHI360_BSX_STATE_TRANSFER_COMPLETE;
                bsx_state_exg[num].block_len = (uint8_t)state_buf_size;
            }
            else
            {
                bsx_state_exg[num].block_len = BHI360_BSX_STATE_BLOCK_LEN;
            }

            bsx_state_exg[num].struct_len = actual_len;

            /* Copy the calibration profile data into the BSX state structure */
            memcpy(&(bsx_state_exg[num].state_data[0]), &calib_prof[pos], bsx_state_exg[num].block_len);
        }

        /* Set bsx states for physical sensors*/
        rslt = bhi360_bsx_algo_param_set_bsx_states(param_id, bsx_state_exg, &bhy);
        coines_delay_msec(100);

        if (rslt == BHI360_OK)
        {
            printf("Resume calibration to BSX Done with sensor id %d.\r\n", phy_sensor_id);
        }
        else
        {
            print_api_error(rslt, &bhy);
        }
    }
}

/**
 * @brief Writes calibration data to a file for a specified physical sensor if certain conditions are met.
 *
 * @param accuracy Accuracy level of the sensor data.
 * @param phy_sensor_id ID of the physical sensor.
 *
 * @note This function writes the calibration profile to a file if the accuracy is 3 and an update is required.
 */
static void get_calib_from_bhi_and_write_to_file(uint8_t accuracy, uint8_t phy_sensor_id, uint8_t idx)
{
    int8_t rslt = BHI360_OK;
    uint16_t param_id = BASE_ADDR | phy_sensor_id;
    uint32_t actual_len = 0;
    bhi360_bsx_algo_param_state_exg bsx_state_exg[BHI360_BSX_STATE_MAX_BLOCKS] = { { 0 } };
    uint8_t calib_prof[WORK_BUFFER_SIZE] = { 0 };

    if (accuracy == CALIBRATION_COMPLETED && offset_update_flag[idx] == NEED_UPDATE_CALIBRATION)
    {
        printf("\r\nCalibration done for sensor with id %d.\r\n", phy_sensor_id);

        /* Get bsx states for BSX fusion parameters*/
        rslt = bhi360_bsx_algo_param_get_bsx_states(param_id,
                                                    bsx_state_exg,
                                                    sizeof(bsx_state_exg) * 20,
                                                    &actual_len,
                                                    &bhy);
        print_api_error(rslt, &bhy);
        switch (idx)
        {
            case ACC_ACCURACY_INDEX:
                printf("Get Accelerometer Calibration profile\r\n");
                break;
            case GYRO_ACCURACY_INDEX:
                printf("Get Gyroscope Calibration profile\r\n");
                break;
            case MAGNET_ACCURACY_INDEX:
                printf("Get Magnetometer Calibration profile\r\n");
                break;
            default:
                break;
        }
        for (uint8_t block = 0; block < BHI360_BSX_STATE_MAX_BLOCKS; block++)
        {
            bhi360_bsx_algo_param_state_exg *current_block = &bsx_state_exg[block];

            for (uint8_t i = 0; i < current_block->block_len; i++)
            {
                printf("%02x ", current_block->state_data[i]);

                if ((i + 1) % 8 == 0 || i == current_block->block_len - 1)
                {
                    printf("\r\n");
                }
            }

            printf("\r\n");

            /* Copy the calibration profile data into the buffer */
            memcpy(calib_prof + block * 64, &(bsx_state_exg[block].state_data[0]), 64);

            if ((current_block->block_len == 0) ||
                (current_block->block_info & BHI360_BSX_STATE_TRANSFER_COMPLETE) != 0)
            {
                break;
            }
        }

        switch (phy_sensor_id)
        {
            case BHI360_PHYS_SENSOR_ID_ACCELEROMETER:

                /* Write accelerometer calibration profile to file*/
                rslt = write_calib_file(CALIB_ACC_FILE_NAME, calib_prof, actual_len);
                if (rslt == BHI360_OK)
                {
                    printf("Accelerometer calibration profile written to file %s successfully.\r\n",
                           CALIB_ACC_FILE_NAME);
                }

                break;
            case BHI360_PHYS_SENSOR_ID_GYROSCOPE:

                /* Write gyroscope calibration profile to file*/
                rslt = write_calib_file(CALIB_GYRO_FILE_NAME, calib_prof, actual_len);
                if (rslt == BHI360_OK)
                {
                    printf("Gyroscope calibration profile written to file %s successfully.\r\n", CALIB_GYRO_FILE_NAME);
                }

                break;

            case BHI360_PHYS_SENSOR_ID_MAGNETOMETER:

                /* Write magnetometer calibration profile to file*/
                rslt = write_calib_file(CALIB_MAG_FILE_NAME, calib_prof, actual_len);
                if (rslt == BHI360_OK)
                {
                    printf("Magnetometer calibration profile written to file %s successfully.\r\n",
                           CALIB_MAG_FILE_NAME);
                }

                break;
            default:
                break;
        }

        offset_update_flag[idx] = UPDATE_CALIBRATION_DONE;
    }
}

/**
 * @brief Parses and processes sensor data for accuracy, and triggers calibration file update if necessary.
 *
 * @param callback_info Information about the FIFO parse data.
 * @param callback_ref Reference data passed to the callback (in this case, accuracy data).
 *
 * @note This function prints sensor data and updates calibration files if needed.
 */
static void get_calib_from_sensor(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhi360_event_data_xyz data;
    uint8_t *accuracy = (uint8_t*)callback_ref;
    if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhi360_event_data_parse_xyz(callback_info->data_ptr, &data);

    if (accuracy)
    {
        switch (callback_info->sensor_id)
        {
            case BHI360_SENSOR_ID_ACC:
            case BHI360_SENSOR_ID_ACC_BIAS:
                if (accuracy[ACC_ACCURACY_INDEX] == CALIBRATION_COMPLETED)
                {
                    get_calib_from_bhi_and_write_to_file(accuracy[ACC_ACCURACY_INDEX],
                                                         BHI360_PHYS_SENSOR_ID_ACCELEROMETER,
                                                         ACC_ACCURACY_INDEX);
                }

                break;
            case BHI360_SENSOR_ID_GYRO:
            case BHI360_SENSOR_ID_GYRO_BIAS:
                if (accuracy[GYRO_ACCURACY_INDEX] == CALIBRATION_COMPLETED)
                {
                    get_calib_from_bhi_and_write_to_file(accuracy[GYRO_ACCURACY_INDEX],
                                                         BHI360_PHYS_SENSOR_ID_GYROSCOPE,
                                                         GYRO_ACCURACY_INDEX);
                }

                break;

            case BHI360_SENSOR_ID_MAG:
            case BHI360_SENSOR_ID_MAG_BIAS:
                if (accuracy[MAGNET_ACCURACY_INDEX] == CALIBRATION_COMPLETED)
                {
                    get_calib_from_bhi_and_write_to_file(accuracy[MAGNET_ACCURACY_INDEX],
                                                         BHI360_PHYS_SENSOR_ID_MAGNETOMETER,
                                                         MAGNET_ACCURACY_INDEX);
                }

                break;
            default:
                break;
        }

    }
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

static void parse_meta_event_calib(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
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
                        if (accuracy[ACC_ACCURACY_INDEX] == 3 && first_run[ACC_ACCURACY_INDEX])
                        {
                            offset_update_flag[ACC_ACCURACY_INDEX] = 1;
                            first_run[ACC_ACCURACY_INDEX] = 0;
                        }

                        break;
                    case BHI360_SENSOR_ID_GYRO_BIAS:
                    case BHI360_SENSOR_ID_GYRO:
                        accuracy[GYRO_ACCURACY_INDEX] = byte2;
                        if (accuracy[GYRO_ACCURACY_INDEX] == 3 && first_run[GYRO_ACCURACY_INDEX])
                        {
                            offset_update_flag[GYRO_ACCURACY_INDEX] = 1;
                            first_run[GYRO_ACCURACY_INDEX] = 0;
                        }

                        break;
                    case BHI360_SENSOR_ID_MAG_BIAS:
                    case BHI360_SENSOR_ID_MAG:
                        accuracy[MAGNET_ACCURACY_INDEX] = byte2;
                        if (accuracy[MAGNET_ACCURACY_INDEX] == 3 && first_run[MAGNET_ACCURACY_INDEX])
                        {
                            offset_update_flag[MAGNET_ACCURACY_INDEX] = 1;
                            first_run[MAGNET_ACCURACY_INDEX] = 0;
                        }

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