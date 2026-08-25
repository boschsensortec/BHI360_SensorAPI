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
 * @file    head_orientation.c
 * @brief   Head orientation example for the BHY
 *
 */

#include "common.h"
#include "bhi360_parse.h"
#include "bhi360_event_data.h"
#include "bhi360/Bosch_Shuttle3_BHI360_BMM350_bsxsam_ndof_HeadOrientation.fw.h"

#define USING_HO_EULER                      UINT8_C(1)
#define PARSE_DATA_WINDOW_SIZE              UINT16_C(3000)

#define HEAD_ORI_QUATERNION_ACCURACY_INDEX  UINT8_C(0)
#define HEAD_ORI_EULER_ACCURACY_INDEX       UINT8_C(1)
#define HEAD_ORI_MIS_ALG_ACCURACY_INDEX     UINT8_C(2)

static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_quaternion(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_euler(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt = BHI360_OK;
    struct bhi360_dev bhy;
    uint8_t boot_status;
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    uint8_t accuracy[3]; /* Accuracy is reported as a meta event. It is being printed alongside the data */

    bhi360_head_feature_param_ver hmc_ver = { 0 };
    bhi360_head_feature_param_ver ho_version = { 0 };
    bhi360_head_feature_param_misalignment_config misalign_config = { 0 };
    uint8_t ho_quat_head_corr_state[4] = { 0 };
    bhi360_head_feature_param_misalignment_quat_corr hmc_quat_corr = { { 0 } };
    bhi360_head_feature_param_misalignment_mode_vector_x hmc_mode_vect_x = { 0 };
    uint8_t ndof_init_head_corr_state = 0;

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

        /* Update the callback table to enable parsing of sensor data */
        rslt = bhi360_update_virtual_sensor_list(&bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT, parse_meta_event, (void*)accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)accuracy, &bhy);
        print_api_error(rslt, &bhy);

        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_IMU_HEAD_ORI_Q,
                                                parse_quaternion,
                                                (void*)&accuracy[HEAD_ORI_QUATERNION_ACCURACY_INDEX],
                                                &bhy);
        print_api_error(rslt, &bhy);

#ifdef USING_HO_EULER
        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_IMU_HEAD_ORI_E,
                                                parse_euler,
                                                (void*)&accuracy[HEAD_ORI_EULER_ACCURACY_INDEX],
                                                &bhy);
        print_api_error(rslt, &bhy);
#endif

        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG,
                                                   parse_quaternion,
                                                   (void*)&accuracy[HEAD_ORI_MIS_ALG_ACCURACY_INDEX],
                                                   &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi360_clear_fifo(0xFF, &bhy);
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
    rslt = bhi360_head_feature_param_get_hmc_version(&hmc_ver, &bhy);
    print_api_error(rslt, &bhy);

    printf("\n");
    printf("Head Misalignment Calibrator version: %u.%u.%u\r\n", hmc_ver.major, hmc_ver.minor, hmc_ver.patch);

    rslt = bhi360_head_feature_param_get_ho_version(&ho_version, &bhy);
    print_api_error(rslt, &bhy);

    printf("\n");
    printf("Head Orientation version: %u.%u.%u\r\n", ho_version.major, ho_version.minor, ho_version.patch);

    /* Set the mode for hmc algorithm:
       1. Two-step calibration Mode = 0
       2. Semi-automatic calibration Mode = 1
       vector_x is the X-axis of the head measured in the sensorcoordinate system
       vector_x is used in Semi-automatic calibration, default is [0, 0, 1], and the
       precise value vector_x must be used if Semi-automatic calibration is enabled  */
    rslt = bhi360_head_feature_param_get_hmc_mode_vector_x(&hmc_mode_vect_x, &bhy);
    print_api_error(rslt, &bhy);
    printf("HMC Mode: 0x%02x, Vector X: %f, Vector Y: %f, Vector Z: %f \r\n",
           hmc_mode_vect_x.mode,
           hmc_mode_vect_x.vector_x_0.f_val,
           hmc_mode_vect_x.vector_x_1.f_val,
           hmc_mode_vect_x.vector_x_2.f_val);

    hmc_mode_vect_x.mode = 0;
    rslt = bhi360_head_feature_param_set_hmc_mode_vector_x(&hmc_mode_vect_x, &bhy);
    print_api_error(rslt, &bhy);
    printf("Set HMC Mode to %s\r\n",
           (hmc_mode_vect_x.mode == 0) ? "Two-step calibration Mode" : "Semi-automatic calibration Mode");

    /* Get the configuration of hmc algorithm and fine-tuned if needed */
    rslt = bhi360_head_feature_param_get_hmc_configuration(&misalign_config, &bhy);
    print_api_error(rslt, &bhy);

    printf("\n");
    printf("Head Misalignment Configuration\r\n");

    printf("    - Still phase max duration: %u\r\n", misalign_config.still_phase_max_dur);
    printf("    - Still phase min duration: %u\r\n", misalign_config.still_phase_min_dur);
    printf("    - Still phase max samples: %u\r\n", misalign_config.still_phase_max_samples);

#ifndef PC
    printf("    - Accelerometer difference threshold: %u\r\n", misalign_config.acc_diff_threshold);
#else
    printf("    - Accelerometer difference threshold: %09u\r\n", misalign_config.acc_diff_threshold);
#endif

    rslt = bhi360_head_feature_param_set_hmc_configuration(&misalign_config, &bhy);
    print_api_error(rslt, &bhy);
    printf("Set Head Misalignment Configuration successfully \r\n");

    /* Set the configuration of hmc algorithm to default */
    rslt = bhi360_head_feature_param_set_default_hmc_cfg(&bhy);
    print_api_error(rslt, &bhy);
    printf("Set Head Misalignment Configuration to Default successfully\r\n");

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
            if (accuracy[HEAD_ORI_MIS_ALG_ACCURACY_INDEX] == 1)
            {
                printf("\nPlease keep your head still.\r\n");
            }
            else if (accuracy[HEAD_ORI_MIS_ALG_ACCURACY_INDEX] == 2)
            {
                printf("\nPlease do nod movement.\r\n");
            }
            else if (accuracy[HEAD_ORI_MIS_ALG_ACCURACY_INDEX] == 3)
            {
                printf("\nCalibration successfully!\r\n");
                break;
            }
            else if (accuracy[HEAD_ORI_MIS_ALG_ACCURACY_INDEX] == 0)
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

    /* Get the HMC calibration correction quaternion */
    rslt = bhi360_head_feature_param_get_hmc_quat_cal_cor_cfg(&hmc_quat_corr, &bhy);
    print_api_error(rslt, &bhy);

    printf("\n");
    printf("Head Misalignment Quaternion Correction Configuration\r\n");
    printf("    - Quaternion x : %f\r\n", hmc_quat_corr.quaternion_x.f_val);
    printf("    - Quaternion y : %f\r\n", hmc_quat_corr.quaternion_y.f_val);
    printf("    - Quaternion z : %f\r\n", hmc_quat_corr.quaternion_z.f_val);
    printf("    - Quaternion w : %f\r\n", hmc_quat_corr.quaternion_w.f_val);
    printf("    - Quaternion accuracy : %f\r\n", hmc_quat_corr.accuracy.f_val);

    /* Get the HMC correction quaternion state */
    rslt = bhi360_head_feature_param_get_quat_init_head_corr(ho_quat_head_corr_state, &bhy);
    print_api_error(rslt, &bhy);
    printf("\n");
    printf("Quaternion Initial Heading Correction Status : %s\r\n",
           (ho_quat_head_corr_state[0] == BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ENABLE) ? "Enabled" : "Disabled");

    /* Enable the inital heading correction */
    ho_quat_head_corr_state[0] = BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ENABLE;
    rslt = bhi360_head_feature_param_set_quat_init_head_corr(ho_quat_head_corr_state, &bhy);
    print_api_error(rslt, &bhy);

    /* Set the HMC calibration correction quaternion */
    rslt = bhi360_head_feature_param_set_hmc_quat_cal_cor_cfg(&hmc_quat_corr, &bhy);
    print_api_error(rslt, &bhy);

    /* Get Ndof initial heading correction state */
    rslt = bhi360_head_feature_param_get_ndof_init_head_corr(&ndof_init_head_corr_state, &bhy);
    print_api_error(rslt, &bhy);
    printf("Ndof Initial Heading Correction Status : %s\r\n",
           (ndof_init_head_corr_state == BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ENABLE) ? "Enabled" : "Disabled");

    /* Set Ndof initial heading correction state */
    ndof_init_head_corr_state = BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ENABLE;
    rslt = bhi360_head_feature_param_set_ndof_init_head_corr(&ndof_init_head_corr_state, &bhy);
    print_api_error(rslt, &bhy);

    /* Get Ndof initial heading correction state again to verify */
    rslt = bhi360_head_feature_param_get_ndof_init_head_corr(&ndof_init_head_corr_state, &bhy);
    print_api_error(rslt, &bhy);
    printf("Ndof Initial Heading Correction Status : %s\r\n",
           (ndof_init_head_corr_state == BHI360_OC_PARAM_INITIAL_HEADING_CORRECTION_ENABLE) ? "Enabled" : "Disabled");

    /* Enabling head orientation quaternion virtual sensor */
    sensor_conf.sample_rate = BHI360_ODR_25_HZ; /* 25Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_IMU_HEAD_ORI_Q, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_IMU_HEAD_ORI_Q), sensor_conf.sample_rate);

#ifdef USING_HO_EULER

    /* Enabling head orientation euler virtual sensor */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_IMU_HEAD_ORI_E, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_IMU_HEAD_ORI_E), sensor_conf.sample_rate);
#endif

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

    /* Disabling head orientation quaternion virtual sensor */
    sensor_conf.sample_rate = BHI360_ODR_0_HZ; /*! Disable the sensor by setting sample rate to 0Hz */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_IMU_HEAD_ORI_Q, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("\nDisable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_IMU_HEAD_ORI_Q));

    /* Disabling head orientation euler virtual sensor */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_IMU_HEAD_ORI_E, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Disable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_IMU_HEAD_ORI_E));

    /* Disabling head orientation misallignement virtual sensor */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Disable %s.\r\n", get_sensor_name(BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG));

    close_interfaces(intf);

    return rslt;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
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

static void parse_euler(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhi360_event_data_orientation data;
    uint32_t s, ns;
    uint64_t timestamp;

    uint8_t *accuracy = (uint8_t*)callback_ref;
    if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhi360_event_data_parse_orientation(callback_info->data_ptr, &data);

    timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    if (accuracy)
    {
#ifndef PC
        printf("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f; acc: %u\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f,
               *accuracy);
#else
        printf("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f; acc: %u\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f,
               *accuracy);
#endif
    }
    else
    {
#ifndef PC
        printf("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f);
#else
        printf("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.heading * 360.0f / 32768.0f,
               data.pitch * 360.0f / 32768.0f,
               data.roll * 360.0f / 32768.0f);
#endif
    }
}

static void parse_meta_event_extended(uint8_t meta_event_type, uint8_t byte1, uint8_t byte2, char*event_text)
{
    switch (meta_event_type)
    {
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
                    case BHI360_SENSOR_ID_IMU_HEAD_ORI_Q:
                        accuracy[HEAD_ORI_QUATERNION_ACCURACY_INDEX] = byte2;
                        break;

                    case BHI360_SENSOR_ID_IMU_HEAD_ORI_E:
                        accuracy[HEAD_ORI_EULER_ACCURACY_INDEX] = byte2;
                        break;
                    case BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG:
                        accuracy[HEAD_ORI_MIS_ALG_ACCURACY_INDEX] = byte2;
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
        default:
            parse_meta_event_extended(meta_event_type, byte1, byte2, event_text);
            break;
    }
}