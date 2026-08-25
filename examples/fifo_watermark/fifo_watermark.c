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
 * @file    fifo_watermark.c
 * @brief   Example for FIFO watermark handling
 *
 */

#include <stdio.h>
#include "common.h"
#include "bhi360.h"
#include "bhi360_parse.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam.fw.h"

static void parse_3axis(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

static void parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

#define PARSE_DATA_WINDOW_SIZE  UINT16_C(3000)

#define ACC_ACCURACY_INDEX      UINT8_C(0)
#define GYRO_ACCURACY_INDEX     UINT8_C(1)

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;

    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    uint8_t accuracy[2] = { 0 };

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

        /* register sensor callback */
        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_ACC,
                                                parse_3axis,
                                                (void*)&accuracy[ACC_ACCURACY_INDEX],
                                                &bhy);
        print_api_error(rslt, &bhy);
        rslt =
            bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_GYRO,
                                                parse_3axis,
                                                (void*)&accuracy[GYRO_ACCURACY_INDEX],
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

    /* Check fifo size and watermark firstly */
    struct bhi360_system_param_fifo_control fifo_ctrl = { 0 };
    rslt = bhi360_system_param_get_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);
    printf(
        "FIFO control info: \r\n wakeup_fifo_size=%u, non_wakeup_fifo_size=%u \r\n wakeup_fifo_watermark=%u, non_wakeup_fifo_watermark=%u\n",
        fifo_ctrl.wakeup_fifo_size,
        fifo_ctrl.non_wakeup_fifo_size,
        fifo_ctrl.wakeup_fifo_watermark,
        fifo_ctrl.non_wakeup_fifo_watermark);

    /* Set none-wakeup fifo watermark and ensure that all FIFO data is fully read out upon each watermark interrupt */
    fifo_ctrl.non_wakeup_fifo_watermark = 256;
    rslt = bhi360_system_param_set_nonwakeup_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Set wakeup fifo watermark ensure that all FIFO data is fully read out upon each watermark interrupt */
    fifo_ctrl.wakeup_fifo_watermark = 256;
    rslt = bhi360_system_param_set_wakeup_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check watermark after setting */
    fifo_ctrl.non_wakeup_fifo_watermark = 0;
    fifo_ctrl.wakeup_fifo_watermark = 0;
    rslt = bhi360_system_param_get_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);
    printf(
        "FIFO control info: \r\n wakeup_fifo_size=%u, non_wakeup_fifo_size=%u\r\n wakeup_fifo_watermark=%u, non_wakeup_fifo_watermark=%u\n",
        fifo_ctrl.wakeup_fifo_size,
        fifo_ctrl.non_wakeup_fifo_size,
        fifo_ctrl.wakeup_fifo_watermark,
        fifo_ctrl.non_wakeup_fifo_watermark);

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* Read out data measured at 100Hz */

    /* Please make sure the watermark interrupt occurs before the latency arrives.
       Here 1000ms report latency, If it is 0, then report immediately, unit: ms */
    sensor_conf.latency = 1000;
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_ACC, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz, latency: %u\r\n",
           get_sensor_name(BHI360_SENSOR_ID_ACC),
           sensor_conf.sample_rate,
           sensor_conf.latency);

    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_GYRO, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz, latency: %u\r\n",
           get_sensor_name(BHI360_SENSOR_ID_GYRO),
           sensor_conf.sample_rate,
           sensor_conf.latency);

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

    close_interfaces(intf);

    return rslt;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void parse_3axis(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
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