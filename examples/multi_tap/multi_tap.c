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
 * @file    multi_tap_param.c
 * @brief   Multi tap example for the BHI360
 *
 */

#include <stdio.h>

#include "bhi360_parse.h"
#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam.fw.h"

#define NUM_TAP_LOOP_COUNT  UINT8_C(5)

/*!
 * @brief Output of the multi tap data is parsed for printing
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 *
 */
static void parse_multitap(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t loop_cnt = 0U;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;

    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };

    bhi360_event_data_multi_tap buffer[8] = { BHI360_NO_TAP };
    bhi360_event_data_multi_tap multitap_setting = BHI360_TRIPLE_DOUBLE_SINGLE_TAP;
    bhi360_multi_tap_param_detector multitap_cnfg = { { 0U } };

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

        /*! Registering the callback functions */
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_MULTI_TAP, parse_multitap, NULL, &bhy);
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

    /*! Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    printf("--- Multi tap log start ---\r\n");

    rslt = bhi360_multi_tap_param_get_config(buffer, &bhy);
    print_api_error(rslt, &bhy);
    printf("Multi Tap Info : %s\r\n", bhi360_event_data_multi_tap_string_out[buffer[0]]);

    rslt = bhi360_multi_tap_param_set_config(&multitap_setting, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_multi_tap_param_detector_get_config(&multitap_cnfg, &bhy);
    print_api_error(rslt, &bhy);

    printf("Single Tap CNFG : %s\r\n",
           (((buffer[0] & (uint8_t)BHI360_SINGLE_TAP) == (uint8_t)BHI360_SINGLE_TAP) ? "Enabled" : "Disabled"));
    printf("    \t\t -<axis_sel> : %d\r\n", multitap_cnfg.stap_setting.as_s.axis_sel);
    printf("    \t\t -<wait_for_timeout> : %d\r\n", multitap_cnfg.stap_setting.as_s.wait_for_timeout);
    printf("    \t\t -<max_pks_for_tap> : %d\r\n", multitap_cnfg.stap_setting.as_s.max_peaks_for_tap);
    printf("    \t\t -<mode> : %d\r\n", multitap_cnfg.stap_setting.as_s.mode);
    printf("Double Tap CNFG : %s\r\n",
           (((buffer[0] & (uint8_t)BHI360_DOUBLE_TAP) == (uint8_t)BHI360_DOUBLE_TAP) ? "Enabled" : "Disabled"));
    printf("    \t\t -<tap_peak_thrs> : %d\r\n", multitap_cnfg.dtap_setting.as_s.tap_peak_thres);
    printf("    \t\t -<max_ges_dur> : %d\r\n", multitap_cnfg.dtap_setting.as_s.max_gesture_dur);
    printf("Triple Tap CNFG : %s\r\n",
           (((buffer[0] & (uint8_t)BHI360_TRIPLE_TAP) == (uint8_t)BHI360_TRIPLE_TAP) ? "Enabled" : "Disabled"));
    printf("    \t\t -<max_dur_bw_pks> : %d\r\n", multitap_cnfg.ttap_setting.as_s.max_dur_between_peaks);
    printf("    \t\t -<tap_shock_settl_dur> : %d\r\n", multitap_cnfg.ttap_setting.as_s.tap_shock_settling_dur);
    printf("    \t\t -<min_quite_dur_bw_taps> : %d\r\n", multitap_cnfg.ttap_setting.as_s.min_quite_dur_between_taps);
    printf("    \t\t -<quite_time_after_ges> : %d\r\n", multitap_cnfg.ttap_setting.as_s.quite_time_after_gesture);

    multitap_cnfg.stap_setting.as_s.mode = 0; /* Sensitive mode*/
    rslt = bhi360_multi_tap_param_detector_set_config(&multitap_cnfg, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_multi_tap_param_detector_get_config(&multitap_cnfg, &bhy);
    printf("Multi tap mode after setting : %d\r\n", multitap_cnfg.stap_setting.as_s.mode);
    print_api_error(rslt, &bhy);

    /*! Setting the Sampling frequency and latency time */
    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /*! Read out data measured at 100Hz */
    sensor_conf.latency = 0; /*! Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_MULTI_TAP, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_virtual_sensor_conf_param_get_cfg(BHI360_SENSOR_ID_MULTI_TAP, &sensor_conf, &bhy);

    printf("Multi tap sensor ID=%d, rate=%.2fHz, latency=%u\r\n",
           BHI360_SENSOR_ID_MULTI_TAP,
           sensor_conf.sample_rate,
           sensor_conf.latency);

    /*! Data from the FIFO is read and the relevant callbacks if registered are called */
    while ((rslt == BHI360_OK) && (loop_cnt < NUM_TAP_LOOP_COUNT))
    {
        if (get_interrupt_status())
        {
            /*! Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);

            loop_cnt++;
        }
    }

    printf("--- Multi tap log stop ---\r\n");
    close_interfaces(intf);

    return rslt;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void parse_multitap(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint32_t s, ns;
    uint64_t tns;

    bhi360_event_data_multi_tap multitap_data = BHI360_NO_TAP;

    if (callback_info->data_size != 2) /*! Check for a valid payload size. Includes sensor ID */
    {
        printf(" ERRORVAL\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    multitap_data = (bhi360_event_data_multi_tap)(*callback_info->data_ptr);

#ifndef PC
    printf("SID: %lu; T: %lu.%09lu; %s; \r\n", callback_info->sensor_id, s, ns,
           bhi360_event_data_multi_tap_string_out[multitap_data]);
#else
    printf("SID: %u; T: %u.%09u; %s; \r\n", callback_info->sensor_id, s, ns,
           bhi360_event_data_multi_tap_string_out[multitap_data]);
#endif
}
