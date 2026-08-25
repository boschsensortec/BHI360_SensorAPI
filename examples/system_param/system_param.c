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
 * @file    system_param.c
 * @brief   System parameters example
 *
 */

#include <stdio.h>
#include "common.h"
#include "bhi360_system_param.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350_BMP58X_BME688_bsxsam_ndof.fw.h"

int main(void)
{
    /*! Device structure */
    struct bhi360_dev bhy;
    enum bhi360_intf intf;
    int8_t rslt;
    uint16_t version = 0;

    uint8_t boot_status = 0;

    struct bhi360_system_param_phys_sensor_info psi;

    struct bhi360_system_param_timestamp ts;

    struct bhi360_system_param_firmware_version fw_ver;

    struct bhi360_system_param_fifo_control fifo_ctrl;

    bhi360_system_param_multi_meta_event_ctrl_t meta_event;

    /*! Selecting the SPI interface for sensor communication */
#ifdef BHI360_USE_I2C
    intf = BHI360_I2C_INTERFACE;
#else
    intf = BHI360_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

    init_sensor(&bhy, intf);

    setup_host_int_ctrl(&bhy);

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

    struct bhi360_system_param_orient_matrix ort_mtx = { { 0 } };
    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, &psi, &bhy);
    print_api_error(rslt, &bhy);

    printf("\r\n");
    printf("Field Name            hex                    | Value (dec)\r\n");
    printf("----------------------------------------------------------\r\n");
    printf("Physical Sensor ID    %02X                     | %d\r\n", psi.sensor_type, psi.sensor_type);
    printf("Driver ID             %02X                     | %d\r\n", psi.driver_id, psi.driver_id);
    printf("Driver Version        %02X                     | %d\r\n", psi.driver_version, psi.driver_version);
    printf("Current Consumption   %02X                     | %.3fmA\r\n", psi.power_current, psi.power_current / 10.f);
    printf("Dynamic Range         %04X                   | %d\r\n", psi.curr_range.u16_val, psi.curr_range.u16_val);

    const char *irq_status[2] = { "Disabled", "Enabled" };
    const char *master_intf[5] = { "None", "SPI0", "I2C0", "SPI1", "I2C1" };
    const char *power_mode[8] =
    { "Sensor Not Present", "Power Down", "Suspend", "Self-Test", "Interrupt Motion", "One Shot", "Low Power Active",
      "Active" };

    printf("Flags                 %02X                     | IRQ status       : %s\r\n", psi.flags,
           irq_status[psi.flags & 0x01]);
    printf("                                             | Master interface : %s\r\n",
           master_intf[(psi.flags >> 1) & 0x0F]);
    printf("                                             | Power mode       : %s\r\n",
           power_mode[(psi.flags >> 5) & 0x07]);
    printf("Slave Address         %02X                     | %d\r\n", psi.slave_address, psi.slave_address);
    printf("GPIO Assignment       %02X                     | %d\r\n", psi.gpio_assignment, psi.gpio_assignment);

    printf("Current Rate          %8u               | %.3fHz\r\n", psi.curr_rate.u32_val, psi.curr_rate.f_val);

    printf("Number of axes        %02X                     | %d\r\n", psi.num_axis, psi.num_axis);

        #define INT4_TO_INT8(INT4)  ((int8_t)(((INT4) > 1) ? -1 : (INT4)))

    ort_mtx.c[0] = (int8_t)((psi.orientation_matrix[0] & 0x0F) << 4) >> 4;
    ort_mtx.c[1] = (int8_t)(psi.orientation_matrix[0] & 0xF0) >> 4;
    ort_mtx.c[2] = (int8_t)((psi.orientation_matrix[1] & 0x0F) << 4) >> 4;
    ort_mtx.c[3] = (int8_t)(psi.orientation_matrix[1] & 0xF0) >> 4;
    ort_mtx.c[4] = (int8_t)((psi.orientation_matrix[2] & 0x0F) << 4) >> 4;
    ort_mtx.c[5] = (int8_t)(psi.orientation_matrix[2] & 0xF0) >> 4;
    ort_mtx.c[6] = (int8_t)((psi.orientation_matrix[3] & 0x0F) << 4) >> 4;
    ort_mtx.c[7] = (int8_t)(psi.orientation_matrix[3] & 0xF0) >> 4;
    ort_mtx.c[8] = (int8_t)((psi.orientation_matrix[4] & 0x0F) << 4) >> 4;

    printf("Orientation Matrix    %02X%02X%02X%02X%02X             | %+02d %+02d %+02d |\r\n",
           psi.orientation_matrix[0],
           psi.orientation_matrix[1],
           psi.orientation_matrix[2],
           psi.orientation_matrix[3],
           psi.orientation_matrix[4],
           ort_mtx.c[0],
           ort_mtx.c[1],
           ort_mtx.c[2]);
    printf("                                             | %+02d %+02d %+02d |\r\n",
           ort_mtx.c[3],
           ort_mtx.c[4],
           ort_mtx.c[5]);
    printf("                                             | %+02d %+02d %+02d |\r\n",
           ort_mtx.c[6],
           ort_mtx.c[7],
           ort_mtx.c[8]);
    printf("Reserved              %02X                     | %d\r\n", psi.reserved, psi.reserved);
    printf("\r\n");

    ort_mtx.c[0] = 0;

    rslt = bhi360_system_param_set_physical_sensor_info(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, &ort_mtx, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_system_param_get_physical_sensor_info(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, &psi, &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        ort_mtx.c[0] = (int8_t)((psi.orientation_matrix[0] & 0x0F) << 4) >> 4;
        printf("ort_mtx.c[0] after changed = %+02d\r\n", ort_mtx.c[0]);
    }

    rslt = bhi360_system_param_get_virtual_sensor_present(&bhy);
    print_api_error(rslt, &bhy);

    printf("\r\n");
    printf("Virtual sensor list.\r\n");
    printf("Sensor ID |                          Sensor Name\r\n");
    printf("----------+--------------------------------------|\r\n");

    for (uint8_t i = 0; i < BHI360_SENSOR_ID_MAX; i++)
    {
        if (bhi360_is_sensor_available(i, &bhy))
        {
            if (i < BHI360_SENSOR_ID_CUSTOM_START)
            {
                printf(" %8u | %36s \r\n", i, get_sensor_name(i));
            }
            else
            {
                printf(" %8u | Undefined custom sensor\n", i);
            }
        }
    }

    rslt = bhi360_system_param_get_physical_sensor_present(&bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("\r\n");
        printf("Physical sensor list.\r\n");
        printf("Sensor ID |                          Sensor Name\r\n");
        printf("----------+--------------------------------------|\r\n");
        for (uint8_t i = 0; i < BHI360_PHYSICAL_SENSOR_ID_MAX; i++)
        {
            if (bhi360_is_physical_sensor_available(i, &bhy))
            {
                printf(" %8u | %36s \r\n", i, get_physical_sensor_name(i));
            }
        }
    }

    rslt = bhi360_system_param_get_timestamps(&ts, &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("\r\n");
        uint32_t s, ns;
        ts.host_int_ts *= 15625; /* Timestamp is now in nanoseconds */
        s = (uint32_t)(ts.host_int_ts / UINT64_C(1000000000));
        ns = (uint32_t)(ts.host_int_ts - (s * UINT64_C(1000000000)));
        printf("Host interrupt timestamp: %u.%09u\r\n", s, ns);

        ts.cur_ts *= 15625; /* Timestamp is now in nanoseconds */
        s = (uint32_t)(ts.cur_ts / UINT64_C(1000000000));
        ns = (uint32_t)(ts.cur_ts - (s * UINT64_C(1000000000)));
        printf("Current timestamp: %u.%09u\r\n", s, ns);

        ts.event_ts *= 15625; /* Timestamp is now in nanoseconds */
        s = (uint32_t)(ts.event_ts / UINT64_C(1000000000));
        ns = (uint32_t)(ts.event_ts - (s * UINT64_C(1000000000)));

        printf("Timestamp evenT: %u .%09u\r\n", s, ns);
    }

    rslt = bhi360_system_param_get_firmware_version(&fw_ver, &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("\r\n");
        printf("Custom version number: %u\r\n", fw_ver.custom_ver_num);

    #ifdef PC

        /*lint -e10 Error 10: Lint does not understand PRIxxx */
        printf("EM Hash: %" PRIx64 "\r\n", fw_ver.em_hash);
        printf("BST Hash: %" PRIx64 "\r\n", fw_ver.bst_hash);
        printf("User Hash: %" PRIx64 "\r\n", fw_ver.user_hash);

        /*lint +e10 */
    #else
        char temp_em_hash[12 + 1] = { '\0' }, temp1_em_hash[8 + 1] = { '\0' }, temp2_em_hash[4 + 1] = { '\0' };
        sprintf(temp1_em_hash, "%08lx", (uint32_t)((fw_ver.em_hash >> 16) & 0xFFFFFFFF));
        sprintf(temp2_em_hash, "%04x", (uint16_t)(fw_ver.em_hash & 0xFFFF));

        strcat(temp_em_hash, temp1_em_hash);
        strcat(temp_em_hash, temp2_em_hash);
        printf("EM Hash: %s\r\n", temp_em_hash);

        char temp_bst_hash[12 + 1] = { '\0' }, temp1_bst_hash[8 + 1] = { '\0' }, temp2_bst_hash[4 + 1] = { '\0' };
        sprintf(temp1_bst_hash, "%08lx", (uint32_t)((fw_ver.bst_hash >> 16) & 0xFFFFFFFF));
        sprintf(temp2_bst_hash, "%04x", (uint16_t)(fw_ver.bst_hash & 0xFFFF));

        strcat(temp_bst_hash, temp1_bst_hash);
        strcat(temp_bst_hash, temp2_bst_hash);
        printf("BST Hash: %s\r\n", temp_bst_hash);

        char temp_user_hash[12 + 1] = { '\0' }, temp1_user_hash[8 + 1] = { '\0' }, temp2_user_hash[4 + 1] = { '\0' };
        sprintf(temp1_user_hash, "%08lx", (uint32_t)((fw_ver.user_hash >> 16) & 0xFFFFFFFF));
        sprintf(temp2_user_hash, "%04x", (uint16_t)(fw_ver.user_hash & 0xFFFF));

        strcat(temp_user_hash, temp1_user_hash);
        strcat(temp_user_hash, temp2_user_hash);
        printf("User Hash: %s\r\n", temp_user_hash);
    #endif
    }

    rslt = bhi360_system_param_get_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("\r\n");
        printf("Wakeup FIFO Watermark = %u\r\n", fifo_ctrl.wakeup_fifo_watermark);
        printf("Wakeup FIFO size =  %u\r\n", fifo_ctrl.wakeup_fifo_size);
        printf("Non Wakeup FIFO Watermark = %u\r\n", fifo_ctrl.non_wakeup_fifo_watermark);
        printf("Non Wakeup FIFO size = %u\r\n", fifo_ctrl.non_wakeup_fifo_size);
    }

    fifo_ctrl.wakeup_fifo_watermark = 500;
    rslt = bhi360_system_param_set_wakeup_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_system_param_get_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("Wakeup FIFO Watermark after changed =  %u\r\n", fifo_ctrl.wakeup_fifo_watermark);
    }

    fifo_ctrl.non_wakeup_fifo_watermark = 500;
    rslt = bhi360_system_param_set_nonwakeup_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_system_param_get_fifo_control(&fifo_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("Non Wakeup FIFO Watermark after changed = %u\r\n", fifo_ctrl.non_wakeup_fifo_watermark);
    }

    rslt = bhi360_system_param_get_meta_event_control(BHI360_SYSTEM_PARAM_META_EVENT_CONTROL_WAKE_UP_FIFO,
                                                      &meta_event,
                                                      &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("\r\n");
        printf("Meta event information:\r\n");
        for (uint8_t count = 0; count < 8; count++)
        {
            printf("%d %d %d %d %d %d %d %d\r\n",
                   meta_event.group[count].as_s.meta_event4_enable_state,
                   meta_event.group[count].as_s.meta_event4_int_enable_state,
                   meta_event.group[count].as_s.meta_event3_enable_state,
                   meta_event.group[count].as_s.meta_event3_int_enable_state,
                   meta_event.group[count].as_s.meta_event2_enable_state,
                   meta_event.group[count].as_s.meta_event2_int_enable_state,
                   meta_event.group[count].as_s.meta_event1_enable_state,
                   meta_event.group[count].as_s.meta_event1_int_enable_state);
        }
    }

    meta_event.group[0].as_uint8 = 128;
    rslt = bhi360_system_param_set_meta_event_control(BHI360_SYSTEM_PARAM_META_EVENT_CONTROL_WAKE_UP_FIFO,
                                                      &meta_event,
                                                      &bhy);
    print_api_error(rslt, &bhy);

    if (rslt == BHI360_OK)
    {
        printf("\r\n");
        printf("Meta event information after changed:\r\n");

        for (uint8_t idx = 0; idx < 8; idx++)
        {
            printf("%d %d %d %d %d %d %d %d\r\n",
                   meta_event.group[idx].as_s.meta_event4_enable_state,
                   meta_event.group[idx].as_s.meta_event4_int_enable_state,
                   meta_event.group[idx].as_s.meta_event3_enable_state,
                   meta_event.group[idx].as_s.meta_event3_int_enable_state,
                   meta_event.group[idx].as_s.meta_event2_enable_state,
                   meta_event.group[idx].as_s.meta_event2_int_enable_state,
                   meta_event.group[idx].as_s.meta_event1_enable_state,
                   meta_event.group[idx].as_s.meta_event1_int_enable_state);
        }
    }

    /*! Close all the active communication */
    close_interfaces(intf);

    return rslt;
}