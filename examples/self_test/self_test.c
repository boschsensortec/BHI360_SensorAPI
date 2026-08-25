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
 * @file    self_test.c
 * @brief   Example for self-test
 *
 */

#include <stdio.h>
#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam.fw.h"

/*! @brief Prints self test response.
 *
 *  @param[in] self_test_resp : Self test response.
 *  @param[in] dev            : Device reference.
 */
static void print_self_test_resp(struct bhi360_self_test_resp *self_test_resp, struct bhi360_dev *dev);

int main(void)
{
    enum bhi360_intf intf;

    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t boot_status = 0;
    struct bhi360_self_test_resp self_test_resp;

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
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    printf("1. Self test pass case:\r\n");
    printf("Self test in progress\r\n");
    rslt = bhi360_perform_self_test(BHI360_PHYS_SENSOR_ID_ACCELEROMETER, &self_test_resp, &bhy);
    print_api_error(rslt, &bhy);

    print_self_test_resp(&self_test_resp, &bhy);

    printf("2. Self test failure case: Physical sensor ID not supported\r\n");
    printf("Self test in progress\r\n");

    /*! Clear the self test response */
    memset(&self_test_resp, 0, sizeof(struct bhi360_self_test_resp));
    rslt = bhi360_perform_self_test(BHI360_PHYS_SENSOR_ID_NOT_SUPPORTED, &self_test_resp, &bhy);
    print_api_error(rslt, &bhy);

    print_self_test_resp(&self_test_resp, &bhy);

    close_interfaces(intf);

    return rslt;
}

static void print_self_test_resp(struct bhi360_self_test_resp *self_test_resp, struct bhi360_dev *dev)
{

    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (self_test_resp == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        switch (self_test_resp->test_status)
        {
            case 0:
                printf("Test passed\r\n");
                break;
            case 1:
                printf("X axis failed\r\n");
                break;
            case 2:
                printf("Y axis failed\r\n");
                break;
            case 4:
                printf("Z axis failed\r\n");
                break;
            case 7:
                printf("Multiple axis failure / single test failed\r\n");
                break;
            case 8:
                printf("Physical sensor ID doesn't support self test\r\n");
                break;
            case 9:
                printf("Physical sensor ID not supported\r\n");
                break;
            default:
                printf("Undefined self test status %u", self_test_resp->test_status);
                break;
        }
        printf("Self test offset, X: %d, Y: %d, Z %d\r\n",
               self_test_resp->x_offset,
               self_test_resp->y_offset,
               self_test_resp->z_offset);
    }

    print_api_error(rslt, dev);
}