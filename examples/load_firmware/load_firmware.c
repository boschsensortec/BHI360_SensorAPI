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
 * @file    load_firmware.c
 * @brief   Example to load firmware for the BHI360
 *
 */

#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_bsxsam.fw.h"

static int8_t upload_firmware_partly(struct bhi360_dev *dev);

int main(void)
{
    enum bhi360_intf intf;
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;

    uint8_t hintr_ctrl, hif_ctrl, boot_status;

#ifdef BHI360_USE_I2C
    intf = BHI360_I2C_INTERFACE;
#else
    intf = BHI360_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHI360_USE_I2C
    rslt = bhi360_init(intf, bhi360_i2c_read, bhi360_i2c_write, bhi360_delay_us, BHI360_RD_WR_LEN, NULL, &bhy);
#else
    rslt = bhi360_init(intf, bhi360_spi_read, bhi360_spi_write, bhi360_delay_us, BHI360_RD_WR_LEN, NULL, &bhy);
#endif
    print_api_error(rslt, &bhy);

    rslt = bhi360_soft_reset(&bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_get_chip_id(&chip_id, &bhy);
    print_api_error(rslt, &bhy);

    /* Check for a valid Chip ID */
    if (chip_id == BHI360_CHIP_ID)
    {
        printf("Chip ID read 0x%X\r\n", chip_id);
    }
    else
    {
        printf("Device not found. Chip ID read 0x%X\r\n", chip_id);
    }

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI360_ICTL_DISABLE_STATUS_FIFO | BHI360_ICTL_DISABLE_DEBUG;

    rslt = bhi360_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi360_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check if the sensor is ready to load firmware */
    rslt = bhi360_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI360_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;
        printf("Loading firmware.\r\n");

        rslt = upload_firmware_partly(&bhy);
        temp_rslt = bhi360_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

        printf("Booting from RAM.\r\n");
        rslt = bhi360_boot_from_ram(&bhy);

        temp_rslt = bhi360_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

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

    close_interfaces(intf);

    return rslt;
}

static int8_t upload_firmware_partly(struct bhi360_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhi360_firmware_image);
    int8_t rslt = BHI360_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHI360_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

        rslt = bhi360_upload_firmware_to_ram_partly(&bhi360_firmware_image[i], len, i, incr, dev);

        printf("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
    }

    printf("\n");

    return rslt;
}
