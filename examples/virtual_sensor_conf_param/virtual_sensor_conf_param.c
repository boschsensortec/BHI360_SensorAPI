/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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
 * @file    virtual_sensor_conf_param.c
 * @brief   Virtual sensor configuration example with sensor id
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "bhi360.h"
#include "common.h"
#include "bhi360_virtual_sensor_conf_param.h"
#include "bhi360_event_data.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350C_BMP580_BME688.fw.h"

#define BHI360_SENSOR_ID_GYROSCOPE_PASSTHROUGH  UINT8_C(10)
#define CUSTOM_SENSOR_ID                        BHI360_SENSOR_ID_GYROSCOPE_PASSTHROUGH

/*!
 * @brief Printing the error codes that generated from the API
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 *
 */
static void print_api_error(int8_t rslt, struct bhi360_dev *dev);

/*! File pointer to download firmware to BHI360 */
static void upload_firmware(uint8_t boot_stat, struct bhi360_dev *dev);

/*! Device structure */
struct bhi360_dev bhy;

/*! Parameter for time in second and nano seconds */
uint32_t seconds, nano_seconds;

enum bhi360_intf intf;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;

    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    /*! Selecting the SPI interface for sensor communication */
#ifdef BHI360_USE_I2C
    intf = BHI360_I2C_INTERFACE;
#else
    intf = BHI360_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHI360_USE_I2C
    rslt = bhi360_init(BHI360_I2C_INTERFACE,
                       bhi360_i2c_read,
                       bhi360_i2c_write,
                       bhi360_delay_us,
                       BHI360_RD_WR_LEN,
                       NULL,
                       &bhy);
#else
    rslt = bhi360_init(BHI360_SPI_INTERFACE,
                       bhi360_spi_read,
                       bhi360_spi_write,
                       bhi360_delay_us,
                       BHI360_RD_WR_LEN,
                       NULL,
                       &bhy);
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

    /*! Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI360_ICTL_DISABLE_STATUS_FIFO | BHI360_ICTL_DISABLE_DEBUG;

    rslt = bhi360_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);
    rslt = bhi360_get_host_interrupt_ctrl(&hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHI360_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHI360_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHI360_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /*! Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi360_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /*! Check if the sensor is ready to load firmware */
    rslt = bhi360_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI360_BST_HOST_INTERFACE_READY)
    {
        upload_firmware(boot_status, &bhy);

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

    /*! Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_virtual_sensor_conf_param_get_cfg(CUSTOM_SENSOR_ID, &sensor_conf, &bhy);

    /*lint -e10 Error 10: Lint does not understand PRIxxx */
    printf("Custom sensor ID=%d, rate=%.2fHz,latency=%" PRIu32 ", range=%d\r\n",
           CUSTOM_SENSOR_ID,
           sensor_conf.sample_rate,
           sensor_conf.latency,
           sensor_conf.range);

    /*lint +e10 */

    sensor_conf.sample_rate = 100.0f; /*! Read out data measured at 100Hz */
    sensor_conf.latency = 0; /*! Report immediately */

    /*! Setting the Sampling frequency and latency time */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(CUSTOM_SENSOR_ID, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_virtual_sensor_conf_param_get_cfg(CUSTOM_SENSOR_ID, &sensor_conf, &bhy);

    /*lint -e10 Error 10: Lint does not understand PRIxxx */
    printf("Custom sensor ID=%d, rate=%.2fHz,latency=%" PRIu32 ", range=%d\r\n",
           CUSTOM_SENSOR_ID,
           sensor_conf.sample_rate,
           sensor_conf.latency,
           sensor_conf.range);

    /*lint +e10 */

    /*! Close all the active communication */
    close_interfaces(intf);

    return rslt;
}

static void print_api_error(int8_t rslt, struct bhi360_dev *dev)
{
    if (rslt != BHI360_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHI360_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHI360_INTF_RET_SUCCESS;
        }

        exit(0);
    }
}

/*!
 * @brief Uploading the firmware to the sensor RAM
 * @param[in] dev Firmware data available here
 *
 * @return  rslt execution result
 *
 */
static void upload_firmware(uint8_t boot_stat, struct bhi360_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHI360_OK;

    printf("Loading firmware into RAM.\r\n");
    rslt = bhi360_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), dev);
    temp_rslt = bhi360_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);

    printf("Booting from RAM.\r\n");
    rslt = bhi360_boot_from_ram(dev);

    temp_rslt = bhi360_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);
}
