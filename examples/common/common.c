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
 * @file    common.c
 * @brief   Common source file for the BHy examples
 *
 */

#include "common.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "bhi360_parse.h"

#define ROBERT_BOSCH_USB_VID   (0x108C)
#define ARDUINO_USB_VID        (0x2341)
#define BST_APP31_CDC_USB_PID  (0xAB38)
#define BST_APP30_CDC_USB_PID  (0xAB3C)
#define BST_APP20_CDC_USB_PID  (0xAB2C)
#define ARDUINO_NICLA_USB_PID  (0x0060)

void verbose_write(uint8_t *buffer, uint16_t length);

#ifdef MCU_APP20
static enum coines_multi_io_pin cs_pin = BHY360_APP20_CS_PIN;
static enum coines_multi_io_pin int_pin = BHY360_APP20_INT_PIN;
static enum coines_multi_io_pin reset_pin = BHY360_APP20_RESET_PIN;
#else
static enum coines_multi_io_pin cs_pin = BHY360_APP30_CS_PIN;
static enum coines_multi_io_pin int_pin = BHY360_APP30_INT_PIN;
static enum coines_multi_io_pin reset_pin = BHY360_APP30_RESET_PIN;
#endif

bool get_interrupt_status(void)
{
    int16_t coines_rslt;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;

    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_get_pin_config(int_pin, &pin_direction, &pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        char *error_text = get_coines_error(coines_rslt);
        printf("Error getting interrupt pin status %s\r\n", error_text);
    }

    return pin_value == COINES_PIN_VALUE_HIGH;
}

static char *get_coines_error_extended(int16_t rslt)
{
    char *ret = " ";

    switch (rslt)
    {
        case COINES_E_I2C_BUS_NOT_ENABLED:
            ret = "[COINES Error] I2C bus not enabled";
            break;
        case COINES_E_I2C_CONFIG_FAILED:
            ret = "[COINES Error] I2C configuration failed";
            break;
        case COINES_E_I2C_CONFIG_EXIST:
            ret = "[COINES Error] I2C already configured";
            break;
        default:
            ret = "[COINES Error] Unknown error code";
    }

    return ret;
}

char *get_coines_error(int16_t rslt)
{
    char *ret = " ";

    switch (rslt)
    {
        case COINES_SUCCESS:
            break;
        case COINES_E_FAILURE:
            ret = "[COINES Error] Generic failure";
            break;
        case COINES_E_COMM_IO_ERROR:
            ret = "[COINES Error] Communication IO failed. Check connections with the sensor";
            break;
        case COINES_E_COMM_INIT_FAILED:
            ret = "[COINES Error] Communication initialization failed";
            break;
        case COINES_E_UNABLE_OPEN_DEVICE:
            ret = "[COINES Error] Unable to open device. Check if the board is in use";
            break;
        case COINES_E_DEVICE_NOT_FOUND:
            ret = "[COINES Error] Device not found. Check if the board is powered on";
            break;
        case COINES_E_UNABLE_CLAIM_INTF:
            ret = "[COINES Error] Unable to claim interface. Check if the board is in use";
            break;
        case COINES_E_MEMORY_ALLOCATION:
            ret = "[COINES Error] Error allocating memory";
            break;
        case COINES_E_NOT_SUPPORTED:
            ret = "[COINES Error] Feature not supported";
            break;
        case COINES_E_NULL_PTR:
            ret = "[COINES Error] Null pointer error";
            break;
        case COINES_E_COMM_WRONG_RESPONSE:
            ret = "[COINES Error] Unexpected response";
            break;
        case COINES_E_SPI16BIT_NOT_CONFIGURED:
            ret = "[COINES Error] 16-Bit SPI not configured";
            break;
        case COINES_E_SPI_INVALID_BUS_INTF:
            ret = "[COINES Error] Invalid SPI bus interface";
            break;
        case COINES_E_SPI_CONFIG_EXIST:
            ret = "[COINES Error] SPI already configured";
            break;
        case COINES_E_SPI_BUS_NOT_ENABLED:
            ret = "[COINES Error] SPI bus not enabled";
            break;
        case COINES_E_SPI_CONFIG_FAILED:
            ret = "[COINES Error] SPI configuration failed";
            break;
        case COINES_E_I2C_INVALID_BUS_INTF:
            ret = "[COINES Error] Invalid I2C bus interface";
            break;
        default:
            ret = get_coines_error_extended(rslt);
    }

    return ret;
}

char *get_api_error(int8_t error_code)
{
    char *ret = " ";

    switch (error_code)
    {
        case BHI360_OK:
            break;
        case BHI360_E_NULL_PTR:
            ret = "[API Error] Null pointer";
            break;
        case BHI360_E_INVALID_PARAM:
            ret = "[API Error] Invalid parameter";
            break;
        case BHI360_E_IO:
            ret = "[API Error] IO error";
            break;
        case BHI360_E_MAGIC:
            ret = "[API Error] Invalid firmware";
            break;
        case BHI360_E_TIMEOUT:
            ret = "[API Error] Timed out";
            break;
        case BHI360_E_BUFFER:
            ret = "[API Error] Invalid buffer";
            break;
        case BHI360_E_INVALID_FIFO_TYPE:
            ret = "[API Error] Invalid FIFO type";
            break;
        case BHI360_E_INVALID_EVENT_SIZE:
            ret = "[API Error] Invalid Event size";
            break;
        case BHI360_E_PARAM_NOT_SET:
            ret = "[API Error] Parameter not set";
            break;
        default:
            ret = "[API Error] Unknown API error code";
    }

    return ret;
}

void setup_interfaces(bool reset_power, enum bhi360_intf intf)
{
    int16_t coines_rslt = COINES_SUCCESS;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;
    struct coines_board_info board_info;
    char *error_text;

#ifndef PC
    struct coines_ble_config ble_config;
    ble_config.name = NULL;
    ble_config.tx_power = COINES_TX_POWER_8_DBM;
    coines_rslt = coines_ble_config(&ble_config);
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);
#else
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
#endif
    if (coines_rslt)
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\n", error_text);
    }

    coines_rslt = coines_get_board_info(&board_info);
    if (coines_rslt == COINES_SUCCESS)
    {
        if (board_info.board == 5) /* Application Board 3.0 */
        {
            cs_pin = BHY360_APP30_CS_PIN;
            int_pin = BHY360_APP30_INT_PIN;
            reset_pin = BHY360_APP30_RESET_PIN;
        }
    }
    else
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\n", error_text);
    }

    if (reset_power)
    {
        coines_rslt = coines_set_shuttleboard_vdd_vddio_config(0, 0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\n", error_text);
        }

        pin_direction = COINES_PIN_DIRECTION_OUT;
        pin_value = COINES_PIN_VALUE_LOW;
        coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\n", error_text);
        }

        coines_delay_msec(10);
    }

    if (intf == BHI360_SPI_INTERFACE)
    {
        printf("Host Interface : SPI\r\n");
        coines_rslt = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_1_MHZ, COINES_SPI_MODE0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to SPI %s\r\n", error_text);
        }
    }
    else
    {
        printf("Host Interface : I2C\r\n");
        coines_rslt = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to I2C %s\r\n", error_text);
        }
    }

    coines_rslt = coines_set_shuttleboard_vdd_vddio_config(1800, 1800);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting Vdd and Vddio to 1.8V %s\r\n", error_text);
    }

    pin_direction = COINES_PIN_DIRECTION_OUT;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting the reset pin %s\r\n", error_text);
    }

    /* Configure as a pull-down. The BHy operates the interrupt pin as an active high, level, push-pull by default */
    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_LOW;
    coines_rslt = coines_set_pin_config(int_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error configuring the interrupt pin %s\r\n", error_text);
    }

    coines_delay_msec(50);
}

void setup_interfaces_with_port(bool reset_power, enum bhi360_intf intf, const char *com_port)
{
    int16_t coines_rslt = COINES_SUCCESS;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;
    struct coines_board_info board_info;
    char *error_text;

#ifndef PC
    struct coines_ble_config ble_config;
    ble_config.name = NULL;
    ble_config.tx_power = COINES_TX_POWER_8_DBM;
    coines_rslt = coines_ble_config(&ble_config);
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);
#else
    struct coines_serial_com_config scom_config;
    scom_config.baud_rate = 9600;
    scom_config.vendor_id = ROBERT_BOSCH_USB_VID;
    scom_config.product_id = BST_APP30_CDC_USB_PID;
    scom_config.com_port_name = strdup(com_port);
    scom_config.rx_buffer_size = 2048;
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, &scom_config);
    free(scom_config.com_port_name); /* free memory */
#endif
    if (coines_rslt)
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\n", error_text);
    }

    coines_rslt = coines_get_board_info(&board_info);
    if (coines_rslt == COINES_SUCCESS)
    {
#ifdef PC
        if (board_info.board == 3) /* Application Board 2.0 */
        {
            scom_config.product_id = BST_APP20_CDC_USB_PID;
        }
        else if (board_info.board == 9) /* Application Board 3.1 */
        {
            scom_config.product_id = BST_APP31_CDC_USB_PID;
        }

#endif
        if (board_info.board == 5) /* Application Board 3.0 */
        {
            cs_pin = BHY360_APP30_CS_PIN;
            int_pin = BHY360_APP30_INT_PIN;
            reset_pin = BHY360_APP30_RESET_PIN;
        }
    }
    else
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\r\n", error_text);
    }

    if (reset_power)
    {
        coines_rslt = coines_set_shuttleboard_vdd_vddio_config(0, 0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\r\n", error_text);
        }

        pin_direction = COINES_PIN_DIRECTION_OUT;
        pin_value = COINES_PIN_VALUE_LOW;
        coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\r\n", error_text);
        }

        coines_delay_msec(10);
    }

    if (intf == BHI360_SPI_INTERFACE)
    {
        printf("Host Interface : SPI\r\n");
        coines_rslt = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_1_MHZ, COINES_SPI_MODE0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to SPI %s\r\n", error_text);
        }
    }
    else
    {
        printf("Host Interface : I2C\r\n");
        coines_rslt = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to I2C %s\r\n", error_text);
        }
    }

    coines_rslt = coines_set_shuttleboard_vdd_vddio_config(1800, 1800);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting Vdd and Vddio to 1.8V.\r\n%s\r\n", error_text);
    }

    pin_direction = COINES_PIN_DIRECTION_OUT;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting the reset pin %s\r\n", error_text);
    }

    /* Configure as a pull-down. The BHy operates the interrupt pin as an active high, level, push-pull by default */
    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_LOW;
    coines_rslt = coines_set_pin_config(int_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error configuring the interrupt pin %s\r\n", error_text);
    }

    coines_delay_msec(50);
}

void close_interfaces(enum bhi360_intf intf)
{
    if (intf == BHI360_I2C_INTERFACE)
    {
        (void)coines_deconfig_i2c_bus(COINES_I2C_BUS_0);
    }
    else
    {
        (void)coines_deconfig_spi_bus(COINES_SPI_BUS_0);
    }

    (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    (void)fflush(stdout);

    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);

    coines_delay_msec(100);

    /* Coines interface reset */
    coines_soft_reset();
    coines_delay_msec(100);
}

int8_t bhi360_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_read_spi(COINES_SPI_BUS_0, cs_pin, reg_addr, reg_data, (uint16_t)length);
}

int8_t bhi360_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_write_spi(COINES_SPI_BUS_0, cs_pin, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

int8_t bhi360_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_read_i2c(COINES_I2C_BUS_0, 0x28, reg_addr, reg_data, (uint16_t)length);
}

int8_t bhi360_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_write_i2c(COINES_I2C_BUS_0, 0x28, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

void bhi360_delay_us(uint32_t us, void *private_data)
{
    (void)private_data;
    coines_delay_usec(us);
}

static char *get_sensor_error_text_part_one(uint8_t sensor_error)
{
    char *ret = " ";

    switch (sensor_error)
    {
        case 0x00:
            break;
        case 0x10:
            ret = "[Sensor error] Bootloader reports: Firmware Expected Version Mismatch";
            break;
        case 0x11:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Header CRC";
            break;
        case 0x12:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: SHA Hash Mismatch";
            break;
        case 0x13:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Image CRC";
            break;
        case 0x14:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: ECDSA Signature Verification Failed";
            break;
        case 0x15:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Public Key CRC";
            break;
        case 0x16:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Signed Firmware Required";
            break;
        case 0x17:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: FW Header Missing";
            break;
        case 0x19:
            ret = "[Sensor error] Bootloader reports: Unexpected Watchdog Reset";
            break;
        case 0x1A:
            ret = "[Sensor error] ROM Version Mismatch";
            break;
        case 0x1B:
            ret = "[Sensor error] Bootloader reports: Fatal Firmware Error";
            break;
        case 0x1C:
            ret = "[Sensor error] Chained Firmware Error: Next Payload Not Found";
            break;
        case 0x1D:
            ret = "[Sensor error] Chained Firmware Error: Payload Not Valid";
            break;
        case 0x1E:
            ret = "[Sensor error] Chained Firmware Error: Payload Entries Invalid";
            break;
        case 0x1F:
            ret = "[Sensor error] Bootloader reports: Bootloader Error: OTP CRC Invalid";
            break;
        case 0x20:
            ret = "[Sensor error] Firmware Init Failed";
            break;
        case 0x21:
            ret = "[Sensor error] Sensor Init Failed: Unexpected Device ID";
            break;
        case 0x22:
            ret = "[Sensor error] Sensor Init Failed: No Response from Device";
            break;
    }

    return ret;
}

static char *get_sensor_error_text_part_two(uint8_t sensor_error)
{
    char *ret = " ";

    switch (sensor_error)
    {
        case 0x23:
            ret = "[Sensor error] Sensor Init Failed: Unknown";
            break;
        case 0x24:
            ret = "[Sensor error] Sensor Error: No Valid Data";
            break;
        case 0x25:
            ret = "[Sensor error] Slow Sample Rate";
            break;
        case 0x26:
            ret = "[Sensor error] Data Overflow (saturated sensor data)";
            break;
        case 0x27:
            ret = "[Sensor error] Stack Overflow";
            break;
        case 0x28:
            ret = "[Sensor error] Insufficient Free RAM";
            break;
        case 0x29:
            ret = "[Sensor error] Sensor Init Failed: Driver Parsing Error";
            break;
        case 0x2A:
            ret = "[Sensor error] Too Many RAM Banks Required";
            break;
        case 0x2B:
            ret = "[Sensor error] Invalid Event Specified";
            break;
        case 0x2C:
            ret = "[Sensor error] More than 32 On Change";
            break;
        case 0x2D:
            ret = "[Sensor error] Firmware Too Large";
            break;
        case 0x2F:
            ret = "[Sensor error] Invalid RAM Banks";
            break;
        case 0x30:
            ret = "[Sensor error] Math Error";
            break;
        case 0x40:
            ret = "[Sensor error] Memory Error";
            break;
        case 0x41:
            ret = "[Sensor error] SWI3 Error";
            break;
        case 0x42:
            ret = "[Sensor error] SWI4 Error";
            break;
        case 0x43:
            ret = "[Sensor error] Illegal Instruction Error";
            break;
        case 0x44:
            ret = "[Sensor error] Bootloader reports: Unhandled Interrupt Error / Exception / Postmortem Available";
            break;
        case 0x45:
            ret = "[Sensor error] Invalid Memory Access";
            break;
    }

    return ret;
}

static char *get_sensor_error_text_part_three(uint8_t sensor_error)
{
    char *ret = " ";

    switch (sensor_error)
    {
        case 0x50:
            ret = "[Sensor error] Algorithm Error: BSX Init";
            break;
        case 0x51:
            ret = "[Sensor error] Algorithm Error: BSX Do Step";
            break;
        case 0x52:
            ret = "[Sensor error] Algorithm Error: Update Sub";
            break;
        case 0x53:
            ret = "[Sensor error] Algorithm Error: Get Sub";
            break;
        case 0x54:
            ret = "[Sensor error] Algorithm Error: Get Phys";
            break;
        case 0x55:
            ret = "[Sensor error] Algorithm Error: Unsupported Phys Rate";
            break;
        case 0x56:
            ret = "[Sensor error] Algorithm Error: Cannot find BSX Driver";
            break;
        case 0x60:
            ret = "[Sensor error] Sensor Self-Test Failure";
            break;
        case 0x61:
            ret = "[Sensor error] Sensor Self-Test X Axis Failure";
            break;
        case 0x62:
            ret = "[Sensor error] Sensor Self-Test Y Axis Failure";
            break;
        case 0x64:
            ret = "[Sensor error] Sensor Self-Test Z Axis Failure";
            break;
        case 0x65:
            ret = "[Sensor error] FOC Failure";
            break;
        case 0x66:
            ret = "[Sensor error] Sensor Busy";
            break;
        case 0x6F:
            ret = "[Sensor error] Self-Test or FOC Test Unsupported";
            break;
        case 0x72:
            ret = "[Sensor error] No Host Interrupt Set";
            break;
        case 0x73:
            ret = "[Sensor error] Event ID Passed to Host Interface Has No Known Size";
            break;
        case 0x75:
            ret = "[Sensor error] Host Download Channel Underflow (Host Read Too Fast)";
            break;
    }

    return ret;
}

static char *get_sensor_error_text_part_four(uint8_t sensor_error)
{
    char *ret = " ";

    switch (sensor_error)
    {
        case 0x76:
            ret = "[Sensor error] Host Upload Channel Overflow (Host Wrote Too Fast)";
            break;
        case 0x77:
            ret = "[Sensor error] Host Download Channel Empty";
            break;
        case 0x78:
            ret = "[Sensor error] DMA Error";
            break;
        case 0x79:
            ret = "[Sensor error] Corrupted Input Block Chain";
            break;
        case 0x7A:
            ret = "[Sensor error] Corrupted Output Block Chain";
            break;
        case 0x7B:
            ret = "[Sensor error] Buffer Block Manager Error";
            break;
        case 0x7C:
            ret = "[Sensor error] Input Channel Not Word Aligned";
            break;
        case 0x7D:
            ret = "[Sensor error] Too Many Flush Events";
            break;
        case 0x7E:
            ret = "[Sensor error] Unknown Host Channel Error";
            break;
        case 0x81:
            ret = "[Sensor error] Decimation Too Large";
            break;
        case 0x90:
            ret = "[Sensor error] Master SPI/I2C Queue Overflow";
            break;
        case 0x91:
            ret = "[Sensor error] SPI/I2C Callback Error";
            break;
        case 0xA0:
            ret = "[Sensor error] Timer Scheduling Error";
            break;
        case 0xB0:
            ret = "[Sensor error] Invalid GPIO for Host IRQ";
            break;
        case 0xB1:
            ret = "[Sensor error] Error Sending Initialized Meta Events";
            break;
    }

    return ret;
}

static char *get_sensor_error_text_part_five(uint8_t sensor_error)
{
    char *ret = " ";

    switch (sensor_error)
    {
        case 0xC0:
            ret = "[Sensor error] Bootloader reports: Command Error";
            break;
        case 0xC1:
            ret = "[Sensor error] Bootloader reports: Command Too Long";
            break;
        case 0xC2:
            ret = "[Sensor error] Bootloader reports: Command Buffer Overflow";
            break;
        case 0xD0:
            ret = "[Sensor error] User Mode Error: Sys Call Invalid";
            break;
        case 0xD1:
            ret = "[Sensor error] User Mode Error: Trap Invalid";
            break;
        case 0xE1:
            ret = "[Sensor error] Firmware Upload Failed: Firmware header corrupt";
            break;
        case 0xE2:
            ret = "[Sensor error] Sensor Data Injection: Invalid input stream";
            break;
        default:
            ret = "[Sensor error] Unknown error code";
    }

    return ret;
}

char *get_sensor_error_text(uint8_t sensor_error)
{
    char *ret = " ";

    ret = get_sensor_error_text_part_one(sensor_error);

    if (strcmp(ret, " ") == 0)
    {
        ret = get_sensor_error_text_part_two(sensor_error);

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_error_text_part_three(sensor_error);

            if (strcmp(ret, " ") == 0)
            {
                ret = get_sensor_error_text_part_four(sensor_error);

                if (strcmp(ret, " ") == 0)
                {
                    ret = get_sensor_error_text_part_five(sensor_error);
                }
            }
        }
    }

    return ret;
}

static char *get_physical_sensor_name_extended(uint8_t sensor_id)
{
    char *ret = "";

    switch (sensor_id)
    {
        case BHI360_PHYS_SENSOR_ID_BME_TEMP:
            ret = "BME Temperature";
            break;
        case BHI360_PHYS_SENSOR_ID_FEATURE_CORE:
            ret = "Feature Core";
            break;
        case BHI360_PHYS_SENSOR_ID_GPS:
            ret = "GPS";
            break;
        case BHI360_PHYS_SENSOR_ID_LIGHT:
            ret = "Light";
            break;
        case BHI360_PHYS_SENSOR_ID_PROXIMITY:
            ret = "Proximity";
            break;
        case BHI360_PHYS_SENSOR_ID_ACT_REC:
            ret = "Activity Recognition";
            break;
        case BHI360_PHYS_SENSOR_ID_PHYS_NO_MOTION:
            ret = "No Motion";
            break;
        case BHI360_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT:
            ret = "Wrist Gesture Detector";
            break;
        case BHI360_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP:
            ret = "Wrist Wear Wakeup";
            break;
        default:
            ret = "Undefined sensor ID ";
    }

    return ret;
}

char *get_physical_sensor_name(uint8_t sensor_id)
{
    char *ret = "";

    switch (sensor_id)
    {
        case BHI360_PHYS_SENSOR_ID_ACCELEROMETER:
            ret = "Accelerometer";
            break;
        case BHI360_PHYS_SENSOR_ID_NOT_SUPPORTED:
            ret = "Not supported now";
            break;
        case BHI360_PHYS_SENSOR_ID_GYROSCOPE:
            ret = "Gyroscope";
            break;
        case BHI360_PHYS_SENSOR_ID_MAGNETOMETER:
            ret = "Magnetometer";
            break;
        case BHI360_PHYS_SENSOR_ID_TEMP_GYRO:
            ret = "Temperature Gyroscope";
            break;
        case BHI360_PHYS_SENSOR_ID_BMP_PRESSURE:
            ret = "BMP Pressure";
            break;
        case BHI360_PHYS_SENSOR_ID_BMP_TEMPERATURE:
            ret = "BMP Temperature";
            break;
        case BHI360_PHYS_SENSOR_ID_POSITION:
            ret = "Position";
            break;
        case BHI360_PHYS_SENSOR_ID_HUMIDITY:
            ret = "Humidity";
            break;
        case BHI360_PHYS_SENSOR_ID_TEMPERATURE:
            ret = "Temperature";
            break;
        case BHI360_PHYS_SENSOR_ID_GAS_RESISTOR:
            ret = "Gas Resistor";
            break;
        case BHI360_PHYS_SENSOR_ID_PHYS_STEP_COUNTER:
            ret = "Step Counter";
            break;
        case BHI360_PHYS_SENSOR_ID_PHYS_STEP_DETECTOR:
            ret = "Step Detector";
            break;
        case BHI360_PHYS_SENSOR_ID_PHYS_SIGN_MOTION:
            ret = "Significant Motion";
            break;
        case BHI360_PHYS_SENSOR_ID_PHYS_ANY_MOTION:
            ret = "Any Motion";
            break;
        default:
            ret = get_physical_sensor_name_extended(sensor_id);
    }

    return ret;
}

uint8_t get_physical_sensor_id(uint8_t virt_sensor_id)
{
    uint8_t ret = 0;

    if ((virt_sensor_id >= BHI360_SENSOR_ID_ACC_PASS && virt_sensor_id <= BHI360_SENSOR_ID_ACC_RAW_WU) ||
        (virt_sensor_id == BHI360_SENSOR_ID_ACC_BIAS_WU))
    {
        ret = BHI360_PHYS_SENSOR_ID_ACCELEROMETER;
    }
    else if ((virt_sensor_id >= BHI360_SENSOR_ID_GYRO_PASS && virt_sensor_id <= BHI360_SENSOR_ID_GYRO_RAW_WU) ||
             (virt_sensor_id == BHI360_SENSOR_ID_GYRO_BIAS_WU))
    {
        ret = BHI360_PHYS_SENSOR_ID_GYROSCOPE;
    }
    else if ((virt_sensor_id >= BHI360_SENSOR_ID_MAG_PASS && virt_sensor_id <= BHI360_SENSOR_ID_MAG_RAW_WU) ||
             (virt_sensor_id == BHI360_SENSOR_ID_MAG_BIAS_WU))
    {
        ret = BHI360_PHYS_SENSOR_ID_MAGNETOMETER;
    }

    return ret;
}

char *get_sensor_name_part_one(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_ACC_PASS:
            ret = "Accelerometer passthrough";
            break;
        case BHI360_SENSOR_ID_ACC_RAW:
            ret = "Accelerometer uncalibrated";
            break;
        case BHI360_SENSOR_ID_ACC:
            ret = "Accelerometer corrected";
            break;
        case BHI360_SENSOR_ID_ACC_BIAS:
            ret = "Accelerometer offset";
            break;
        case BHI360_SENSOR_ID_ACC_WU:
            ret = "Accelerometer corrected wake up";
            break;
        case BHI360_SENSOR_ID_ACC_RAW_WU:
            ret = "Accelerometer uncalibrated wake up";
            break;
        case BHI360_SENSOR_ID_GYRO_PASS:
            ret = "Gyroscope passthrough";
            break;
        case BHI360_SENSOR_ID_GYRO_RAW:
            ret = "Gyroscope uncalibrated";
            break;
        case BHI360_SENSOR_ID_GYRO:
            ret = "Gyroscope corrected";
            break;
        case BHI360_SENSOR_ID_GYRO_BIAS:
            ret = "Gyroscope offset";
            break;
        case BHI360_SENSOR_ID_GYRO_WU:
            ret = "Gyroscope wake up";
            break;
        case BHI360_SENSOR_ID_GYRO_RAW_WU:
            ret = "Gyroscope uncalibrated wake up";
            break;
        case BHI360_SENSOR_ID_MAG_PASS:
            ret = "Magnetometer passthrough";
            break;
        case BHI360_SENSOR_ID_MAG_RAW:
            ret = "Magnetometer uncalibrated";
            break;
        case BHI360_SENSOR_ID_MAG:
            ret = "Magnetometer corrected";
            break;
        case BHI360_SENSOR_ID_MAG_BIAS:
            ret = "Magnetometer offset";
            break;
        case BHI360_SENSOR_ID_MAG_WU:
            ret = "Magnetometer wake up";
            break;
        case BHI360_SENSOR_ID_MAG_RAW_WU:
            ret = "Magnetometer uncalibrated wake up";
            break;
        case BHI360_SENSOR_ID_GRA:
            ret = "Gravity vector";
            break;
    }

    return ret;
}

char *get_sensor_name_part_two(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_GRA_WU:
            ret = "Gravity vector wake up";
            break;
        case BHI360_SENSOR_ID_LACC:
            ret = "Linear acceleration";
            break;
        case BHI360_SENSOR_ID_LACC_WU:
            ret = "Linear acceleration wake up";
            break;
        case BHI360_SENSOR_ID_RV:
            ret = "Rotation vector";
            break;
        case BHI360_SENSOR_ID_RV_WU:
            ret = "Rotation vector wake up";
            break;
        case BHI360_SENSOR_ID_GAMERV:
            ret = "Game rotation vector";
            break;
        case BHI360_SENSOR_ID_GAMERV_WU:
            ret = "Game rotation vector wake up";
            break;
        case BHI360_SENSOR_ID_GEORV:
            ret = "Geo-magnetic rotation vector";
            break;
        case BHI360_SENSOR_ID_GEORV_WU:
            ret = "Geo-magnetic rotation vector wake up";
            break;
        case BHI360_SENSOR_ID_ORI:
            ret = "Orientation";
            break;
        case BHI360_SENSOR_ID_ORI_WU:
            ret = "Orientation wake up";
            break;
        case BHI360_SENSOR_ID_ACC_BIAS_WU:
            ret = "Accelerometer offset wake up";
            break;
        case BHI360_SENSOR_ID_GYRO_BIAS_WU:
            ret = "Gyroscope offset wake up";
            break;
        case BHI360_SENSOR_ID_MAG_BIAS_WU:
            ret = "Magnetometer offset wake up";
            break;
        case BHI360_SENSOR_ID_KLIO:
            ret = "Klio";
            break;
        case BHI360_SENSOR_ID_KLIO_LOG:
            ret = "Klio log";
            break;
        case BHI360_SENSOR_ID_TEMP:
            ret = "Temperature";
            break;
        case BHI360_SENSOR_ID_BARO:
            ret = "Barometer";
            break;
    }

    return ret;
}

char *get_sensor_name_part_three(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_HUM:
            ret = "Humidity";
            break;
        case BHI360_SENSOR_ID_GAS:
            ret = "Gas";
            break;
        case BHI360_SENSOR_ID_TEMP_WU:
            ret = "Temperature wake up";
            break;
        case BHI360_SENSOR_ID_BARO_WU:
            ret = "Barometer wake up";
            break;
        case BHI360_SENSOR_ID_HUM_WU:
            ret = "Humidity wake up";
            break;
        case BHI360_SENSOR_ID_GAS_WU:
            ret = "Gas wake up";
            break;
        case BHI360_SENSOR_ID_SI_ACCEL:
            ret = "SI Accel";
            break;
        case BHI360_SENSOR_ID_SI_GYROS:
            ret = "SI Gyro";
            break;
        case BHI360_SENSOR_ID_LIGHT:
            ret = "Light";
            break;
        case BHI360_SENSOR_ID_LIGHT_WU:
            ret = "Light wake up";
            break;
        case BHI360_SENSOR_ID_PROX:
            ret = "Proximity";
            break;
        case BHI360_SENSOR_ID_PROX_WU:
            ret = "Proximity wake up";
            break;
        case BHI360_SENSOR_ID_STC:
            ret = "Step counter";
            break;
        case BHI360_SENSOR_ID_STC_WU:
            ret = "Step counter wake up";
            break;
        case BHI360_SENSOR_ID_STC_LP:
            ret = "Low Power Step counter";
            break;
        case BHI360_SENSOR_ID_STC_LP_WU:
            ret = "Low Power Step counter wake up";
            break;
        case BHI360_SENSOR_ID_SIG:
            ret = "Significant motion";
            break;
        case BHI360_SENSOR_ID_STD:
            ret = "Step detector";
            break;
        case BHI360_SENSOR_ID_STD_WU:
            ret = "Step detector wake up";
            break;
    }

    return ret;
}

char *get_sensor_name_part_four(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_TILT_DETECTOR:
            ret = "Tilt detector";
            break;
        case BHI360_SENSOR_ID_WAKE_GESTURE:
            ret = "Wake gesture";
            break;
        case BHI360_SENSOR_ID_GLANCE_GESTURE:
            ret = "Glance gesture";
            break;
        case BHI360_SENSOR_ID_PICKUP_GESTURE:
            ret = "Pickup gesture";
            break;
        case BHI360_SENSOR_ID_SIG_LP_WU:
            ret = "Low Power Significant motion wake up";
            break;
        case BHI360_SENSOR_ID_STD_LP:
            ret = "Low Power Step detector";
            break;
        case BHI360_SENSOR_ID_STD_LP_WU:
            ret = "Low Power Step detector wake up";
            break;
        case BHI360_SENSOR_ID_AR:
            ret = "Activity recognition";
            break;
        case BHI360_SENSOR_ID_GPS:
            ret = "GPS";
            break;
        case BHI360_SENSOR_ID_WRIST_TILT_GESTURE:
            ret = "Wrist tilt gesture";
            break;
        case BHI360_SENSOR_ID_DEVICE_ORI:
            ret = "Device orientation";
            break;
        case BHI360_SENSOR_ID_DEVICE_ORI_WU:
            ret = "Device orientation wake up";
            break;
        case BHI360_SENSOR_ID_STATIONARY_DET:
            ret = "Stationary detect";
            break;
        case BHI360_SENSOR_ID_ANY_MOTION_LP_WU:
            ret = "Low Power Any motion wake up";
            break;
        case BHI360_SENSOR_ID_NO_MOTION_LP_WU:
            ret = "Low Power No Motion wake up";
            break;
        case BHI360_SENSOR_ID_MOTION_DET:
            ret = "Motion detect";
            break;
    }

    return ret;
}

char *get_sensor_name_part_five(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_BMP_TEMPERATURE:
            ret = "BMP Temperature ";
            break;
        case BHI360_SENSOR_ID_PRESSURE:
            ret = "BMP Pressure ";
            break;
        case BHI360_SENSOR_ID_AR_WEAR_WU:
            ret = "Activity recognition for Wearables";
            break;
        case BHI360_SENSOR_ID_WRIST_WEAR_LP_WU:
            ret = "Low Power Wrist Wear wake up";
            break;
        case BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            ret = "Low Power Wrist Gesture wake up";
            break;
        case BHI360_SENSOR_ID_MULTI_TAP:
            ret = "Multi Tap Detector";
            break;
        case BHI360_SENSOR_ID_AIR_QUALITY:
            ret = "Air Quality";
            break;
        case BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG:
            ret = "Head Misalignment Calibrator";
            break;
        case BHI360_SENSOR_ID_IMU_HEAD_ORI_Q:
            ret = "IMU Head Orientation Quaternion";
            break;
        case BHI360_SENSOR_ID_NDOF_HEAD_ORI_Q:
            ret = "NDOF Head Orientation Quaternion";
            break;
        case BHI360_SENSOR_ID_IMU_HEAD_ORI_E:
            ret = "IMU Head Orientation Euler";
            break;
        case BHI360_SENSOR_ID_NDOF_HEAD_ORI_E:
            ret = "NDOF Head Orientation Euler";
            break;
        case BHI360_SENSOR_ID_HEAD_GESTURE_DETECTOR:
            ret = "Head Gesture Detector";
            break;
        default:
            if ((sensor_id >= BHI360_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHI360_SENSOR_ID_CUSTOM_END))
            {
                ret = "Custom sensor ID ";
            }
            else
            {
                ret = "Undefined sensor ID ";
            }
    }

    return ret;
}
char *get_sensor_name(uint8_t sensor_id)
{
    char *ret = " ";

    ret = get_sensor_name_part_one(sensor_id);
    if (strcmp(ret, " ") == 0)
    {
        ret = get_sensor_name_part_two(sensor_id);

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_name_part_three(sensor_id);

            if (strcmp(ret, " ") == 0)
            {
                ret = get_sensor_name_part_four(sensor_id);

                if (strcmp(ret, " ") == 0)
                {
                    ret = get_sensor_name_part_five(sensor_id);
                }
            }
        }
    }

    return ret;
}

float get_sensor_dynamic_range_scaling(uint8_t sensor_id, float dynamic_range)
{
    float scaling = -1.0f;

    if ((sensor_id >= BHI360_SENSOR_ID_ACC_PASS && sensor_id <= BHI360_SENSOR_ID_ACC_RAW_WU) ||
        (sensor_id == BHI360_SENSOR_ID_ACC_BIAS_WU))
    {
        scaling = dynamic_range / 32768.0f;
    }
    else if ((sensor_id >= BHI360_SENSOR_ID_GYRO_PASS && sensor_id <= BHI360_SENSOR_ID_GYRO_RAW_WU) ||
             (sensor_id == BHI360_SENSOR_ID_GYRO_BIAS_WU))
    {
        scaling = dynamic_range / 32768.0f;
    }
    else if ((sensor_id >= BHI360_SENSOR_ID_MAG_PASS && sensor_id <= BHI360_SENSOR_ID_MAG_RAW_WU) ||
             (sensor_id == BHI360_SENSOR_ID_MAG_BIAS_WU))
    {
        scaling = dynamic_range / 32768.0f;
    }
    else
    {
        printf("Sensor ID not supported for dynamic range scaling\r\n");
        scaling = -1.0f; /* Do not apply the scaling factor */

    }

    return scaling;
}

char *get_sensor_si_unit(uint8_t sensor_id)
{
    char *ret;

    if ((sensor_id >= BHI360_SENSOR_ID_ACC_PASS && sensor_id <= BHI360_SENSOR_ID_ACC_RAW_WU) ||
        (sensor_id == BHI360_SENSOR_ID_ACC_BIAS_WU))
    {
        ret = "Earth g-s";
    }
    else if ((sensor_id >= BHI360_SENSOR_ID_GYRO_PASS && sensor_id <= BHI360_SENSOR_ID_GYRO_RAW_WU) ||
             (sensor_id == BHI360_SENSOR_ID_GYRO_BIAS_WU))
    {
        ret = "degrees/second";
    }
    else if ((sensor_id >= BHI360_SENSOR_ID_MAG_PASS && sensor_id <= BHI360_SENSOR_ID_MAG_RAW_WU) ||
             (sensor_id == BHI360_SENSOR_ID_MAG_BIAS_WU))
    {
        ret = "microtesla";
    }
    else
    {
        ret = "";
    }

    return ret;
}

static char *get_sensor_parse_format_part_one(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_ACC_PASS:
        case BHI360_SENSOR_ID_ACC_RAW:
        case BHI360_SENSOR_ID_ACC:
        case BHI360_SENSOR_ID_ACC_BIAS:
        case BHI360_SENSOR_ID_ACC_BIAS_WU:
        case BHI360_SENSOR_ID_ACC_WU:
        case BHI360_SENSOR_ID_ACC_RAW_WU:
        case BHI360_SENSOR_ID_GYRO_PASS:
        case BHI360_SENSOR_ID_GYRO_RAW:
        case BHI360_SENSOR_ID_GYRO:
        case BHI360_SENSOR_ID_GYRO_BIAS:
        case BHI360_SENSOR_ID_GYRO_BIAS_WU:
        case BHI360_SENSOR_ID_GYRO_WU:
        case BHI360_SENSOR_ID_GYRO_RAW_WU:
        case BHI360_SENSOR_ID_MAG_PASS:
        case BHI360_SENSOR_ID_MAG_RAW:
        case BHI360_SENSOR_ID_MAG:
        case BHI360_SENSOR_ID_MAG_BIAS:
            ret = "s16,s16,s16";
            break;
    }

    return ret;
}

static char *get_sensor_parse_format_part_two(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_MAG_BIAS_WU:
        case BHI360_SENSOR_ID_MAG_WU:
        case BHI360_SENSOR_ID_MAG_RAW_WU:
        case BHI360_SENSOR_ID_GRA:
        case BHI360_SENSOR_ID_GRA_WU:
        case BHI360_SENSOR_ID_LACC:
        case BHI360_SENSOR_ID_LACC_WU:
            ret = "s16,s16,s16";
            break;
        case BHI360_SENSOR_ID_RV:
        case BHI360_SENSOR_ID_RV_WU:
        case BHI360_SENSOR_ID_GAMERV:
        case BHI360_SENSOR_ID_GAMERV_WU:
        case BHI360_SENSOR_ID_GEORV:
        case BHI360_SENSOR_ID_GEORV_WU:
            ret = "s16,s16,s16,s16,u16";
            break;
        case BHI360_SENSOR_ID_ORI:
        case BHI360_SENSOR_ID_ORI_WU:
            ret = "s16,s16,s16";
            break;
        case BHI360_SENSOR_ID_KLIO:
            ret = "u8,s8,u8,u8,u8,u8,f,f";
            break;
    }

    return ret;
}

static char *get_sensor_parse_format_part_three(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_DEVICE_ORI:
        case BHI360_SENSOR_ID_DEVICE_ORI_WU:
        case BHI360_SENSOR_ID_HUM:
        case BHI360_SENSOR_ID_HUM_WU:
        case BHI360_SENSOR_ID_PROX:
        case BHI360_SENSOR_ID_PROX_WU:
        case BHI360_SENSOR_ID_MULTI_TAP:
            ret = "u8";
            break;
        case BHI360_SENSOR_ID_TEMP:
        case BHI360_SENSOR_ID_TEMP_WU:
            ret = "s16";
            break;
        case BHI360_SENSOR_ID_BARO:
        case BHI360_SENSOR_ID_BARO_WU:
            ret = "u24";
            break;
        case BHI360_SENSOR_ID_SI_ACCEL:
        case BHI360_SENSOR_ID_SI_GYROS:
            ret = "f,f,f";
            break;
    }

    return ret;
}

static char *get_sensor_parse_format_part_four(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_GAS:
        case BHI360_SENSOR_ID_GAS_WU:
        case BHI360_SENSOR_ID_STC:
        case BHI360_SENSOR_ID_STC_WU:
        case BHI360_SENSOR_ID_STC_LP:
        case BHI360_SENSOR_ID_STC_LP_WU:
            ret = "u32";
            break;
        case BHI360_SENSOR_ID_LIGHT:
        case BHI360_SENSOR_ID_LIGHT_WU:
            ret = "s16";
            break;
        case BHI360_SENSOR_ID_AR:
        case BHI360_SENSOR_ID_AR_WEAR_WU:
            ret = "u16";
            break;
        case BHI360_SENSOR_ID_GPS:
            ret = "st";
            break;
        case BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            ret = "u8";
            break;
        case BHI360_SENSOR_ID_AIR_QUALITY:
            ret = "f32,f32,f32,f32,f32,f32,f32,u8";
            break;
        case BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG:
        case BHI360_SENSOR_ID_IMU_HEAD_ORI_Q:
        case BHI360_SENSOR_ID_NDOF_HEAD_ORI_Q:
            ret = "s16,s16,s16,s16";
            break;
    }

    return ret;
}

static char *get_sensor_parse_format_part_five(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_SIG:
        case BHI360_SENSOR_ID_STD:
        case BHI360_SENSOR_ID_STD_WU:
        case BHI360_SENSOR_ID_TILT_DETECTOR:
        case BHI360_SENSOR_ID_WAKE_GESTURE:
        case BHI360_SENSOR_ID_GLANCE_GESTURE:
        case BHI360_SENSOR_ID_PICKUP_GESTURE:
        case BHI360_SENSOR_ID_SIG_LP_WU:
        case BHI360_SENSOR_ID_STD_LP:
        case BHI360_SENSOR_ID_STD_LP_WU:
        case BHI360_SENSOR_ID_WRIST_TILT_GESTURE:
        case BHI360_SENSOR_ID_STATIONARY_DET:
        case BHI360_SENSOR_ID_ANY_MOTION_LP_WU:
        case BHI360_SENSOR_ID_NO_MOTION_LP_WU:
        case BHI360_SENSOR_ID_MOTION_DET:
        case BHI360_SENSOR_ID_WRIST_WEAR_LP_WU:
            ret = "";
            break;

        case BHI360_SENSOR_ID_IMU_HEAD_ORI_E:
        case BHI360_SENSOR_ID_NDOF_HEAD_ORI_E:
            ret = "s16,s16,s16";
            break;
        default:
            ret = "";
    }

    return ret;
}

char *get_sensor_parse_format(uint8_t sensor_id)
{
    char *ret;

    ret = get_sensor_parse_format_part_one(sensor_id);
    if (strcmp(ret, " ") == 0)
    {
        ret = get_sensor_parse_format_part_two(sensor_id);

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_parse_format_part_three(sensor_id);

            if (strcmp(ret, " ") == 0)
            {
                ret = get_sensor_parse_format_part_four(sensor_id);

                if (strcmp(ret, " ") == 0)
                {
                    ret = get_sensor_parse_format_part_five(sensor_id);
                }
            }
        }
    }

    return ret;
}

static char *get_sensor_axis_names_part_one(uint8_t sensor_id)
{
    char *ret = "";

    if (((sensor_id >= BHI360_SENSOR_ID_ACC_PASS) && (sensor_id <= BHI360_SENSOR_ID_LACC_WU)) ||
        (sensor_id == BHI360_SENSOR_ID_ACC_BIAS_WU) || (sensor_id == BHI360_SENSOR_ID_GYRO_BIAS_WU) ||
        (sensor_id == BHI360_SENSOR_ID_MAG_BIAS_WU))
    {
        ret = "x,y,z";
    }

    return ret;
}

static char *get_sensor_axis_names_part_two(uint8_t sensor_id)
{
    char *ret = "";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_RV:
        case BHI360_SENSOR_ID_RV_WU:
        case BHI360_SENSOR_ID_GAMERV:
        case BHI360_SENSOR_ID_GAMERV_WU:
        case BHI360_SENSOR_ID_GEORV:
        case BHI360_SENSOR_ID_GEORV_WU:
            ret = "x,y,z,w,ar";
            break;
        case BHI360_SENSOR_ID_ORI:
        case BHI360_SENSOR_ID_ORI_WU:
            ret = "h,p,r";
            break;
        case BHI360_SENSOR_ID_DEVICE_ORI:
        case BHI360_SENSOR_ID_DEVICE_ORI_WU:
            ret = "o";
            break;
        case BHI360_SENSOR_ID_KLIO:
            ret = "lin,lid,lpr,lcr,rin,rid,rc,rsc";
            break;
        case BHI360_SENSOR_ID_TEMP:
        case BHI360_SENSOR_ID_TEMP_WU:
            ret = "t";
            break;
    }

    return ret;
}

static char *get_sensor_axis_names_part_three(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_BARO:
        case BHI360_SENSOR_ID_BARO_WU:
            ret = "p";
            break;
        case BHI360_SENSOR_ID_HUM:
        case BHI360_SENSOR_ID_HUM_WU:
            ret = "h";
            break;
        case BHI360_SENSOR_ID_GAS:
        case BHI360_SENSOR_ID_GAS_WU:
            ret = "g";
            break;
        case BHI360_SENSOR_ID_LIGHT:
        case BHI360_SENSOR_ID_LIGHT_WU:
            ret = "l";
            break;
        case BHI360_SENSOR_ID_PROX:
        case BHI360_SENSOR_ID_PROX_WU:
            ret = "p";
            break;
        case BHI360_SENSOR_ID_STC:
        case BHI360_SENSOR_ID_STC_WU:
        case BHI360_SENSOR_ID_STC_LP:
        case BHI360_SENSOR_ID_STC_LP_WU:
            ret = "c";
            break;
    }

    return ret;
}

char *get_sensor_axis_names_part_four(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_SIG:
        case BHI360_SENSOR_ID_STD:
        case BHI360_SENSOR_ID_STD_WU:
        case BHI360_SENSOR_ID_TILT_DETECTOR:
        case BHI360_SENSOR_ID_WAKE_GESTURE:
        case BHI360_SENSOR_ID_GLANCE_GESTURE:
        case BHI360_SENSOR_ID_PICKUP_GESTURE:
        case BHI360_SENSOR_ID_SIG_LP_WU:
        case BHI360_SENSOR_ID_STD_LP:
        case BHI360_SENSOR_ID_STD_LP_WU:
        case BHI360_SENSOR_ID_WRIST_TILT_GESTURE:
        case BHI360_SENSOR_ID_STATIONARY_DET:
        case BHI360_SENSOR_ID_ANY_MOTION_LP_WU:
        case BHI360_SENSOR_ID_NO_MOTION_LP_WU:
        case BHI360_SENSOR_ID_MOTION_DET:
        case BHI360_SENSOR_ID_WRIST_WEAR_LP_WU:
            ret = "e";
            break;
    }

    return ret;
}

char *get_sensor_axis_names_part_five(uint8_t sensor_id)
{
    char *ret = " ";

    switch (sensor_id)
    {
        case BHI360_SENSOR_ID_AR:
        case BHI360_SENSOR_ID_AR_WEAR_WU:
            ret = "a";
            break;
        case BHI360_SENSOR_ID_GPS:
            ret = "g";
            break;
        case BHI360_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            ret = "wrist_gesture";
            break;
        case BHI360_SENSOR_ID_MULTI_TAP:
            ret = "taps";
            break;
        case BHI360_SENSOR_ID_AIR_QUALITY:
            ret = "t,h,g,i,si,c,v,a";
            break;
        case BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG:
        case BHI360_SENSOR_ID_IMU_HEAD_ORI_Q:
        case BHI360_SENSOR_ID_NDOF_HEAD_ORI_Q:
            ret = "x,y,z,w";
            break;
        case BHI360_SENSOR_ID_IMU_HEAD_ORI_E:
        case BHI360_SENSOR_ID_NDOF_HEAD_ORI_E:
            ret = "h,p,r";
            break;
        default:
            ret = "";
    }

    return ret;
}

char *get_sensor_axis_names(uint8_t sensor_id)
{
    char *ret;

    ;

    ret = get_sensor_axis_names_part_one(sensor_id);

    if (strcmp(ret, " ") == 0)
    {
        ret = get_sensor_axis_names_part_two(sensor_id);

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_axis_names_part_three(sensor_id);
        }

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_axis_names_part_four(sensor_id);
        }

        if (strcmp(ret, " ") == 0)
        {
            ret = get_sensor_axis_names_part_five(sensor_id);
        }
    }

    return ret;
}

char *get_klio_error(bhi360_klio_param_driver_error_state_t error)
{
    char *ret = "";

    switch (error)
    {
        case KLIO_DRIVER_ERROR_NONE:
            break;
        case KLIO_DRIVER_ERROR_INVALID_PARAMETER:
            ret = "[Klio error] Invalid parameter";
            break;
        case KLIO_DRIVER_ERROR_PARAMETER_OUT_OF_RANGE:
            ret = "[Klio error] Parameter out of range";
            break;
        case KLIO_DRIVER_ERROR_INVALID_PATTERN_OPERATION:
            ret = "[Klio error] Invalid pattern operation";
            break;
        case KLIO_DRIVER_ERROR_NOT_IMPLEMENTED:
            ret = "[Klio error] Not implemented";
            break;
        case KLIO_DRIVER_ERROR_BUFSIZE:
            ret = "[Klio error] Buffer size";
            break;
        case KLIO_DRIVER_ERROR_INTERNAL:
            ret = "[Klio error] Internal";
            break;
        case KLIO_DRIVER_ERROR_UNDEFINED:
            ret = "[Klio error] Undefined";
            break;
        case KLIO_DRIVER_ERROR_OPERATION_PENDING:
            ret = "[Klio error] Operation pending";
            break;
        default:
            ret = "[Klio error] Unknown error code";
    }

    return ret;
}

#ifndef PC
void default_verbose_write(uint8_t *buffer, uint16_t length)
{
    coines_write_intf(COINES_COMM_INTF_USB, buffer, length);
}

void verbose_write(uint8_t *buffer, uint16_t length) __attribute__ ((weak, alias("default_verbose_write")));

#endif

void print_api_error(int8_t rslt, struct bhi360_dev *dev)
{
    if (rslt != BHI360_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHI360_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHI360_INTF_RET_SUCCESS;
        }
    }
}

void upload_firmware(const uint8_t fw[], uint32_t length, struct bhi360_dev *dev)
{
    uint8_t sensor_error = 0;
    int8_t temp_rslt;
    int8_t rslt = BHI360_OK;

    printf("Loading firmware into RAM.\r\n");
    rslt = bhi360_upload_firmware_to_ram(fw, length, dev);

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

void setup_host_int_ctrl(struct bhi360_dev *dev)
{
    int8_t rslt;
    uint8_t hintr_ctrl, hif_ctrl;

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI360_ICTL_DISABLE_STATUS_FIFO | BHI360_ICTL_DISABLE_DEBUG;

    rslt = bhi360_set_host_interrupt_ctrl(hintr_ctrl, dev);
    print_api_error(rslt, dev);
    rslt = bhi360_get_host_interrupt_ctrl(&hintr_ctrl, dev);
    print_api_error(rslt, dev);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHI360_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHI360_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHI360_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi360_set_host_intf_ctrl(hif_ctrl, dev);
    print_api_error(rslt, dev);
}

void init_sensor(struct bhi360_dev *dev, enum bhi360_intf intf)
{
    uint8_t chip_id = 0;
    int8_t rslt;

#ifdef BHI360_USE_I2C
    rslt = bhi360_init(intf, bhi360_i2c_read, bhi360_i2c_write, bhi360_delay_us, BHI360_RD_WR_LEN, NULL, dev);
#else
    rslt = bhi360_init(intf, bhi360_spi_read, bhi360_spi_write, bhi360_delay_us, BHI360_RD_WR_LEN, NULL, dev);
#endif

    print_api_error(rslt, dev);

    rslt = bhi360_soft_reset(dev);
    print_api_error(rslt, dev);

    rslt = bhi360_get_chip_id(&chip_id, dev);
    print_api_error(rslt, dev);

    /* Check for a valid Chip ID */
    if (chip_id == BHI360_CHIP_ID)
    {
        printf("Chip ID read 0x%X\r\n", chip_id);
    }
    else
    {
        printf("Device not found. Chip ID read 0x%X\r\n", chip_id);

        return;
    }
}