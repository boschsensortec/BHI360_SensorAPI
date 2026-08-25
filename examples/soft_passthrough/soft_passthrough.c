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
 * @file    soft_passthrough.c
 * @brief   Example for soft pass-through
 *
 */

#include <stdio.h>

#include "common.h"
#include "bhi360_parse.h"
#include "bhi360/Bosch_Shuttle3_BHI360_BMM350_BMP58X_BME688_bsxsam_ndof.fw.h"

#define MAX_EXECUTE_TIME                UINT8_C(50)

/*! BMM350 Macro Definitions */
#define BMM350_I2C_SLAVE_ADDR           UINT8_C(0x10)
#define BMM350_CHIP_ID_REG              UINT8_C(0x40)
#define BMM350_POWER_CONTROL_REG        UINT8_C(0x4B)
#define BMM350_POWER_CONTROL_REG_VALUE  UINT8_C(0x81)
#define BMM350_CHIP_ID                  UINT8_C(0x32)

/*! BME688 Macro Definitions */
#define BME688_I2C_SLAVE_ADDR           UINT8_C(0x77)
#define BME688_CHIP_ID_REG              UINT8_C(0xD0)
#define BME688_RESET_REG                UINT8_C(0xE0)
#define BME688_RESET_REG_VALUE          UINT8_C(0xB6)
#define BME688_CHIP_ID                  UINT8_C(0x61)

/*! BMI270 Macro Definitions */
#define BMI270_ACC_CONF_REG             UINT8_C(0x40)
#define BMI270_ACC_RANGE_16G            UINT8_C(0x02) /*in BHI360, BMI270 updated Acc Range, also the Register Value */

#define CHIP_ID_READ_ON_POR             UINT8_C(0x00)
#define MAX_PASSTHROUGH_TRANSFER_BYTE   UINT8_C(244)
#define SPI_TRANSFER_RATE               UINT16_C(0xE803)
#define SPI_M1_CS_PIN                   UINT8_C(25)
#define SCALING_FACTOR_INVALID_LIMIT    -1.0f

/*! @brief Soft Pass-Through function to read from register via I2C.
 *
 *  @param[in] sif_bus        : soft-passthrough interface
 *  @param[in] dev_addr   : Device address.
 *  @param[in] reg_addr   : Register address to be read from.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] length     : Length of the data buffer.
 *  @param[in] dev        : Device reference.
 */
static void spt_i2c_read(uint8_t sif_bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t *reg_data,
                         uint8_t length,
                         struct bhi360_dev *dev);

/*! @brief Soft Pass-Through function to write to register via I2C.
 *
 *  @param[in] sif_bus        : soft-passthrough interface
 *  @param[in] dev_addr   : Device address.
 *  @param[in] reg_addr   : Register address to write to.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] length     : Length of the data buffer.
 *  @param[in] dev        : Device reference.
 */
static void spt_i2c_write(uint8_t sif_bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t *reg_data,
                          uint8_t length,
                          struct bhi360_dev *dev);

/*! @brief Parse accel data.
 *
 *  @param[in] callback_info : sensor data info.
 *  @param[in] callback_ref  : Parse reference.
 */
static void parse_accel(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref);

/*! @brief Soft Pass-Through function to read from register via SPI.
 *
 *  @param[in] dev        : Device reference.
 *  @param[in] reg_addr   : Register address.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] read_cnt   : Length of the data to be read.
 */
static void spt_spi0_read(uint8_t reg_addr, uint8_t* reg_data, uint8_t read_cnt, struct bhi360_dev *dev);

/*! @brief Soft Pass-Through function to write to register via SPI.
 *
 *  @param[in] dev        : Device reference.
 *  @param[in] reg_addr   : Register address to write to.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] write_cnt  : Length of the data to be written.
 */
static void spt_spi0_write(uint8_t reg_addr, uint8_t* reg_data, uint8_t write_cnt, struct bhi360_dev *dev);

int main(void)
{
    enum bhi360_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t reg_data[16];
    uint8_t boot_status = 0;
    uint8_t data_length = 0;
    uint8_t write_data[16] = { 0 };
    uint8_t accuracy; /* Accuracy is reported as a meta event. It is being printed alongside the data */
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    uint8_t loop = 0U;

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
                                                   bhi360_parse_meta_event,
                                                   (void*)&accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SYS_ID_META_EVENT_WU,
                                                   bhi360_parse_meta_event,
                                                   (void*)&accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi360_register_fifo_parse_callback(BHI360_SENSOR_ID_ACC_WU, parse_accel, (void*)&accuracy, &bhy);
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

    /* Attempt to read, M1 SPI interface to access BMI270 */
    printf("BMI270 Accelerometer Configuration Register Read\r\n");
    data_length = 8;
    spt_spi0_read(BMI270_ACC_CONF_REG, reg_data, data_length, &bhy);

    data_length = 2;
    write_data[0] = reg_data[1];
    write_data[1] = BMI270_ACC_RANGE_16G; /* set accelerometer g-range to 16g */
    spt_spi0_write(BMI270_ACC_CONF_REG, write_data, data_length, &bhy);

    data_length = 8;
    spt_spi0_read(BMI270_ACC_CONF_REG, reg_data, data_length, &bhy);

    if (reg_data[2] == BMI270_ACC_RANGE_16G)
    {
        printf("BMI270 Accelerometer g-range set at 16G\r\n");
    }

    /* Clear the buffer */
    memset(reg_data, 0, sizeof(reg_data));
    memset(write_data, 0, sizeof(write_data));

    /* Attempt to read out the chip ID from the BMM350 connected over SIF2 of the BHI360 - on power on reset */
    data_length = 1;
    spt_i2c_read(BHI360_SPASS_SIF2, BMM350_I2C_SLAVE_ADDR, BMM350_CHIP_ID_REG, reg_data, data_length, &bhy);
    if (reg_data[0] == CHIP_ID_READ_ON_POR)
    {
        printf("BMM350 Chip ID read returns 0x00: In suspend mode\r\n"); /* Set the power bit to 1 in power control
                                                                          * register (0x4B), to put the device from
                                                                          * suspend mode to sleep mode. In this state
                                                                          * Chip ID can be read */
    }

    /* Attempt to set the power bit and soft reset trigger bit to 1 in power control register (0x4B); bit7： Soft reset,
     * bit0: power control bit */
    write_data[0] = BMM350_POWER_CONTROL_REG_VALUE;
    data_length = 1;
    printf("BMM350 Power Control Register Write\r\n");
    spt_i2c_write(BHI360_SPASS_SIF2, BMM350_I2C_SLAVE_ADDR, BMM350_POWER_CONTROL_REG, write_data, data_length, &bhy);

    /* Attempt to read out the BMM350 chip ID after writing power control register */
    spt_i2c_read(BHI360_SPASS_SIF2, BMM350_I2C_SLAVE_ADDR, BMM350_CHIP_ID_REG, reg_data, data_length, &bhy);
    if (reg_data[0] == BMM350_CHIP_ID)
    {
        printf("BMM150 Chip ID read returns: %X In sleep mode\r\n", reg_data[0]);
    }

    /* Attempt to reset BME688 from unknown status, connected over SIF3 of the BHI360 */
    printf("BME688 Reset Register Write\r\n");
    write_data[0] = BME688_RESET_REG_VALUE;
    data_length = 1;
    spt_i2c_write(BHI360_SPASS_SIF3, BME688_I2C_SLAVE_ADDR, BME688_RESET_REG, write_data, data_length, &bhy);

    /* Attempt to read out BME688 chip ID after reset */
    spt_i2c_read(BHI360_SPASS_SIF3, BME688_I2C_SLAVE_ADDR, BME688_CHIP_ID_REG, reg_data, data_length, &bhy);
    if (reg_data[0] == BME688_CHIP_ID)
    {
        printf("BME688 Chip ID read returns: %X\r\n", reg_data[0]);
    }

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi360_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    struct bhi360_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    sensor_conf.sample_rate = BHI360_ODR_100_HZ; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi360_virtual_sensor_conf_param_set_cfg(BHI360_SENSOR_ID_ACC_WU, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI360_SENSOR_ID_ACC_WU), sensor_conf.sample_rate);
    printf("Data Acquisition:\r\n");

    while (rslt == BHI360_OK && loop < MAX_EXECUTE_TIME)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi360_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            loop++;
            print_api_error(rslt, &bhy);
        }
    }

    close_interfaces(intf);

    return rslt;
}

static void parse_accel(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_xyz data;
    float scaling_factor = SCALING_FACTOR_INVALID_LIMIT;

    uint32_t s, ns;
    uint64_t timestamp;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    bhi360_event_data_parse_xyz(callback_info->data_ptr, &data);

    scaling_factor = (float)BHI360_ACCEL_16G / 32768.0f; /* since the sensor data will be scaled to signed 16bits */

    timestamp = *callback_info->time_stamp; /* Store the last timestamp */
    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    #ifndef PC
    printf("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f; ; unit: LSB/G\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x * scaling_factor,
           data.y * scaling_factor,
           data.z * scaling_factor);
#else
    printf("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f; ; unit: LSB/G\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x * scaling_factor,
           data.y * scaling_factor,
           data.z * scaling_factor);
#endif
}

static void spt_spi0_read(uint8_t reg_addr, uint8_t* reg_data, uint8_t read_cnt, struct bhi360_dev *dev)
{
    uint8_t rslt = BHI360_OK;
    uint8_t transfer_bytes = 0;
    union bhi360_soft_passthrough_conf conf;

    /* transfer bytes */
    if (read_cnt > MAX_PASSTHROUGH_TRANSFER_BYTE)
    {
        transfer_bytes = MAX_PASSTHROUGH_TRANSFER_BYTE;
    }
    else
    {
        transfer_bytes = read_cnt;
    }

    conf.conf.direction = BHI360_SPASS_READ;
    conf.conf.trans_type = BHI360_SPASS_SINGLE_TRANS;
    conf.conf.delay_ctrl = BHI360_SPASS_DELAY_DIS;
    conf.conf.master_bus = BHI360_SPASS_SIF1;

    conf.conf.spi_mode = BHI360_SPASS_SPI_4_WIRE;
    conf.conf.cpol = BHI360_SPASS_SPI_CPOL_1;
    conf.conf.cpha = BHI360_SPASS_SPI_CPHA_1;
    conf.conf.cs_level = BHI360_SPASS_SPI_CS_LOW;
    conf.conf.lsb_first = BHI360_SPASS_SPI_LSB_FIRST_DIS;
    conf.conf.trans_rate = SPI_TRANSFER_RATE; /*in kHz, 16-bit unsigned, LSB first 0x03E8 = 1MHz */
    conf.conf.address_shift = 0;
    conf.conf.read_bit_pol = 1;
    conf.conf.read_bit_pos = BHI360_SPASS_SPI_READ_BIT_POS_7;
    conf.conf.func_set.cs_pin = SPI_M1_CS_PIN;

    conf.conf.trans_count = transfer_bytes + 1;
    conf.conf.reg = reg_addr;

    /* BMI270 Reg_data[0] is dummy value, reg_data[1] is register value */
    rslt = bhi360_soft_passthrough_transfer(&conf, conf.conf.reg, conf.conf.trans_count, reg_data, dev);

    print_api_error(rslt, dev);
}

static void spt_spi0_write(uint8_t reg_addr, uint8_t* reg_data, uint8_t write_cnt, struct bhi360_dev *dev)
{
    uint8_t rslt = BHI360_OK;
    uint8_t transfer_bytes = 0;
    union bhi360_soft_passthrough_conf conf;

    /* transfer bytes */
    if (write_cnt > MAX_PASSTHROUGH_TRANSFER_BYTE)
    {
        transfer_bytes = MAX_PASSTHROUGH_TRANSFER_BYTE;
    }
    else
    {
        transfer_bytes = write_cnt;
    }

    conf.conf.direction = BHI360_SPASS_WRITE;
    conf.conf.trans_type = BHI360_SPASS_MULTI_TRANS; /* multiple bytes to write */
    conf.conf.delay_ctrl = BHI360_SPASS_DELAY_DIS;
    conf.conf.master_bus = BHI360_SPASS_SIF1;

    conf.conf.spi_mode = BHI360_SPASS_SPI_4_WIRE;
    conf.conf.cpol = BHI360_SPASS_SPI_CPOL_1;
    conf.conf.cpha = BHI360_SPASS_SPI_CPHA_1;
    conf.conf.cs_level = BHI360_SPASS_SPI_CS_LOW;
    conf.conf.lsb_first = BHI360_SPASS_SPI_LSB_FIRST_DIS;
    conf.conf.trans_rate = SPI_TRANSFER_RATE; /*in kHz, 16-bit unsigned, LSB first 0x03E8 = 1MHz */
    conf.conf.address_shift = 0;
    conf.conf.read_bit_pol = 1;
    conf.conf.read_bit_pos = BHI360_SPASS_SPI_READ_BIT_POS_7;
    conf.conf.func_set.cs_pin = SPI_M1_CS_PIN;

    conf.conf.trans_count = transfer_bytes;
    conf.conf.reg = reg_addr;

    rslt = bhi360_soft_passthrough_transfer(&conf, conf.conf.reg, conf.conf.trans_count, reg_data, dev);
    print_api_error(rslt, dev);
}

static void spt_i2c_read(uint8_t sif_bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t *reg_data,
                         uint8_t length,
                         struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    union bhi360_soft_passthrough_conf spt_conf = { { 0 } };

    spt_conf.conf.direction = BHI360_SPASS_READ;
    spt_conf.conf.trans_type = BHI360_SPASS_SINGLE_TRANS;
    spt_conf.conf.master_bus = sif_bus;
    spt_conf.conf.func_set.slave_address = dev_addr;

    rslt = bhi360_soft_passthrough_transfer(&spt_conf, reg_addr, length, reg_data, dev);
    print_api_error(rslt, dev);
}

static void spt_i2c_write(uint8_t sif_bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t *reg_data,
                          uint8_t length,
                          struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    union bhi360_soft_passthrough_conf spt_conf = { { 0 } };

    spt_conf.conf.direction = BHI360_SPASS_WRITE;
    spt_conf.conf.trans_type = BHI360_SPASS_SINGLE_TRANS;
    spt_conf.conf.master_bus = sif_bus;
    spt_conf.conf.func_set.slave_address = dev_addr;

    rslt = bhi360_soft_passthrough_transfer(&spt_conf, reg_addr, length, reg_data, dev);
    print_api_error(rslt, dev);
}