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
* @file       bhy.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360.h"
#include "bhi360_hif.h"
#include "bhi360_defs.h"
#include "bhi360_system_param.h"
#include "bhi360_virtual_sensor_info_param.h"

#define CMD_FIFO_DELAY_TIMEOUT_US  (10000U)
#define VARI_DATA_BUF_SIZE         (118U)

typedef enum
{
    BHI360_BUFFER_STATUS_OK,
    BHI360_BUFFER_STATUS_RELOAD
} buffer_status_t;

/*
 * Array base address "0" corresponds to virtual sensor id 245
 * Debug Data: 250 : event size : 18
 * 245 & 251 : Timestamp small Delta : event size : 2
 * 252 & 246 : Timestamp Large Delta : event size : 3
 * 253 & 247 : Full timestamp: event size: 6
 * 254 & 248 : Meta event: event size: 4
 * Filler: 255 : event size : 1
 */
static const uint8_t bhi360_sysid_event_size[11] = { 2, 3, 6, 4, 0, 18, 2, 3, 6, 4, 1 };

static int8_t parse_fifo(enum bhi360_fifo_type source, struct bhi360_fifo_buffer *fifo_p, struct bhi360_dev *dev);
static int8_t get_buffer_status(const struct bhi360_fifo_buffer *fifo_p, uint8_t event_size, buffer_status_t *status);
static int8_t get_time_stamp(enum bhi360_fifo_type source, uint64_t **time_stamp, struct bhi360_dev *dev);
static int8_t get_callback_info(uint8_t sensor_id, struct bhi360_fifo_parse_callback_table *info,
                                struct bhi360_dev *dev);
static int8_t parse_status_fifo(struct bhi360_fifo_buffer *fifo_p, struct bhi360_dev *dev);
static int8_t process_status_fifo(int8_t rslt,
                                  uint16_t int_status,
                                  struct bhi360_fifo_buffer *fifo_p,
                                  struct bhi360_dev *dev);

/*function to check the return values in parse_fifo function*/
static inline int8_t check_return_value(int8_t result)
{

    if (result != BHI360_OK)
    {
        return result;
    }

    return BHI360_OK;
}

int8_t bhi360_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_regs(reg_addr, reg_data, length, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint16_t length, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_set_regs(reg_addr, reg_data, length, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

static int8_t bhi360_get_and_process_fifo_support(uint8_t *int_status,
                                                  int8_t *rslt,
                                                  struct bhi360_fifo_buffer *fifo_temp,
                                                  struct bhi360_dev *dev)
{
    uint32_t bytes_read = 0;
    int8_t temp_rslt = BHI360_OK;

    while ((*int_status || fifo_temp->remain_length) && (*rslt == BHI360_OK))
    {
        if (((BHI360_IS_INT_FIFO_W(*int_status)) == BHI360_IST_FIFO_W_DRDY) ||
            ((BHI360_IS_INT_FIFO_W(*int_status)) == BHI360_IST_FIFO_W_LTCY) ||
            ((BHI360_IS_INT_FIFO_W(*int_status)) == BHI360_IST_FIFO_W_WM) || (fifo_temp->remain_length))
        {

            /* Reset read_pos to the start of the buffer */
            fifo_temp->read_pos = 0;

            /* Append data into the work_buffer linked through fifos */
            *rslt =
                bhi360_hif_get_wakeup_fifo(&fifo_temp->buffer[fifo_temp->read_length],
                                           (fifo_temp->buffer_size - fifo_temp->read_length),
                                           &bytes_read,
                                           &fifo_temp->remain_length,
                                           &dev->hif);
            if (*rslt != BHI360_OK)
            {
                return *rslt;
            }

            fifo_temp->read_length += bytes_read;
        }

        *rslt = parse_fifo(BHI360_FIFO_TYPE_WAKEUP, fifo_temp, dev);
        *int_status = 0;
    }

    return temp_rslt;

}

int8_t bhi360_get_and_process_fifo(uint8_t *work_buffer, uint32_t buffer_size, struct bhi360_dev *dev)
{
    uint16_t int_status;
    uint8_t int_status_bak;
    uint32_t bytes_read = 0;
    int8_t rslt;
    struct bhi360_fifo_buffer fifos;

    if ((dev == NULL) || (work_buffer == NULL))
    {
        return BHI360_E_NULL_PTR;
    }

    if (buffer_size == 0)
    {
        return BHI360_E_BUFFER;
    }

    memset(work_buffer, 0, buffer_size);
    memset(&fifos, 0, sizeof(struct bhi360_fifo_buffer));

    fifos.buffer = work_buffer;
    fifos.buffer_size = buffer_size;

    rslt = bhi360_hif_get_interrupt_status(&int_status_bak, &dev->hif);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Get and process the Wake up FIFO */
    fifos.read_length = 0;
    int_status = int_status_bak;

    rslt = bhi360_get_and_process_fifo_support((uint8_t *)&int_status, &rslt, &fifos, dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Get and process the Non Wake-up FIFO */
    fifos.read_length = 0;
    int_status = int_status_bak;
    while ((int_status || fifos.remain_length) && (rslt == BHI360_OK))
    {
        if (((BHI360_IS_INT_FIFO_NW(int_status)) == BHI360_IST_FIFO_NW_DRDY) ||
            ((BHI360_IS_INT_FIFO_NW(int_status)) == BHI360_IST_FIFO_NW_LTCY) ||
            ((BHI360_IS_INT_FIFO_NW(int_status)) == BHI360_IST_FIFO_NW_WM) || (fifos.remain_length))
        {
            /* Reset read_pos to the start of the buffer */
            fifos.read_pos = 0;

            /* Append data into the work_buffer linked through fifos */
            rslt =
                bhi360_hif_get_nonwakeup_fifo(&fifos.buffer[fifos.read_length],
                                              (fifos.buffer_size - fifos.read_length),
                                              &bytes_read,
                                              &fifos.remain_length,
                                              &dev->hif);
            if (rslt != BHI360_OK)
            {
                return rslt;
            }

            fifos.read_length += bytes_read;
        }

        rslt = parse_fifo(BHI360_FIFO_TYPE_NON_WAKEUP, &fifos, dev);
        int_status = 0;
    }

    /* Get and process the Status fifo */
    fifos.read_length = 0;
    int_status = int_status_bak;

    rslt = process_status_fifo(rslt, int_status, &fifos, dev);

    return rslt;
}

int8_t bhi360_set_virt_sensor_range(uint8_t sensor_id, uint16_t range, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t buffer[4] = { 0 };

    if (dev != NULL)
    {
        buffer[0] = sensor_id;
        buffer[1] = (uint8_t)(range & 0xFF);
        buffer[2] = (uint8_t)((range >> 8) & 0xFF);
        buffer[3] = 0x00; /* Reserved */
        rslt = bhi360_hif_exec_cmd(BHI360_CMD_CHANGE_RANGE, buffer, 4, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_product_id(uint8_t *product_id, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_product_id(product_id, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_chip_id(uint8_t *chip_id, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_chip_id(chip_id, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_revision_id(uint8_t *revision_id, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev != NULL) && (revision_id != NULL))
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_REVISION_ID, revision_id, 1, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_rom_version(uint16_t *rom_version, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev != NULL) && (rom_version != NULL))
    {
        rslt = bhi360_hif_get_rom_version(rom_version, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_kernel_version(uint16_t *kernel_version, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_kernel_version(kernel_version, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_user_version(uint16_t *user_version, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_user_version(user_version, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_boot_status(uint8_t *boot_status, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_boot_status(boot_status, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_host_status(uint8_t *host_status, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_host_status(host_status, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_get_feature_status(uint8_t *feat_status, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_get_feature_status(feat_status, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_upload_firmware_to_ram(const uint8_t *firmware, uint32_t length, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (firmware == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_upload_firmware_to_ram(firmware, length, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_upload_firmware_to_ram_partly(const uint8_t *firmware,
                                            uint32_t total_size,
                                            uint32_t cur_pos,
                                            uint32_t packet_len,
                                            struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (firmware == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_upload_firmware_to_ram_partly(firmware, total_size, cur_pos, packet_len, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_boot_from_ram(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_boot_program_ram(&dev->hif);
    }

    return rslt;
}

int8_t bhi360_set_host_interrupt_ctrl(uint8_t hintr_ctrl, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERRUPT_CTRL, &hintr_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_host_interrupt_ctrl(uint8_t *hintr_ctrl, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERRUPT_CTRL, hintr_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_interrupt_status(uint8_t *int_status, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_get_interrupt_status(int_status, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_set_host_intf_ctrl(uint8_t hintf_ctrl, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &hintf_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_host_intf_ctrl(uint8_t *hintf_ctrl, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, hintf_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_set_fifo_format_ctrl(uint8_t param, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t buffer[4] = { 0 };

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        buffer[0] = param & BHI360_FIFO_FORMAT_CTRL_MASK;
        rslt = bhi360_hif_exec_cmd(BHI360_CMD_FIFO_FORMAT_CTRL, buffer, 4, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_set_timestamp_event_req(uint8_t ts_ev_req, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t buffer;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        if (ts_ev_req)
        {
            buffer = 0x01;
        }
        else
        {
            buffer = 0x00;
        }

        rslt = bhi360_hif_set_regs(BHI360_REG_TIME_EV_REQ, &buffer, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_hw_timestamp_ns(uint64_t *timestamp_ns, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (timestamp_ns == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_get_hw_timestamp(timestamp_ns, &dev->hif);
        *timestamp_ns *= UINT64_C(15625);
    }

    return rslt;
}

int8_t bhi360_set_host_ctrl(uint8_t host_ctrl, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_set_regs(BHI360_REG_HOST_CTRL, &host_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_host_ctrl(uint8_t *host_ctrl, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (host_ctrl == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_CTRL, host_ctrl, 1, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_soft_passthrough_transfer(union bhi360_soft_passthrough_conf *conf,
                                        uint8_t reg_addr,
                                        uint8_t length,
                                        uint8_t *reg_data,
                                        struct bhi360_dev *dev)
{
    uint32_t bytes_read = 0;
    int8_t rslt = BHI360_OK;
    uint8_t cmd[BHI360_COMMAND_PACKET_LEN] = { 0 };
    uint8_t tmp_rd_buf[BHI360_SPASS_WRITE_RESP_PACKET_LEN] = { 0 };
    uint16_t buffer_size = 0;

    if ((dev == NULL) || (conf == NULL) || (reg_data == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        conf->conf.trans_count = length;
        conf->conf.reg = reg_addr;
        if (conf->conf.direction == BHI360_SPASS_READ)
        {
            if ((BHI360_SPASS_READ_PACKET_LEN + length) % 4)
            {
                buffer_size = (uint16_t)(((BHI360_SPASS_READ_PACKET_LEN + length) / 4 + 1) * 4);
            }
            else
            {
                buffer_size = (uint16_t)(BHI360_SPASS_READ_PACKET_LEN + length);
            }

            if (buffer_size <= BHI360_COMMAND_PACKET_LEN)
            {
                rslt = bhi360_hif_exec_soft_passthrough(conf->data, 8, cmd, buffer_size, &bytes_read, &dev->hif);
                memcpy(reg_data, &cmd[9], length);
            }
            else
            {
                rslt = BHI360_E_BUFFER;
            }
        }
        else
        {
            if ((8 + length) % 4)
            {
                buffer_size = (uint16_t)(((8 + length) / 4 + 1) * 4);
            }
            else
            {
                buffer_size = (uint16_t)(8 + length);
            }

            if (buffer_size <= BHI360_COMMAND_PACKET_LEN)
            {
                memcpy(&cmd[0], conf->data, 8);
                memcpy(&cmd[8], reg_data, length);
                rslt = bhi360_hif_exec_soft_passthrough(&cmd[0],
                                                        buffer_size,
                                                        tmp_rd_buf,
                                                        sizeof(tmp_rd_buf),
                                                        &bytes_read,
                                                        &dev->hif);
            }
            else
            {
                rslt = BHI360_E_BUFFER;
            }
        }
    }

    return rslt;
}

int8_t bhi360_flush_fifo(uint8_t sensor_id, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_set_fifo_flush(sensor_id, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_soft_reset(struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_reset(&dev->hif);
    }

    return rslt;
}

int8_t bhi360_perform_self_test(uint8_t phys_sensor_id,
                                struct bhi360_self_test_resp *self_test_resp,
                                struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (self_test_resp == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_do_self_test(phys_sensor_id, self_test_resp, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_perform_foc(uint8_t phys_sensor_id, struct bhi360_foc_resp *foc_status, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (foc_status == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_do_foc(phys_sensor_id, foc_status, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_orientation_matrix(uint8_t phys_sensor_id,
                                     struct bhi360_system_param_orient_matrix *orient_matrix,
                                     struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    struct bhi360_system_param_phys_sensor_info phy_sen_info;

    if ((dev == NULL) || (orient_matrix == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_system_param_get_physical_sensor_info(phys_sensor_id, &phy_sen_info, dev);

        /*lint -e702 Info 702: Shift right of signed quantity (int) */
        orient_matrix->c[0] = (int8_t)((phy_sen_info.orientation_matrix[0] & 0x0F) << 4) >> 4;
        orient_matrix->c[1] = (int8_t)(phy_sen_info.orientation_matrix[0] & 0xF0) >> 4;
        orient_matrix->c[2] = (int8_t)((phy_sen_info.orientation_matrix[1] & 0x0F) << 4) >> 4;
        orient_matrix->c[3] = (int8_t)(phy_sen_info.orientation_matrix[1] & 0xF0) >> 4;
        orient_matrix->c[4] = (int8_t)((phy_sen_info.orientation_matrix[2] & 0x0F) << 4) >> 4;
        orient_matrix->c[5] = (int8_t)(phy_sen_info.orientation_matrix[2] & 0xF0) >> 4;
        orient_matrix->c[6] = (int8_t)((phy_sen_info.orientation_matrix[3] & 0x0F) << 4) >> 4;
        orient_matrix->c[7] = (int8_t)(phy_sen_info.orientation_matrix[3] & 0xF0) >> 4;
        orient_matrix->c[8] = (int8_t)((phy_sen_info.orientation_matrix[4] & 0x0F) << 4) >> 4;

        /*lint +e702 */
    }

    return rslt;
}

int8_t bhi360_get_post_mortem_data(uint8_t *post_mortem,
                                   uint32_t buffer_len,
                                   uint32_t *actual_len,
                                   struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint16_t crash_dump_status_code = 0;

    if ((dev == NULL) || (post_mortem == NULL) || (actual_len == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_get_post_mortem(&crash_dump_status_code, actual_len, post_mortem, buffer_len, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_init(enum bhi360_intf intf,
                   bhi360_read_fptr_t read,
                   bhi360_write_fptr_t write,
                   bhi360_delay_us_fptr_t delay_us,
                   uint32_t read_write_len,
                   void *intf_ptr,
                   struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        memset(dev, 0, sizeof(struct bhi360_dev));

        rslt = bhi360_hif_init(intf, read, write, delay_us, read_write_len, intf_ptr, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_register_fifo_parse_callback(uint8_t sensor_id,
                                           bhi360_fifo_parse_callback_t callback,
                                           void *callback_ref,
                                           struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t i = 0;

    if ((dev == NULL) || (callback == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
        {
            if (dev->table[i].sensor_id == 0)
            {
                dev->table[i].sensor_id = sensor_id;
                dev->table[i].callback = callback;
                dev->table[i].callback_ref = callback_ref;
                break;
            }
        }

        if (i == BHI360_MAX_SIMUL_SENSORS)
        {
            rslt = BHI360_E_INSUFFICIENT_MAX_SIMUL_SENSORS;
        }
    }

    return rslt;
}

int8_t bhi360_deregister_fifo_parse_callback(uint8_t sensor_id, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t i = 0;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
        {
            if (dev->table[i].sensor_id == sensor_id)
            {
                dev->table[i].sensor_id = 0;
                dev->table[i].callback = NULL;
                dev->table[i].callback_ref = NULL;
                break;
            }
        }
    }

    return rslt;
}

int8_t bhi360_update_virtual_sensor_list(struct bhi360_dev *dev)
{
    int8_t rslt;
    uint8_t sensor_id;
    struct bhi360_virtual_sensor_info_param_info info;
    uint8_t sensor_index;
    uint8_t bit_mask;

    if (dev == NULL)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        /* Each bit corresponds to presence of Virtual sensor */
        rslt = bhi360_system_param_get_virtual_sensor_present(dev);
        if (rslt == BHI360_OK)
        {
            /* Padding: Sensor id*/
            dev->event_size[0] = 1;

            for (sensor_id = 1; (sensor_id < BHI360_SPECIAL_SENSOR_ID_OFFSET) && (rslt == BHI360_OK); sensor_id++)
            {
                sensor_index = (uint8_t)(sensor_id / 8);
                bit_mask = (uint8_t)(0x01 << (sensor_id % 8));

                if (dev->present_buff[sensor_index] & bit_mask)
                {
                    rslt = bhi360_virtual_sensor_info_param_get_info(sensor_id, &info, dev);
                    if (rslt == BHI360_OK)
                    {
                        if (info.event_size == 0)
                        {
                            rslt = BHI360_E_INVALID_EVENT_SIZE;
                            break;
                        }
                        else
                        {
                            dev->event_size[sensor_id] = info.event_size;
                        }
                    }
                }
            }

            for (sensor_id = BHI360_N_VIRTUAL_SENSOR_MAX - 1; sensor_id >= BHI360_SPECIAL_SENSOR_ID_OFFSET; sensor_id--)
            {
                dev->event_size[sensor_id] = bhi360_sysid_event_size[sensor_id - BHI360_SPECIAL_SENSOR_ID_OFFSET];
            }
        }
    }

    return rslt;
}

int8_t bhi360_get_sensor_info(uint8_t sensor_id,
                              struct bhi360_virtual_sensor_info_param_info *info,
                              struct bhi360_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (info == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_virtual_sensor_info_param_get_info(sensor_id, info, dev);
    }

    return rslt;
}

int8_t bhi360_set_parameter(uint16_t param, const uint8_t *buffer, uint32_t length, struct bhi360_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (buffer == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_exec_cmd(param, buffer, length, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_parameter(uint16_t param,
                            uint8_t *buffer,
                            uint32_t length,
                            uint32_t *actual_len,
                            struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (buffer == NULL) || (actual_len == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_hif_get_parameter(param, buffer, length, actual_len, &dev->hif);
    }

    return rslt;
}

int8_t bhi360_get_error_value(uint8_t *error_value, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if ((dev == NULL) || (error_value == NULL))
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        rslt = bhi360_get_regs(BHI360_REG_ERROR_VALUE, error_value, 1, dev);
    }

    return rslt;
}

int8_t bhi360_set_data_injection_mode(enum bhi360_data_inj_mode mode, struct bhi360_dev *dev)
{
    uint8_t data_inj_payload[8] = { 0 };
    int8_t rslt = BHI360_OK;
    uint8_t work_buffer[256] = { 0 };
    uint32_t actual_len = 0;
    uint16_t cmd = BHI360_CMD_SET_INJECT_MODE;

    data_inj_payload[0] = (uint8_t)(cmd & 0xFF);
    data_inj_payload[1] = (uint8_t)((cmd >> 8) & 0xFF);
    data_inj_payload[2] = BHI360_DATA_INJECT_MODE_PAYLOAD_LEN;

    if (dev != NULL)
    {

        switch (mode)
        {
            case BHI360_NORMAL_MODE:
                data_inj_payload[4] = 0;
                break;
            case BHI360_REAL_TIME_INJECTION:
                data_inj_payload[4] = 1;
                break;
            case BHI360_STEP_BY_STEP_INJECTION:
                data_inj_payload[4] = 2;
                break;
            default:
                rslt = BHI360_E_INVALID_PARAM;
        }

        if (rslt == BHI360_OK)
        {
            rslt = bhi360_hif_set_inject_data_mode(data_inj_payload,
                                                   sizeof (data_inj_payload),
                                                   work_buffer,
                                                   256,
                                                   &actual_len,
                                                   &dev->hif);
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhi360_dev *dev)
{
    return bhi360_hif_inject_data(payload, payload_len, &dev->hif);
}

uint8_t bhi360_is_sensor_available(uint8_t sensor_id, const struct bhi360_dev *dev)
{
    uint8_t offset = sensor_id / 8;
    uint8_t mask = 1U << (sensor_id & 0x7);
    uint8_t sensor_present = 0;

    if (dev != NULL)
    {
        sensor_present = (dev->present_buff[offset] & mask);
    }

    return sensor_present;
}

uint8_t bhi360_is_physical_sensor_available(uint8_t sensor_id, const struct bhi360_dev *dev)
{
    uint8_t offset = (uint8_t)(sensor_id / 8);
    uint8_t mask = (uint8_t)(0x01 << (sensor_id & 0x7));
    uint8_t physical_sensor_present = 0;

    if (dev != NULL)
    {
        physical_sensor_present = (dev->phy_present_buff[offset] & mask);
    }

    return physical_sensor_present;
}

/**
 * @brief Function to get the BHy variant ID
 */
int8_t bhi360_get_variant_id(uint32_t *variant_id, struct bhi360_dev *dev)
{
    int8_t rslt;
    uint8_t cfg_read_command[] = { 0x16, 0x00, 0x00, 0x00 };
    uint8_t buffer[VARI_DATA_BUF_SIZE];

    rslt = bhi360_soft_reset(dev);
    if (rslt == BHI360_OK)
    {
        rslt = bhi360_set_regs(BHI360_REG_CHAN_CMD, cfg_read_command, sizeof(cfg_read_command), dev);

        /* Give the device some time to prepare data */
        dev->hif.delay_us(CMD_FIFO_DELAY_TIMEOUT_US, NULL);
    }

    for (uint8_t idx = 0U; idx < sizeof(buffer); idx++)
    {
        buffer[idx] = 0U;
    }

    if (rslt == BHI360_OK)
    {
        /* Read status FIFO. First check if response is correct */
        rslt = bhi360_get_regs(BHI360_REG_CHAN_STATUS, buffer, 4, dev);
    }

    if (rslt == BHI360_OK)
    {
        if (!((buffer[0] == 0x10) && (buffer[1] == 0x00) && (buffer[2] == sizeof(buffer)) && (buffer[3] == 0)))
        {
            rslt = BHI360_E_INVALID_PARAM;
        }
    }

    if (rslt == BHI360_OK)
    {
        /* Read out the configuration */
        rslt = bhi360_get_regs(BHI360_REG_CHAN_STATUS, buffer, sizeof(buffer), dev);
        if (rslt == BHI360_OK)
        {
            *variant_id = ((uint32_t)buffer[40] << 24) | ((uint32_t)buffer[41] << 16) | ((uint32_t)buffer[42] << 8) |
                          buffer[43];
        }
    }

    return rslt;
}

static int8_t get_callback_info(uint8_t sensor_id, struct bhi360_fifo_parse_callback_table *info,
                                struct bhi360_dev *dev)
{

    int8_t rslt = BHI360_OK;
    uint8_t i = 0, j;

    if ((dev != NULL) && (info != NULL))
    {
        for (j = 0; j < BHI360_MAX_SIMUL_SENSORS; j++)
        {
            if (sensor_id == dev->table[j].sensor_id)
            {
                *info = dev->table[j];
                i = j;
                break;
            }
        }

        if (i == BHI360_MAX_SIMUL_SENSORS)
        {
            rslt = BHI360_E_INSUFFICIENT_MAX_SIMUL_SENSORS;
        }
        else
        {
            if ((sensor_id >= BHI360_SPECIAL_SENSOR_ID_OFFSET) && (dev->event_size[sensor_id] == 0))
            {
                dev->event_size[sensor_id] = bhi360_sysid_event_size[sensor_id - BHI360_SPECIAL_SENSOR_ID_OFFSET];
            }

            if ((sensor_id == 0) && (dev->event_size[sensor_id] == 0))
            {
                dev->event_size[sensor_id] = 1;
            }
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

static int8_t get_buffer_status(const struct bhi360_fifo_buffer *fifo_p, uint8_t event_size, buffer_status_t *status)
{
    if ((fifo_p->read_pos + event_size) <= fifo_p->read_length)
    {
        *status = BHI360_BUFFER_STATUS_OK;
    }
    else
    {
        *status = BHI360_BUFFER_STATUS_RELOAD;
    }

    return BHI360_OK;
}

static int8_t get_time_stamp(enum bhi360_fifo_type source, uint64_t **time_stamp, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (source < BHI360_FIFO_TYPE_MAX)
    {
        *time_stamp = &dev->last_time_stamp[source];
    }
    else
    {
        rslt = BHI360_E_INVALID_FIFO_TYPE;
    }

    return rslt;
}

static int8_t parse_fifo_support(struct bhi360_fifo_buffer *fifo_buf)
{
    uint8_t i;

    if (fifo_buf->read_length)
    {
        if (fifo_buf->read_length < fifo_buf->read_pos)
        {
            return BHI360_E_INVALID_PARAM;
        }

        fifo_buf->read_length -= fifo_buf->read_pos;
        if (fifo_buf->read_length)
        {
            for (i = 0; i < fifo_buf->read_length; i++)
            {
                fifo_buf->buffer[i] = fifo_buf->buffer[fifo_buf->read_pos + i];
            }
        }
    }

    return BHI360_OK;
}

static int8_t parse_fifo(enum bhi360_fifo_type source, struct bhi360_fifo_buffer *fifo_p, struct bhi360_dev *dev)
{
    uint8_t tmp_sensor_id = 0;
    int8_t rslt = BHI360_OK;
    uint32_t tmp_read_pos;
    struct bhi360_fifo_parse_data_info data_info;
    uint64_t *time_stamp;
    struct bhi360_fifo_parse_callback_table info = { 0 };
    buffer_status_t status = BHI360_BUFFER_STATUS_OK;

    for (; (fifo_p->read_pos < fifo_p->read_length) && (status == BHI360_BUFFER_STATUS_OK);)
    {
        tmp_read_pos = fifo_p->read_pos;
        tmp_sensor_id = fifo_p->buffer[tmp_read_pos];

        rslt = get_time_stamp(source, &time_stamp, dev);
        rslt = check_return_value(rslt);
        switch (tmp_sensor_id)
        {
            case BHI360_SYS_ID_FILLER:
            case BHI360_SYS_ID_PADDING:
                fifo_p->read_pos += 1;
                break;
            case BHI360_SYS_ID_TS_SMALL_DELTA_WU:
            case BHI360_SYS_ID_TS_SMALL_DELTA:
                rslt = get_buffer_status(fifo_p, 2, &status);
                rslt = check_return_value(rslt);
                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp += fifo_p->buffer[tmp_read_pos + 1];
                fifo_p->read_pos += 2;
                break;
            case BHI360_SYS_ID_TS_LARGE_DELTA:
            case BHI360_SYS_ID_TS_LARGE_DELTA_WU:
                rslt = get_buffer_status(fifo_p, 3, &status);
                rslt = check_return_value(rslt);

                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp += BHI360_LE2U16(fifo_p->buffer + tmp_read_pos + 1);
                fifo_p->read_pos += 3;
                break;
            case BHI360_SYS_ID_BHI360_LOG_DOSTEP:
                rslt = get_buffer_status(fifo_p, 23, &status);
                rslt = check_return_value(rslt);

                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                fifo_p->read_pos += 23;

                break;
            case BHI360_SYS_ID_TS_FULL:
            case BHI360_SYS_ID_TS_FULL_WU:
                rslt = get_buffer_status(fifo_p, 6, &status);
                rslt = check_return_value(rslt);

                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp = BHI360_LE2U40(fifo_p->buffer + tmp_read_pos + UINT8_C(1));
                fifo_p->read_pos += 6;
                break;
            default:
                memset(&info, 0, sizeof(struct bhi360_fifo_parse_callback_table));
                rslt = get_callback_info(tmp_sensor_id, &info, dev);
                rslt = check_return_value(rslt);
                rslt = get_buffer_status(fifo_p, dev->event_size[tmp_sensor_id], &status); /*lint !e838 suppressing
                                                                                            * previously assigned value
                                                                                            * not used info */
                rslt = check_return_value(rslt);
                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                if (info.callback != NULL)
                {
                    /* Read position is incremented by 1 to exclude sensor id */
                    data_info.data_ptr = &fifo_p->buffer[tmp_read_pos + 1];
                    data_info.fifo_type = source;
                    data_info.time_stamp = time_stamp;
                    data_info.sensor_id = tmp_sensor_id;
                    data_info.data_size = dev->event_size[tmp_sensor_id];
                    info.callback(&data_info, info.callback_ref);
                }

                fifo_p->read_pos += dev->event_size[tmp_sensor_id];
                break;
        }
    }

    rslt = parse_fifo_support(fifo_p);

    return rslt;
}

int8_t bhi360_clear_fifo(uint8_t flush_cfg, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t buffer[4] = { 0 };

    if (dev != NULL)
    {
        buffer[0] = flush_cfg;
        rslt = bhi360_hif_exec_cmd(BHI360_CMD_FIFO_FLUSH, buffer, 4, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_read_status(uint16_t *status_code,
                          uint8_t *status_buff,
                          uint32_t status_len,
                          uint32_t *actual_len,
                          struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (dev != NULL)
    {
        rslt = bhi360_hif_wait_status_ready(&dev->hif);
        if (rslt != BHI360_OK)
        {
            return rslt;
        }

        rslt = bhi360_hif_get_status_fifo(status_code, status_buff, status_len, actual_len, &dev->hif);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

static int8_t parse_status_fifo(struct bhi360_fifo_buffer *fifo_p, struct bhi360_dev *dev)
{
    uint8_t tmp_sensor_id = 0;
    int8_t rslt = BHI360_OK;
    uint32_t tmp_read_pos;
    struct bhi360_fifo_parse_data_info data_info;
    uint64_t *time_stamp;
    struct bhi360_fifo_parse_callback_table info = { 0 };
    buffer_status_t status = BHI360_BUFFER_STATUS_OK;

    for (; (fifo_p->read_pos < fifo_p->read_length) && (status == BHI360_BUFFER_STATUS_OK);)
    {
        tmp_read_pos = fifo_p->read_pos;
        tmp_sensor_id = fifo_p->buffer[tmp_read_pos];

        rslt = get_time_stamp(BHI360_FIFO_TYPE_STATUS, &time_stamp, dev);
        rslt = check_return_value(rslt);
        switch (tmp_sensor_id)
        {
            case BHI360_SYS_ID_FILLER:
            case BHI360_SYS_ID_PADDING:
                fifo_p->read_pos += 1;
                break;
            case BHI360_SYS_ID_TS_SMALL_DELTA_WU:
            case BHI360_SYS_ID_TS_SMALL_DELTA:
                rslt = get_buffer_status(fifo_p, BHI360_TS_SMALL_DELTA_FIFO_RD_SIZE, &status);
                rslt = check_return_value(rslt);
                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp += fifo_p->buffer[tmp_read_pos + 1];
                fifo_p->read_pos += BHI360_TS_SMALL_DELTA_FIFO_RD_SIZE;
                break;
            case BHI360_SYS_ID_TS_LARGE_DELTA:
            case BHI360_SYS_ID_TS_LARGE_DELTA_WU:
                rslt = get_buffer_status(fifo_p, BHI360_TS_LARGE_DELTA_RD_FIFO_SIZE, &status);
                rslt = check_return_value(rslt);

                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp += BHI360_LE2U16(fifo_p->buffer + tmp_read_pos + 1);
                fifo_p->read_pos += BHI360_TS_LARGE_DELTA_RD_FIFO_SIZE;
                break;
            case BHI360_SYS_ID_BHI360_LOG_DOSTEP:
                rslt = get_buffer_status(fifo_p, BHI360_LOG_DOSTEP_RD_FIFO_SIZE, &status);
                rslt = check_return_value(rslt);

                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                fifo_p->read_pos += BHI360_LOG_DOSTEP_RD_FIFO_SIZE;

                break;
            case BHI360_SYS_ID_TS_FULL:
            case BHI360_SYS_ID_TS_FULL_WU:
                rslt = get_buffer_status(fifo_p, BHI360_TS_FULL_RD_FIFO_SIZE, &status);
                rslt = check_return_value(rslt);

                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                *time_stamp = BHI360_LE2U40(fifo_p->buffer + tmp_read_pos + UINT8_C(1));
                fifo_p->read_pos += BHI360_TS_FULL_RD_FIFO_SIZE;
                break;
            default:
                rslt = get_callback_info(tmp_sensor_id, &info, dev);
                rslt = check_return_value(rslt);
                rslt = get_buffer_status(fifo_p, dev->event_size[tmp_sensor_id], &status); /*lint !e838 suppressing
                                                                * previously assigned value
                                                                * not used info */
                rslt = check_return_value(rslt);
                if (status != BHI360_BUFFER_STATUS_OK)
                {
                    break;
                }

                if (info.callback != NULL)
                {
                    /* Read position is incremented by 1 to exclude sensor id */
                    data_info.data_ptr = &fifo_p->buffer[tmp_read_pos + 1];
                    data_info.fifo_type = BHI360_FIFO_TYPE_STATUS;
                    data_info.time_stamp = time_stamp;
                    data_info.sensor_id = tmp_sensor_id;
                    data_info.data_size = dev->event_size[tmp_sensor_id];
                    info.callback(&data_info, info.callback_ref);
                }

                fifo_p->read_pos += dev->event_size[tmp_sensor_id];
                break;
        }
    }

    rslt = parse_fifo_support(fifo_p);

    return rslt;
}

static int8_t process_status_fifo(int8_t rslt,
                                  uint16_t int_status,
                                  struct bhi360_fifo_buffer *fifo_p,
                                  struct bhi360_dev *dev)
{
    int8_t ret_val = rslt;
    uint32_t bytes_read = 0;
    uint16_t int_status_back = int_status;

    while ((int_status_back || fifo_p->remain_length) && (ret_val == BHI360_OK))
    {
        if ((((BHI360_IS_INT_ASYNC_STATUS(int_status_back)) == BHI360_IST_MASK_DEBUG) || (fifo_p->remain_length)))
        {
            fifo_p->read_pos = 0;
            ret_val =
                bhi360_hif_get_status_fifo_async(&fifo_p->buffer[fifo_p->read_length],
                                                 (fifo_p->buffer_size - fifo_p->read_length),
                                                 &bytes_read,
                                                 &fifo_p->remain_length,
                                                 &dev->hif);
            if (ret_val != BHI360_OK)
            {
                return ret_val;
            }

            fifo_p->read_length += bytes_read;
        }
        else if ((BHI360_IS_INT_STATUS(int_status_back)) == BHI360_IST_MASK_STATUS)
        {
            fifo_p->read_pos = 0;
            ret_val =
                bhi360_hif_get_status_fifo(&int_status_back, &fifo_p->buffer[fifo_p->read_length],
                                           (fifo_p->buffer_size - fifo_p->read_length), &bytes_read, &dev->hif);
            if (ret_val != BHI360_OK)
            {
                return ret_val;
            }

            fifo_p->read_length += bytes_read;
        }

        ret_val = parse_status_fifo(fifo_p, dev);
        int_status_back = 0;
    }

    return ret_val;
}
