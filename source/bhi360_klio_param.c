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
* @file       bhi360_klio_param.c
* @date       2026-08-14
* @version    v2.3.1
*
*/

/*********************************************************************/
/* system header files */
#include <string.h>
#include <stdio.h>

/*********************************************************************/
/* BHY SensorAPI header files */
#include "bhi360.h"

/*********************************************************************/
/* own header files */
#include "bhi360_klio_param.h"

int8_t bhi360_klio_param_read_pattern(const uint8_t id, uint8_t *buffer, uint16_t *length, struct bhi360_dev *dev)
{
    int8_t rslt;
    uint32_t ret_len;
    bhi360_klio_param_pattern_transfer_t pattern = { 0 };

    if (!buffer || !length || !dev)
    {
        return BHI360_E_NULL_PTR;
    }

    if (id != 0 || *length < sizeof(pattern.pattern_data))
    {
        return BHI360_E_INVALID_PARAM;
    }

    rslt = bhi360_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN), (uint8_t*)&pattern, sizeof(pattern), &ret_len, dev);

    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    if (ret_len != sizeof(pattern))
    {
        return BHI360_E_INVALID_PARAM;
    }

    memcpy(buffer, pattern.pattern_data, pattern.full_size);
    *length = pattern.full_size;

    return rslt;
}

int8_t bhi360_klio_param_read_reset_driver_status(uint32_t *klio_driver_status, struct bhi360_dev *dev)
{
    uint32_t ret_len = 0;
    int8_t rslt;

    if (!klio_driver_status || !dev)
    {
        return BHI360_E_NULL_PTR;
    }

    do
    {
        rslt =
            bhi360_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_DRIVER_STATUS),
                                 (uint8_t*)klio_driver_status,
                                 sizeof(*klio_driver_status),
                                 &ret_len,
                                 dev);

        if (rslt != BHI360_OK)
        {
            return rslt;
        }
    } while (*klio_driver_status == KLIO_DRIVER_ERROR_OPERATION_PENDING);

    return rslt;
}

int8_t bhi360_klio_param_write_pattern(const uint8_t idx,
                                       const uint8_t *pattern_data,
                                       const uint16_t size,
                                       struct bhi360_dev *dev)
{
    bhi360_klio_param_pattern_transfer_t pattern = {
        .block_id = 0, .block_size = (uint8_t)size, .full_size = size, .pattern_id = idx, .pattern_data = { 0 }
    };

    if (!dev || size > sizeof(pattern.pattern_data))
    {
        return BHI360_E_INVALID_PARAM;
    }

    memcpy(pattern.pattern_data, pattern_data, size);

    return bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN), (const uint8_t*)&pattern, sizeof(pattern), dev);
}

int8_t bhi360_klio_param_set_pattern_states(const bhi360_klio_param_pattern_state_t operation,
                                            const uint8_t *pattern_ids,
                                            const uint16_t count,
                                            struct bhi360_dev *dev)
{
    /* 2 bytes header: operation + number of patterns */
    const uint32_t header_size = 2;
    uint32_t buffer_size = sizeof(bhi360_klio_param_pattern_state_op_t) + count;
    uint8_t buffer[buffer_size];

    if (!dev || count == 0)
    {
        return BHI360_E_INVALID_PARAM;
    }

    buffer[0] = operation;
    buffer[1] = (uint8_t)count;

    for (uint8_t i = 0; i < count; i++)
    {
        buffer[i + header_size] = pattern_ids[i];
    }

    return bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_STATE), buffer, buffer_size, dev);
}

int8_t bhi360_klio_param_set_state(const bhi360_klio_param_sensor_state_t *state, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;
    uint8_t buffer[4];

    if (!dev || !state)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        buffer[0] = state->learning_enabled;
        buffer[1] = state->learning_reset;
        buffer[2] = state->recognition_enabled;
        buffer[3] = state->recognition_reset;

        rslt = bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGORITHM_STATE), buffer, sizeof(buffer), dev);
    }

    return rslt;
}

int8_t bhi360_klio_param_get_state(bhi360_klio_param_sensor_state_t *state, struct bhi360_dev *dev)
{
    int8_t rslt = BHI360_OK;

    if (!dev || !state)
    {
        rslt = BHI360_E_NULL_PTR;
    }
    else
    {
        uint8_t buffer[4];
        uint32_t ret_len;

        rslt = bhi360_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGORITHM_STATE), buffer, sizeof(buffer), &ret_len, dev);
        if (rslt == BHI360_OK && ret_len == sizeof(buffer))
        {
            state->learning_enabled = buffer[0];
            state->learning_reset = buffer[1];
            state->recognition_enabled = buffer[2];
            state->recognition_reset = buffer[3];
        }
    }

    return rslt;
}

int8_t bhi360_klio_param_similarity_score(const uint8_t *first_pattern,
                                          const uint8_t *second_pattern,
                                          const uint16_t size,
                                          float *similarity,
                                          struct bhi360_dev *dev)
{
    uint32_t klio_driver_status = 0;
    int8_t rslt;
    uint32_t ret_len;
    bhi360_klio_param_pattern_transfer_t pattern1 = { 0 }, pattern2 = { 0 };
    uint8_t buffer[4];

    if (!dev || size > sizeof(pattern1.pattern_data))
    {
        return BHI360_E_INVALID_PARAM;
    }

    pattern1.block_size = pattern2.block_size = (uint8_t)size;
    pattern1.full_size = pattern2.full_size = size;
    memcpy(pattern1.pattern_data, first_pattern, size);
    memcpy(pattern2.pattern_data, second_pattern, size);

    rslt = bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY),
                                (uint8_t*)&pattern1,
                                sizeof(pattern1),
                                dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    rslt = bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY),
                                (uint8_t*)&pattern2,
                                sizeof(pattern2),
                                dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Wait for similarity calculation to complete. */
    rslt = bhi360_klio_param_read_reset_driver_status(&klio_driver_status, dev);
    if (rslt != BHI360_OK || klio_driver_status != KLIO_DRIVER_ERROR_NONE)
    {
        return BHI360_E_INVALID_PARAM;
    }

    rslt = bhi360_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY), buffer, sizeof(buffer), &ret_len, dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    if (ret_len != sizeof(buffer))
    {
        return BHI360_E_INVALID_PARAM;
    }

    memcpy(similarity, buffer, sizeof(*similarity));

    return BHI360_OK;
}

int8_t bhi360_klio_param_similarity_score_multiple(const uint8_t idx,
                                                   const uint8_t *indexes,
                                                   const uint8_t count,
                                                   float *similarity,
                                                   struct bhi360_dev *dev)
{
    uint32_t klio_driver_status;
    int8_t rslt;
    uint32_t ret_len;
    uint32_t buffer_size = sizeof(bhi360_klio_param_similarity_calculation_t) + count;

    buffer_size = BHI360_ROUND_WORD_HIGHER(buffer_size);
    uint8_t buffer[buffer_size];
    memset(buffer, 0, buffer_size);

    if (!indexes || !similarity || !dev)
    {
        return BHI360_E_NULL_PTR;
    }

    /* buffer[0-3] values are initialized to zero's to distinguish from regular pattern comparison*/
    buffer[4] = idx;
    buffer[5] = count;
    for (uint8_t i = 0; i < count; i++)
    {
        buffer[i + 6] = indexes[i];
    }

    /* Start similarity calculations. */
    rslt = bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY), (uint8_t*)buffer, buffer_size, dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    /* Wait for similarity calculation to complete. */
    rslt = bhi360_klio_param_read_reset_driver_status(&klio_driver_status, dev);
    if (rslt != BHI360_OK || klio_driver_status != KLIO_DRIVER_ERROR_NONE)
    {
        return BHI360_E_INVALID_PARAM;
    }

    /* Read similarity results. */
    rslt = bhi360_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_SIMILARITY),
                                (uint8_t *)similarity,
                                sizeof(float) * count,
                                &ret_len,
                                dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    return BHI360_OK;
}

int8_t bhi360_klio_param_set_parameter(const bhi360_klio_param_t id,
                                       const void *parameter_data,
                                       const uint16_t size,
                                       struct bhi360_dev *dev)
{
    bhi360_klio_param_wrapper_t buffer;

    buffer.id = id;
    buffer.flags = 0;
    buffer.size = (uint8_t)size;
    memcpy(buffer.payload.data, parameter_data, size);

    /* Size is 3 bytes header + buffer.size, rounded up to a four byte multiple. */
    return bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER),
                                (uint8_t*)&buffer,
                                BHI360_ROUND_WORD_HIGHER((3 + buffer.size)),
                                dev);
}

int8_t bhi360_klio_param_get_parameter(const bhi360_klio_param_t id,
                                       uint8_t *parameter_data,
                                       uint16_t *size,
                                       struct bhi360_dev *dev)
{
    uint32_t ret_len;
    bhi360_klio_param_wrapper_t buffer;
    int8_t rslt;

    buffer.id = id;
    buffer.flags = 0;
    buffer.size = 0;

    /* Size is 3 bytes header + buffer.size, rounded up to a four byte multiple. */
    rslt = bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER),
                                (uint8_t*)&buffer,
                                BHI360_ROUND_WORD_HIGHER((3 + buffer.size)),
                                dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    rslt = bhi360_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_ALGO_DRIVER_PARAMETER),
                                (uint8_t*)&buffer,
                                sizeof(buffer),
                                &ret_len,
                                dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    if (ret_len < 3 || buffer.size > *size)
    {
        return BHI360_E_INVALID_PARAM;
    }

    for (uint16_t i = 0; i < buffer.size; i++)
    {
        parameter_data[i] = buffer.payload.data[i];
    }

    *size = buffer.size;

    return BHI360_OK;
}

int8_t bhi360_klio_param_set_pattern_parameter(const uint8_t pattern_id,
                                               const bhi360_klio_param_pattern_parameter_t parameter_id,
                                               const void *parameter_data,
                                               const uint16_t size,
                                               struct bhi360_dev *dev)
{
    bhi360_klio_param_pattern_parameter_wrapper_t wrapper;

    if (!parameter_data || !dev)
    {
        return BHI360_E_NULL_PTR;
    }

    wrapper.header.pattern_id = pattern_id;
    wrapper.header.parameter_id = parameter_id;

    wrapper.header.size = (uint8_t)size;
    memcpy(wrapper.payload.data, parameter_data, size);

    return bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_PARAM), (uint8_t*)&wrapper, sizeof(wrapper), dev);
}

int8_t bhi360_klio_param_get_pattern_parameter(const uint8_t pattern_id,
                                               const bhi360_klio_param_pattern_parameter_t parameter_id,
                                               uint8_t *result_buffer,
                                               const uint16_t result_buffer_size,
                                               uint16_t *bytes_written,
                                               struct bhi360_dev *dev)
{
    bhi360_klio_param_pattern_parameter_wrapper_t wrapper;

    if (!result_buffer || !bytes_written || !dev)
    {
        return BHI360_E_NULL_PTR;
    }

    wrapper.header.pattern_id = pattern_id;
    wrapper.header.parameter_id = parameter_id;
    wrapper.header.size = 0;

    int8_t rslt =
        bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_PARAM), (uint8_t*)&wrapper, sizeof(wrapper.header), dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    uint32_t ret_len;
    rslt = bhi360_get_parameter(KLIO_PARAM(KLIO_HIF_PARAM_PATTERN_PARAM),
                                (uint8_t*)&wrapper,
                                sizeof(wrapper),
                                &ret_len,
                                dev);
    if (rslt != BHI360_OK)
    {
        return rslt;
    }

    if (ret_len < sizeof(wrapper.header) || wrapper.header.size > result_buffer_size)
    {
        return BHI360_E_INVALID_PARAM;
    }

    if (wrapper.header.pattern_id != pattern_id || wrapper.header.parameter_id != parameter_id)
    {
        return BHI360_E_IO;
    }

    memcpy(result_buffer, &wrapper.payload, wrapper.header.size);

    *bytes_written = wrapper.header.size;

    return BHI360_OK;
}

int8_t bhi360_klio_param_reset(struct bhi360_dev *dev)
{
    int8_t rslt;
    uint8_t buffer[1] = { 0 };

    if (!dev)
    {
        return BHI360_E_NULL_PTR;
    }

    rslt = bhi360_set_parameter(KLIO_PARAM(KLIO_HIF_PARAM_RESET), buffer, sizeof(buffer), dev);

    return rslt;
}
