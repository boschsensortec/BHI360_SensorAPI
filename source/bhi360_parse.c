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
* @file       bhi360_parse.c
* @date       2026-08-14
* @version    v2.3.1
*
*/

#include "bhi360.h"
#include "bhi360_parse.h"
#include "verbose.h"
#include "coines.h"
#include <inttypes.h>

/*SCALE FACTOR from BHI3 data sheet, IAQ data format*/
#define SCALE_IAQ_VOC                100.0
#define SCALE_IAQ_TEMP               256.0
#define SCALE_IAQ_HUMI               500.0

#define MAXIMUM_VIRTUAL_SENSOR_LIST  UINT16_C(256)
#define SENSOR_ID_TIME_NS            UINT8_C(0xF0) /* Sensor ID for the Unsigned 64bit timestamp in nanoseconds */
#define PARAM_BUF_LEN                252

static uint16_t count[MAXIMUM_VIRTUAL_SENSOR_LIST] = { 0 };
static bool enable_ds[MAXIMUM_VIRTUAL_SENSOR_LIST] = { false };
static int16_t odr_ds[MAXIMUM_VIRTUAL_SENSOR_LIST] = { 0 };
static uint64_t last_hex_timestamp = 0;
static klio_info k_info;

/**
* @brief Function to convert time in tick to seconds and nanoseconds
* @param[in] time_ticks : Time in ticks
* @param[out] s         : Second part of time
* @param[out] ns        : Nanosecond part of time
* @param[out] tns       : Total time in nanoseconds
*/
static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

/**
* @brief Function to parse sensor status meta event
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] byte1         : Byte 1 in meta event
* @param[in] byte2         : Byte 2 in meta event
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_sensor_status(char *event_text,
                                           uint32_t s,
                                           uint32_t ns,
                                           uint8_t byte1,
                                           uint8_t byte2,
                                           struct bhi360_parse_ref *parse_table)
{
    struct bhi360_parse_sensor_details *sensor_details;

    DATA("%s; T: %lu.%09lu; Accuracy for sensor id %u changed to %u\r\n", event_text, s, ns, byte1, byte2);
    sensor_details = bhi360_parse_get_sensor_details(byte1, parse_table);

    /*lint -e774 */
    if (parse_table && sensor_details)
    {
        sensor_details->accuracy = byte2;
    }
    else
    {
        INFO("Parse slot not defined for %u\r\n", byte1);
    }
}

/**
 * @brief Function to parse extended meta events
 * @param[in] meta_event_type : Type of the meta event
 * @param[in] byte1           : First byte of the event
 * @param[in] byte2           : Second byte of the event
 * @param[in] event_text      : Event text
 * @param[in] s               : Second part of time
 * @param[in] ns              : Nanosecond part of time
 */
static void parse_meta_event_extended(uint8_t meta_event_type,
                                      uint8_t byte1,
                                      uint8_t byte2,
                                      char*event_text,
                                      uint32_t s,
                                      uint32_t ns)
{
    switch (meta_event_type)
    {
        case BHI360_META_EVENT_MAG_DISTORTION:
            DATA("%s; T: %lu.%09lu; Magnetic distortion detected for sensor id %u changed to %u\r\n",
                 event_text,
                 s,
                 ns,
                 byte1,
                 byte2);
            break;
        case BHI360_META_EVENT_IMU_SATURATION:
            DATA("%s; T: %lu.%09lu; IMU saturation detected for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        default:
            DATA("%s; T: %lu.%09lu; Unknown meta event with id: %u\r\n", event_text, s, ns, meta_event_type);
            break;
    }
}

/**
* @brief Function to parse meta event
* @param[in] callback_info : Pointer to callback information
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_type(const struct bhi360_fifo_parse_data_info *callback_info,
                                  char *event_text,
                                  uint32_t s,
                                  uint32_t ns,
                                  struct bhi360_parse_ref *parse_table)
{
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];

    switch (meta_event_type)
    {
        case BHI360_META_EVENT_FLUSH_COMPLETE:
            DATA("%s; T: %lu.%09lu; Flush complete for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SAMPLE_RATE_CHANGED:
            DATA("%s; T: %lu.%09lu; Sample rate changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_POWER_MODE_CHANGED:
            DATA("%s; T: %lu.%09lu; Power mode changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SYSTEM_ERROR:
            DATA("%s; T: %lu.%09lu; System error event, Error Register: %u, Interrupt State Register: %u\r\n",
                 event_text,
                 s,
                 ns,
                 byte1,
                 byte2);
            break;
        case BHI360_META_EVENT_ALGORITHM_EVENTS:
            DATA("%s; T: %lu.%09lu; Algorithm event\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_SENSOR_STATUS:
            parse_meta_event_sensor_status(event_text, s, ns, byte1, byte2, parse_table);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_MAIN:
            DATA("%s; T: %lu.%09lu; BSX event (do steps main)\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_CALIB:
            DATA("%s; T: %lu.%09lu; BSX event (do steps calib)\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            DATA("%s; T: %lu.%09lu; BSX event (get output signal)\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_SENSOR_ERROR:
            DATA("%s; T: %lu.%09lu; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns, byte1, byte2);
            break;
        case BHI360_META_EVENT_FIFO_OVERFLOW:
            DATA("%s; T: %lu.%09lu; FIFO overflow\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_DYNAMIC_RANGE_CHANGED:
            DATA("%s; T: %lu.%09lu; Dynamic range changed for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_FIFO_WATERMARK:
            DATA("%s; T: %lu.%09lu; FIFO watermark reached\r\n", event_text, s, ns);
            break;
        case BHI360_META_EVENT_INITIALIZED:
            DATA("%s; T: %lu.%09lu; Firmware initialized. Firmware version %u\r\n", event_text, s, ns,
                 ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHI360_META_TRANSFER_CAUSE:
            DATA("%s; T: %lu.%09lu; Transfer cause for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SENSOR_FRAMEWORK:
            DATA("%s; T: %lu.%09lu; Sensor framework event for sensor id %u\r\n", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_RESET:
            DATA("%s; T: %lu.%09lu; Reset event. Cause : %u\r\n", event_text, s, ns, byte2);
            break;
        case BHI360_META_EVENT_SPACER:
            break;
        default:
            parse_meta_event_extended(meta_event_type, byte1, byte2, event_text, s, ns);
            break;
    }
}

/**
* @brief Function to log data
* @param[in] sid           : Sensor ID
* @param[in] tns           : Time in nanoseconds
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
* @param[in] logdev        : Device instance for log
*/
static void log_data(uint8_t sid,
                     uint64_t tns,
                     uint8_t event_size,
                     const uint8_t *event_payload,
                     struct bhi360_logbin_dev *logdev)
{
    if (logdev && logdev->logfile)
    {
#if !defined(PC) && defined(MCU_APP30)
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif

#if defined(MCU_APP31)
        coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

        bhi360_logbin_add_data(sid, tns, event_size, event_payload, logdev);

#if !defined(PC) && defined(MCU_APP30)
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

#if defined(MCU_APP31)
        coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif
    }
}

/**
 * @brief Function to stream hex data
 * @param[in] tns           : Timestamp in nanoseconds
 * @param[in] sid           : Sensor ID
 * @param[in] event_size    : Event size (number of payload bytes)
 * @param[in] event_payload : Event payload buffer
 */
static void stream_hex_data(uint64_t tns, uint8_t sid, uint8_t event_size, const uint8_t *event_payload)
{
    uint16_t i = 0;
    char hex_buffer[276] = { 0 }; /* Assuming a maximum event length in hex of 256 bytes + 2 bytes for sensor id +
                                   * 18 bytes for timestamp including timestamp id */

    if (tns != last_hex_timestamp)
    {
        last_hex_timestamp = tns;

        /* Print sensor ID */
        sprintf(hex_buffer, "%02x", SENSOR_ID_TIME_NS);

        for (i = 0; i < sizeof(tns); i++)
        {
            /* Output raw data in hex */
            sprintf(hex_buffer + strlen(hex_buffer), "%02x", ((uint8_t *)&tns)[i]);
        }
    }

    sprintf(hex_buffer + strlen(hex_buffer), "%02x", sid);

    for (i = 0; i < event_size; i++)
    {
        /* Output raw data in hex */
        sprintf(hex_buffer + strlen(hex_buffer), "%02x", event_payload[i]);
    }

    HEX("%s\r\n", hex_buffer);
}

/**
* @brief Function to print activity in string
* @param[in] activity : Activity value
*/
static void print_activity(uint16_t activity)
{
    if (activity & BHI360_STILL_ACTIVITY_ENDED)
    {
        PRINT_D(" Still activity ended,");
    }

    if (activity & BHI360_WALKING_ACTIVITY_ENDED)
    {
        PRINT_D(" Walking activity ended,");
    }

    if (activity & BHI360_RUNNING_ACTIVITY_ENDED)
    {
        PRINT_D(" Running activity ended,");
    }

    if (activity & BHI360_ON_BICYCLE_ACTIVITY_ENDED)
    {
        PRINT_D(" On bicycle activity ended,");
    }

    if (activity & BHI360_IN_VEHICLE_ACTIVITY_ENDED)
    {
        PRINT_D(" In vehicle ended,");
    }

    if (activity & BHI360_STILL_ACTIVITY_STARTED)
    {
        PRINT_D(" Still activity started,");
    }

    if (activity & BHI360_WALKING_ACTIVITY_STARTED)
    {
        PRINT_D(" Walking activity started,");
    }

    if (activity & BHI360_RUNNING_ACTIVITY_STARTED)
    {
        PRINT_D(" Running activity started,");
    }

    if (activity & BHI360_ON_BICYCLE_ACTIVITY_STARTED)
    {
        PRINT_D(" On bicycle activity started,");
    }

    if (activity & BHI360_IN_VEHICLE_ACTIVITY_STARTED)
    {
        PRINT_D(" In vehicle activity started,");
    }
}

/**
* @brief Function to get sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_get_sensor_details(uint8_t id, struct bhi360_parse_ref *ref)
{
    uint8_t i;

    for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
    {
        if (ref->sensor[i].id == id)
        {
            return &ref->sensor[i];
        }
    }

    return NULL;
}

/**
* @brief Function to add sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_add_sensor_details(uint8_t id, struct bhi360_parse_ref *ref)
{
    uint8_t i = 0;

    struct bhi360_parse_sensor_details *sensor_details;

    sensor_details = bhi360_parse_get_sensor_details(id, ref);
    if (sensor_details)
    {

        /* Slot for the sensor ID is already used */
        return sensor_details;
    }
    else
    {
        /* Find a new slot */
        for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
        {
            if (ref->sensor[i].id == 0)
            {
                INFO("Using slot %u for SID %u\r\n", i, id);
                ref->sensor[i].id = id;

                return &ref->sensor[i];
            }
        }
    }

    return NULL;
}

/**
* @brief Function to check stream log flags
* @param[in] callback_info  : Pointer to callback information
* @param[in] parse_flag     : Stream log flags
*/
static void check_stream_log_flags(const struct bhi360_fifo_parse_data_info *callback_info, uint8_t parse_flag)
{
    if ((parse_flag & PARSE_FLAG_STREAM) && (count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
    {
        if (count[callback_info->sensor_id] == odr_ds[callback_info->sensor_id])
        {
            count[callback_info->sensor_id] = 0;
        }
    }
}

/**
* @brief Function to print log for 3-axis format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] sensor_details : Pointer to sensor details
*/
static void print_log_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info,
                                struct bhi360_event_data_xyz data,
                                float scaling_factor,
                                uint32_t s,
                                uint32_t ns,
                                const struct bhi360_parse_sensor_details *sensor_details)
{
    DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f; acc: %u\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.x * scaling_factor,
         data.y * scaling_factor,
         data.z * scaling_factor,
         sensor_details->accuracy);
}

/**
* @brief Function to stream and log for 3-axis format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] sensor_details : Pointer to sensor details
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_3axis_s16(bool flag,
                                     const struct bhi360_fifo_parse_data_info *callback_info,
                                     struct bhi360_event_data_xyz data,
                                     uint32_t s,
                                     uint32_t ns,
                                     uint64_t tns,
                                     struct bhi360_parse_ref *parse_table,
                                     uint8_t parse_flag,
                                     const struct bhi360_parse_sensor_details *sensor_details,
                                     float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_3axis_s16(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_3axis_s16(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
    }

    if (parse_flag & PARSE_FLAG_HEXSTREAM)
    {
        stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for euler format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] sensor_details : Pointer to sensor details
*/
static void print_log_euler(const struct bhi360_fifo_parse_data_info *callback_info,
                            struct bhi360_event_data_orientation data,
                            float scaling_factor,
                            uint32_t s,
                            uint32_t ns,
                            const struct bhi360_parse_sensor_details *sensor_details)
{
    DATA("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f; acc: %u\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.heading * scaling_factor,
         data.pitch * scaling_factor,
         data.roll * scaling_factor,
         sensor_details->accuracy);
}

/**
* @brief Function to stream and log for euler format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] sensor_details : Pointer to sensor details
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_euler(bool flag,
                                 const struct bhi360_fifo_parse_data_info *callback_info,
                                 struct bhi360_event_data_orientation data,
                                 uint32_t s,
                                 uint32_t ns,
                                 uint64_t tns,
                                 struct bhi360_parse_ref *parse_table,
                                 uint8_t parse_flag,
                                 const struct bhi360_parse_sensor_details *sensor_details,
                                 float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_euler(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_euler(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for quaternion format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_quaternion(const struct bhi360_fifo_parse_data_info *callback_info,
                                 struct bhi360_event_data_quaternion data,
                                 uint32_t s,
                                 uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %f\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f,
         ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

/**
* @brief Function to stream and log for quaternion format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_quaternion(bool flag,
                                      const struct bhi360_fifo_parse_data_info *callback_info,
                                      struct bhi360_event_data_quaternion data,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct bhi360_parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_quaternion(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_quaternion(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 16-bit signed format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_s16_as_float(const struct bhi360_fifo_parse_data_info *callback_info,
                                   int16_t data,
                                   float scaling_factor,
                                   uint32_t s,
                                   uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, data * scaling_factor);
}

/**
* @brief Function to stream and log for 16-bit signed format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_s16_as_float(bool flag,
                                        const struct bhi360_fifo_parse_data_info *callback_info,
                                        int16_t data,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct bhi360_parse_ref *parse_table,
                                        uint8_t parse_flag,
                                        float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_s16_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_s16_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 32-bit scalar format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_u32(const struct bhi360_fifo_parse_data_info *callback_info,
                                 uint32_t data,
                                 uint32_t s,
                                 uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %lu\r\n", callback_info->sensor_id, s, ns, data);
}

/**
* @brief Function to stream and log for 32-bit scalar format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_scalar_u32(bool flag,
                                      const struct bhi360_fifo_parse_data_info *callback_info,
                                      uint32_t data,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct bhi360_parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_u32(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_scalar_u32(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for scalar event format
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_event(const struct bhi360_fifo_parse_data_info *callback_info, uint32_t s, uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu;\r\n", callback_info->sensor_id, s, ns);
}

/**
* @brief Function to stream and log for scalar event format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_scalar_event(bool flag,
                                        const struct bhi360_fifo_parse_data_info *callback_info,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct bhi360_parse_ref *parse_table,
                                        uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_event(callback_info, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_scalar_event(callback_info, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for activity format
* @param[in] callback_info  : Pointer to callback information
* @param[in] activity       : Activity value
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_activity(const struct bhi360_fifo_parse_data_info *callback_info,
                               uint16_t activity,
                               uint32_t s,
                               uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; ", callback_info->sensor_id, s, ns);

    print_activity(activity);

    PRINT_D("\r\n");
}

/**
* @brief Function to stream and log for activity format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] activity       : Activity value
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_activity(bool flag,
                                    const struct bhi360_fifo_parse_data_info *callback_info,
                                    uint16_t activity,
                                    uint32_t s,
                                    uint32_t ns,
                                    uint64_t tns,
                                    struct bhi360_parse_ref *parse_table,
                                    uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_activity(callback_info, activity, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_activity(callback_info, activity, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 24-bit unsigned format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_u24_as_float(const struct bhi360_fifo_parse_data_info *callback_info,
                                   uint32_t data,
                                   float scaling_factor,
                                   uint32_t s,
                                   uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %f\r\n", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
}

/**
* @brief Function to stream and log for 24-bit unsigned format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_u24_as_float(bool flag,
                                        const struct bhi360_fifo_parse_data_info *callback_info,
                                        uint32_t data,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct bhi360_parse_ref *parse_table,
                                        uint8_t parse_flag,
                                        float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_u24_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_u24_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 8-bit unsigned scalar format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_u8(const struct bhi360_fifo_parse_data_info *callback_info,
                                uint8_t data,
                                uint32_t s,
                                uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %u\r\n", callback_info->sensor_id, s, ns, data);
}

/**
* @brief Function to stream and log for 8-bit unsigned scalar format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_scalar_u8(bool flag,
                                 const struct bhi360_fifo_parse_data_info *callback_info,
                                 uint8_t data,
                                 uint32_t s,
                                 uint32_t ns,
                                 uint64_t tns,
                                 struct bhi360_parse_ref *parse_table,
                                 uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_u8(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_scalar_u8(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for generic format
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_generic(const struct bhi360_fifo_parse_data_info *callback_info, uint32_t s, uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; D: ", callback_info->sensor_id, s, ns);

    for (uint8_t i = 0; i < (callback_info->data_size - 1); i++)
    {
        PRINT_D("%02X", callback_info->data_ptr[i]);
    }

    PRINT_D("\r\n");
}

/**
* @brief Function to stream and log for generic format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_generic(bool flag,
                                   const struct bhi360_fifo_parse_data_info *callback_info,
                                   uint32_t s,
                                   uint32_t ns,
                                   uint64_t tns,
                                   struct bhi360_parse_ref *parse_table,
                                   uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_generic(callback_info, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_generic(callback_info, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse meta event (wake-up and non-wake-up)
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint32_t s, ns;
    uint64_t tns;
    char *event_text;

    if (!callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

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

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    parse_meta_event_type(callback_info, event_text, s, ns, parse_table);
}

/**
* @brief Function to parse 3-axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_parse_xyz(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_3axis_s16(flag,
                                 callback_info,
                                 data,
                                 s,
                                 ns,
                                 tns,
                                 parse_table,
                                 parse_flag,
                                 sensor_details,
                                 scaling_factor);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_3axis_s16(flag,
                                 callback_info,
                                 data,
                                 s,
                                 ns,
                                 tns,
                                 parse_table,
                                 parse_flag,
                                 sensor_details,
                                 scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse euler format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_euler(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_orientation data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_parse_orientation(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_euler(flag,
                             callback_info,
                             data,
                             s,
                             ns,
                             tns,
                             parse_table,
                             parse_flag,
                             sensor_details,
                             scaling_factor);

        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_euler(flag,
                             callback_info,
                             data,
                             s,
                             ns,
                             tns,
                             parse_table,
                             parse_flag,
                             sensor_details,
                             scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse quaternion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_quaternion(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_quaternion data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_parse_quaternion(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_quaternion(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_quaternion(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 16-bit signed format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_s16_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    int16_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not define for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHI360_LE2S16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_s16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_s16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 32-bit scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u32(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    data = BHI360_LE2U32(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_scalar_u32(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_scalar_u32(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse scalar event format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_scalar_event(flag, callback_info, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_scalar_event(flag, callback_info, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse activity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_activity(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t activity;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    activity = BHI360_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_activity(flag, callback_info, activity, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_activity(flag, callback_info, activity, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 24-bit unsigned format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_u24_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHI360_LE2U24(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_u24_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_u24_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 8-bit unsigned scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u8(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    data = callback_info->data_ptr[0];
    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_scalar_u8(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_scalar_u8(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse generic format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_generic(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_generic(flag, callback_info, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_generic(flag, callback_info, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse debug message
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_debug_message(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;

    uint32_t s, ns;
    uint64_t tns;
    uint8_t msg_length;
    uint8_t debug_msg[MAX_DEBUG_MESSAGE_PAYLOAD_SIZE + 1] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero
                                                                    * if the payload is full */

    if (!callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    msg_length = callback_info->data_ptr[0];

    if (msg_length > MAX_DEBUG_MESSAGE_PAYLOAD_SIZE)
    {
        ERROR("Exceeded message length\r\n");

        return;
    }

    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */

    DATA("[DEBUG MSG]; T: %lu.%09lu; %s\r\n", s, ns, debug_msg);
}

/**
* @brief Function to print log for Multi-tap
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Multi-tap
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_multitap(const struct bhi360_fifo_parse_data_info *callback_info,
                               bhi360_event_data_multi_tap data,
                               uint32_t s,
                               uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; %s; \r\n",
         callback_info->sensor_id,
         s,
         ns,
         bhi360_event_data_multi_tap_string_out[data]);
}

/**
* @brief Function to stream and log for Multi-tap
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Multi-tap
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_multitap(bool flag,
                                    const struct bhi360_fifo_parse_data_info *callback_info,
                                    bhi360_event_data_multi_tap data,
                                    uint32_t s,
                                    uint32_t ns,
                                    uint64_t tns,
                                    struct bhi360_parse_ref *parse_table,
                                    uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_multitap(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_multitap(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Multi-tap
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_multitap(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;

    bhi360_event_data_multi_tap multitap_data = BHI360_NO_TAP;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    multitap_data = (bhi360_event_data_multi_tap)(*callback_info->data_ptr);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_multitap(flag, callback_info, multitap_data, s, ns, tns, parse_table, parse_flag);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_multitap(flag, callback_info, multitap_data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for Wrist Gesture Detector
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Wrist Gesture Detector
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_wrist_gesture_detect(const struct bhi360_fifo_parse_data_info *callback_info,
                                           bhi360_event_data_wrist_gesture_detect_t data,
                                           uint32_t s,
                                           uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; wrist_gesture: %s; \r\n",
         callback_info->sensor_id,
         s,
         ns,
         bhi360_event_data_wrist_gesture_detect_output[data.wrist_gesture]);
}

/**
* @brief Function to stream and log for Wrist Gesture Detector
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Wrist Gesture Detector
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_wrist_gesture_detect(bool flag,
                                                const struct bhi360_fifo_parse_data_info *callback_info,
                                                bhi360_event_data_wrist_gesture_detect_t data,
                                                uint32_t s,
                                                uint32_t ns,
                                                uint64_t tns,
                                                struct bhi360_parse_ref *parse_table,
                                                uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_wrist_gesture_detect(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_wrist_gesture_detect(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Wrist Gesture Detector
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_wrist_gesture_detect(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;

    bhi360_event_data_wrist_gesture_detect_t wrist_gesture_detect_data;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhi360_event_data_wrist_gesture_detect_parsing(callback_info->data_ptr, &wrist_gesture_detect_data);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_wrist_gesture_detect(flag,
                                            callback_info,
                                            wrist_gesture_detect_data,
                                            s,
                                            ns,
                                            tns,
                                            parse_table,
                                            parse_flag);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_wrist_gesture_detect(flag,
                                            callback_info,
                                            wrist_gesture_detect_data,
                                            s,
                                            ns,
                                            tns,
                                            parse_table,
                                            parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for hmc
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation euler
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_hmc(const struct bhi360_fifo_parse_data_info *callback_info,
                          bhi360_event_data_head_orientation_quat data,
                          uint32_t s,
                          uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f);
}

/**
* @brief Function to stream and log for hmc
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation hmc
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_hmc(bool flag,
                               const struct bhi360_fifo_parse_data_info *callback_info,
                               bhi360_event_data_head_orientation_quat data,
                               uint32_t s,
                               uint32_t ns,
                               uint64_t tns,
                               struct bhi360_parse_ref *parse_table,
                               uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_hmc(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_hmc(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Misalignment Calibration
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_hmc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_hmc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_hmc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for oc
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_oc(const struct bhi360_fifo_parse_data_info *callback_info,
                         bhi360_event_data_head_orientation_quat data,
                         uint32_t s,
                         uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f);
}

/**
* @brief Function to stream and log for oc
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation quaternion
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_oc(bool flag,
                              const struct bhi360_fifo_parse_data_info *callback_info,
                              bhi360_event_data_head_orientation_quat data,
                              uint32_t s,
                              uint32_t ns,
                              uint64_t tns,
                              struct bhi360_parse_ref *parse_table,
                              uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_oc(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_oc(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Orientation Quaternion
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_oc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_oc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_oc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for ec
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation euler
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_ec(const struct bhi360_fifo_parse_data_info *callback_info,
                         bhi360_event_data_head_orientation_eul data,
                         uint32_t s,
                         uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f\r\n",
         callback_info->sensor_id,
         s,
         ns,
         (data.heading * 360.0f) / 32768.0f,
         (data.pitch * 360.0f) / 32768.0f,
         (data.roll * 360.0f) / 32768.0f);
}

/**
* @brief Function to stream and log for ec
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation euler
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_ec(bool flag,
                              const struct bhi360_fifo_parse_data_info *callback_info,
                              bhi360_event_data_head_orientation_eul data,
                              uint32_t s,
                              uint32_t ns,
                              uint64_t tns,
                              struct bhi360_parse_ref *parse_table,
                              uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_ec(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_ec(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Orientation Euler
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_ec(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_eul data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        INFO("Parse slot not defined for %u\r\n", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_head_orientation_eul_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_ec(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_ec(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to set down sampling flag
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling value
*/
void bhi360_set_downsampling_flag(uint8_t sen_id, bool enable)
{
    enable_ds[sen_id] = enable;
}

/**
* @brief Function to get down sampling flag
* @param[in] sen_id  : Virtual sensor ID
* @return Down sampling value
*/
bool bhi360_get_downsampling_flag(uint8_t sen_id)
{
    return enable_ds[sen_id];
}

/**
* @brief Function to set down sampling ratio
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling ratio
*/
void bhi360_set_downsampling_odr(uint8_t sen_id, int16_t odr)
{
    odr_ds[sen_id] = odr;
}

/**
 * @brief Function to parse FIFO frame data into temperature
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] temperature  : Reference to the data buffer to store temperature in degree C
 */
void bhi360_parse_temperature_celsius(const uint8_t *data, bhi360_float *temperature)
{
    /* 1 LSB = 1/100 degC */
    float scale_factor = (float)1 / 100;

    *temperature = BHI360_LE2S16(data) * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into humidity
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] humidity : Reference to the data buffer to store humidity in %
 */
void bhi360_parse_humidity(const uint8_t *data, bhi360_float *humidity)
{
    float scale_factor = (float)1;

    *humidity = data[0] * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into barometric pressure
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] pressure : Reference to the data buffer to store pressure in Pascals
 */
void bhi360_parse_pressure(const uint8_t *data, bhi360_float *pressure)
{
    /* 1 LSB = 1/128 Pa */
    float scale_factor = (float)1 / 128;

    *pressure = (float)BHI360_LE2U24(data) * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into altitude
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] altitude : Reference to the data buffer to store altitude
 */
void bhi360_parse_altitude(const uint8_t *data, bhi360_float *altitude)
{
    *altitude = (float)(data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

void bhi360_parse_wrist_wear_wakeup_data(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;

    (void)callback_ref;

    if (callback_info->data_size != 2) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    /*lint -e10 Error 10: Lint does not understand PRIxxx */
    DATA("SID: %u; T: %lu.%09lu; Wake-up\r\n", callback_info->sensor_id, s, ns);

    /*lint +e10 */
}

void bhi360_parse_head_gesture_detector_data(const struct bhi360_fifo_parse_data_info *callback_info,
                                             void *callback_ref)
{
    (void)callback_ref;
    bhi360_event_data_head_gesture_detector hgd_data = BHI360_HGD_NO_GESTURE;
    uint32_t s, ns;
    uint64_t tns;

    if (callback_info->data_size != 2) /*! Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    hgd_data = (bhi360_event_data_head_gesture_detector)(*callback_info->data_ptr);

    DATA("SID: %u; T: %lu.%09lu; %s; \r\n", callback_info->sensor_id, s, ns,
         bhi360_event_data_head_gesture_string_out[hgd_data]);

}

void bhi360_parse_klio_handle_learnt_pattern(const struct bhi360_fifo_parse_data_info *callback_info,
                                             uint32_t s,
                                             uint32_t ns,
                                             struct bhi360_dev *bhy)
{
    uint8_t tmp_buf[PARAM_BUF_LEN];
    uint16_t bufsize = sizeof(tmp_buf);
    klio_info *p_klio_info = bhi360_get_klio_info();
    float similarity_result_buf[25]; /* Klio currently supports max 25 patterns */
    float highest_similarity_score = 0.f;

    /* Read out learnt pattern */
    (void)bhi360_klio_param_read_pattern(0, tmp_buf, &bufsize, bhy);

    DATA("SID: %u; T: %lu.%09lu; Pattern learnt: ", callback_info->sensor_id, s, ns);
    for (uint16_t i = 0; i < bufsize; i++)
    {
        PRINT_D("%02x", tmp_buf[i]);
    }

    PRINT_D("\r\n");

    /* Write back learnt pattern for recognition */
    if (p_klio_info->auto_load_pattern && p_klio_info->auto_load_pattern_write_index < p_klio_info->max_patterns)
    {
        uint32_t klio_driver_status = 0;
        bhi360_klio_param_sensor_state_t klio_sensor_state;

        /* Write pattern for recognition, note that this resets recognition statistics (and repetition counts) */
        (void)bhi360_klio_param_write_pattern(p_klio_info->auto_load_pattern_write_index, tmp_buf, bufsize, bhy);
        (void)bhi360_klio_param_read_reset_driver_status(&klio_driver_status, bhy);

        /* write klio state (enable recognition, and also make sure learning is not disabled) */
        klio_sensor_state.learning_enabled = 1;
        klio_sensor_state.learning_reset = 0;
        klio_sensor_state.recognition_enabled = 1;
        klio_sensor_state.recognition_reset = 0;
        (void)bhi360_klio_param_set_state(&klio_sensor_state, bhy);
        (void)bhi360_klio_param_read_reset_driver_status((uint32_t *)&klio_driver_status, bhy);

        if (p_klio_info->auto_load_pattern_write_index == 0)
        {
            /* This is the first pattern, unconditionally enable it for recognition. */
            (void)bhi360_klio_param_set_pattern_states(KLIO_PATTERN_STATE_ENABLE,
                                                       &p_klio_info->auto_load_pattern_write_index,
                                                       1,
                                                       bhy);
            p_klio_info->auto_load_pattern_write_index++;
        }
        else if (p_klio_info->auto_load_pattern_write_index > 0)
        {
            /* Compare current pattern against all previously stored ones */
            for (uint8_t i = 0; i < p_klio_info->auto_load_pattern_write_index; i++)
            {
                tmp_buf[i] = i;
            }

            PRINT_D("\n");

            (void)bhi360_klio_param_similarity_score_multiple(p_klio_info->auto_load_pattern_write_index,
                                                              tmp_buf,
                                                              p_klio_info->auto_load_pattern_write_index,
                                                              similarity_result_buf,
                                                              bhy);

            DATA("SID: %u; T: %lu.%09lu; Similarity score to already stored patterns: \n",
                 callback_info->sensor_id,
                 s,
                 ns);
            for (uint8_t i = 0; i < p_klio_info->auto_load_pattern_write_index; i++)
            {
                float tmp_score = similarity_result_buf[i];

                PRINT_D("%d: %f ", i, tmp_score);

                if (tmp_score > highest_similarity_score)
                {
                    highest_similarity_score = tmp_score;
                }
            }

            if (highest_similarity_score < 0.6f) /* Enable if initial pattern or dissimilar */
            {
                (void)bhi360_klio_param_set_pattern_states(KLIO_PATTERN_STATE_ENABLE,
                                                           &p_klio_info->auto_load_pattern_write_index,
                                                           1,
                                                           bhy);

                p_klio_info->auto_load_pattern_write_index++;
            }

            PRINT_D("\r\n");
        }
    }
}

void bhi360_parse_klio(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_dev *bhy = (struct bhi360_dev *)parse_table->bhy;
    bhi360_event_data_klio_t data;
    uint32_t s, ns;

    if (callback_info->data_size != sizeof(bhi360_event_data_klio_t) + 1) /* Check for a valid payload size. Includes
                                                                           * sensor ID */
    {
        return;
    }

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    memcpy(&data, callback_info->data_ptr, sizeof(data));

    /*lint -e10 Error 10: Lint does not understand PRIxxx */
    DATA(
        "SID: %u; T: %" PRIu32 ".%09" PRIu32 "; Learning [Id:%d Progress:%u Change:%u]; Recognition[Id:%d Count:%f Score:%f]\r\n",
        callback_info->sensor_id,
        s,
        ns,
        data.learn.index,
        data.learn.progress,
        data.learn.change_reason,
        data.recognize.index,
        data.recognize.count,
        data.recognize.score);

    /*lint +e10 */

    if (data.learn.index != -1) /* -1 means nothing was learnt. */
    {
        bhi360_parse_klio_handle_learnt_pattern(callback_info, s, ns, bhy);
    }
}

/**
* @brief Function to print log for log Klio
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for log Klio
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_klio_log(const struct bhi360_fifo_parse_data_info *callback_info,
                               bhi360_klio_param_log_frame_t data,
                               uint32_t s,
                               uint32_t ns)
{
    DATA("SID: %u; T: %lu.%09lu; ax: %.9g, ay: %.9g, az: %.9g, gx: %.9g, gy: %.9g, gz: %.9g\r\n",
         callback_info->sensor_id,
         s,
         ns,
         data.accel[0],
         data.accel[1],
         data.accel[2],
         data.gyro[0],
         data.gyro[1],
         data.gyro[2]);
}

/**
* @brief Function to stream and log for log Klio
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for log Klio
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_klio_log(bool flag,
                                    const struct bhi360_fifo_parse_data_info *callback_info,
                                    bhi360_klio_param_log_frame_t data,
                                    uint32_t s,
                                    uint32_t ns,
                                    uint64_t tns,
                                    struct bhi360_parse_ref *parse_table,
                                    uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_klio_log(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_klio_log(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(tns, callback_info->sensor_id, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse log Klio
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_klio_log(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    bhi360_klio_param_log_frame_t data;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        ERROR("Null reference\r\r\n");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        ERROR("Parse slot not defined\r\n");

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    memcpy(&data, callback_info->data_ptr, sizeof(data));

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_klio_log(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_klio_log(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to set Klio information (capabilities, state and runtime configuration)
* @param[in] klio_info : Klio information
*/
void bhi360_set_klio_info(const klio_info* info)
{
    k_info.max_patterns = info->max_patterns;
    k_info.max_pattern_blob_size = info->max_pattern_blob_size;
    k_info.auto_load_pattern_write_index = info->auto_load_pattern_write_index;
    k_info.auto_load_pattern = info->auto_load_pattern;
}

/**
* @brief Function to get Klio information (capabilities, state and runtime configuration)
 * @return Klio information
*/
klio_info* bhi360_get_klio_info(void)
{
    return &k_info;
}