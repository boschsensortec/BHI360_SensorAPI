# BHI360 SensorAPI — User Integration Guide

## Table of Contents

- [BHI360 SensorAPI — User Integration Guide](#bhi360-sensorapi--user-integration-guide)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Repository Structure](#repository-structure)
  - [Files Required for Integration](#files-required-for-integration)
    - [Mandatory (Core API)](#mandatory-core-api)
    - [Optional (Reference Only)](#optional-reference-only)
  - [Step-by-Step Integration](#step-by-step-integration)
    - [Step 1: Copy Core Source Files](#step-1-copy-core-source-files)
    - [Step 2: Add Include Path](#step-2-add-include-path)
    - [Step 3: Implement Platform-Specific Callbacks](#step-3-implement-platform-specific-callbacks)
  - [Platform Porting Guide](#platform-porting-guide)
    - [1. Bus Read](#1-bus-read)
    - [2. Bus Write](#2-bus-write)
    - [3. Delay (Microseconds)](#3-delay-microseconds)
    - [Interface Pointer (`intf_ptr`)](#interface-pointer-intf_ptr)
  - [Firmware Upload](#firmware-upload)
  - [API Usage Flow](#api-usage-flow)
  - [Error Handling](#error-handling)
  - [Interface Configuration](#interface-configuration)
    - [I2C](#i2c)
    - [SPI](#spi)
  - [Build Integration](#build-integration)
    - [Clone BHI360 SensorAPI](#clone-bhi360-sensorapi)
	- [Examples Build and Execution with PC as host](#examples-build-and-execution-with-pc-as-host)
	- [Examples Build and Execution with MCU as host](#examples-build-and-execution-with-mcu-as-host)
	
---

## Overview

The BHI360 belongs to the BHI family of highly integrated, ultra-low power programmable smart sensor systems. The BHI360 integrates the Fuser2 processor, which is based on the 32-Bit ARC™ EM4™ floating point RISC processor, an integrated Inertial Measurement Unit (6DoF IMU) and a powerful Event-Driven Software Framework specifically designed for signal data processing and comes with pre-installed sensor fusion and other sensor data processing algorithms.

**Key characteristics:**
- Written in ANSI C (C99)
- No dynamic memory allocation
- No platform-specific dependencies in core source
- Supports SPI (4-wire) and I2C interfaces
- Linux kernel compatible (`__KERNEL__` guard)
- Firmware-driven: sensor algorithms run on-device; host receives data from a FIFO
- Supports virtual sensor callbacks for flexible event-driven data processing
- BSD-3-Clause license

---

## Repository Structure

```
BHI360_SensorAPI/
├── source/                                       <-- Core Sensor API (REQUIRED)
│   ├── bhi360_defs.h                             <-- Macros, register map, error codes, type definitions
│   ├── bhi360.h / bhi360.c                       <-- Core API: init, firmware upload, FIFO processing
│   ├── bhi360_hif.h / bhi360_hif.c               <-- Low-level host interface (HIF) register access
│   ├── bhi360_parse.h / bhi360_parse.c           <-- Virtual sensor output data parsers
│   ├── bhi360_event_data.h / bhi360_event_data.c <-- Event and status data types
│   ├── bhi360_system_param.h / bhi360_system_param.c
│   ├── bhi360_virtual_sensor_conf_param.h/.c
│   ├── bhi360_virtual_sensor_info_param.h/.c
│   ├── bhi360_phy_sensor_ctrl_param.h/.c
│   ├── bhi360_head_feature_param.h/.c
│   ├── bhi360_bsx_algo_param.h/.c
│   ├── bhi360_activity_param.h/.c
│   ├── bhi360_klio_param.h/.c
│   ├── bhi360_multi_tap_param.h/.c
|   ├── bhi360_generic_features_param.h/.c
│   ├── bhi360_logbin.h / bhi360_logbin.c
│   ├── bhi360_param_defs.h
|   ├── bhi360_api_entry.h / bhi360_api_entry.c
│   └── ...
│
├── firmware/bhi360/                              <-- Pre-built firmware images
│   ├── Bosch_Shuttle3_BHI360_bsxsam.fw           <-- BSX sensor fusion (accel + gyro)
│   ├── Bosch_Shuttle3_BHI360_bsxsam.fw.h         <-- C header form for direct inclusion
│   └── ...                                       <-- Additional firmware variants (see Firmware Upload)
│
├── examples/                                     <-- Reference examples (for guidance)
│   ├── common/                                   <-- Platform abstraction layer (COINES-based reference)
│   │   ├── common.h / common.c
│   │   └── verbose.h / verbose.c
│   ├── load_firmware/                            <-- Minimal: upload firmware and boot
│   ├── activity/                                 <-- Activity recognition
│   ├── euler/                                    <-- Euler angle output
|   ├── head_gesture_detection/                   <-- Head gesture detection
│   ├── quaternion/                               <-- Rotation quaternion output
│   ├── step_counter/                             <-- Step counter / detector
│   ├── klio/                                     <-- Klio pattern recognition
│   └── ...                                       <-- Further use-case examples
│
└── LICENSE                                       <-- BSD-3-Clause license text
```

---

## Files Required for Integration

### Mandatory (Core API)

Copy these files into your project. They have **no external dependencies** beyond the C standard library.

| File | Description |
|------|-------------|
| `source/bhi360_defs.h` | All macros, register addresses, error codes, and type definitions |
| `source/bhi360.h` / `source/bhi360.c` | Core API: `bhi360_init()`, firmware upload, FIFO processing |
| `source/bhi360_hif.h` / `source/bhi360_hif.c` | Host interface low-level register access |
| `source/bhi360_parse.h` / `source/bhi360_parse.c` | Virtual sensor data parsers |
| `source/bhi360_event_data.h` / `source/bhi360_event_data.c` | Event/status data types and parsers |
| `source/bhi360_param_defs.h` | Shared parameter type definitions |

### Optional (Reference Only)

Include additional source files only for the features your application uses. **Do NOT copy example platform files directly** — use them as reference to write your own platform layer.

| File | Purpose |
|------|---------|
| `source/bhi360_system_param.h/.c` | Read/write system parameters |
| `source/bhi360_head_feature_param.h/.c` | Read/write head feature parameters |
| `source/bhi360_generic_features_param.h/.c` | Read/write generic features parameters |
| `source/bhi360_virtual_sensor_conf_param.h/.c` | Virtual sensor configuration parameters |
| `source/bhi360_virtual_sensor_info_param.h/.c` | Query virtual sensor information |
| `source/bhi360_phy_sensor_ctrl_param.h/.c` | Physical sensor control parameters |
| `source/bhi360_bsx_algo_param.h/.c` | BSX algorithm parameter tuning |
| `source/bhi360_activity_param.h/.c` | Activity recognition parameters |
| `source/bhi360_klio_param.h/.c` | Klio repetitive motion learning/recognition |
| `source/bhi360_multi_tap_param.h/.c` | Multi-tap detection parameters |
| `source/bhi360_logbin.h/.c` | Binary logging support |
| `source/bhi360_api_entry.h/.c` | API entry point utilities |
| `examples/common/common.c` | Reference bus read/write/delay using COINES |
| `examples/load_firmware/load_firmware.c` | Example: firmware upload and FIFO processing |

---

## Step-by-Step Integration

### Step 1: Copy Core Source Files

Copy the mandatory files into your project:

```
your_project/
├── bhi360/
│   ├── bhi360_defs.h
│   ├── bhi360.h
│   ├── bhi360.c
│   ├── bhi360_hif.h
│   ├── bhi360_hif.c
│   ├── bhi360_parse.h
│   ├── bhi360_parse.c
│   ├── bhi360_event_data.h
│   ├── bhi360_event_data.c
│   └── bhi360_param_defs.h
├── your_platform/
│   ├── bhi360_platform.h        <-- You create this
│   └── bhi360_platform.c        <-- You create this
├── firmware/
│   └── Bosch_Shuttle3_BHI360_bsxsam_lite.fw.h  <-- Include the firmware of your choice
└── main.c
```

### Step 2: Add Include Path

Add the `bhi360/` folder to your compiler's include path:

```
# GCC example
CFLAGS += -Ibhi360/

```

### Step 3: Implement Platform-Specific Callbacks

You must implement **3 callback functions** and pass them to `bhi360_init()`. See [Platform Porting Guide](#platform-porting-guide) below.

---

## Platform Porting Guide

You need to implement 3 functions matching these exact signatures (defined in `bhi360_defs.h`):

### 1. Bus Read

```c
int8_t my_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    /* TODO: Implement your platform's SPI/I2C read here
     * - Assert chip-select (SPI) or start I2C transaction
     * - Send reg_addr to sensor
     * - Read 'length' bytes into reg_data buffer
     * - Deassert chip-select / stop I2C transaction
     */
    return 0;  /* 0 = success, non-zero = failure */
}
```

### 2. Bus Write

```c
int8_t my_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    /* TODO: Implement your platform's SPI/I2C write here
     * - Assert chip-select (SPI) or start I2C transaction
     * - Send reg_addr followed by 'length' bytes from reg_data
     * - Deassert chip-select / stop I2C transaction
     */
    return 0;  /* 0 = success, non-zero = failure */
}
```

### 3. Delay (Microseconds)

```c
void my_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;

    /* TODO: Implement platform-specific microsecond delay
     * Examples:
     *   - HAL_Delay(period_us / 1000) for STM32 (ms granularity)
     *   - DWT cycle counter for precise us delay on Cortex-M
     *   - usleep(period_us) for Linux userspace
     */
}
```

### Interface Pointer (`intf_ptr`)

The `intf_ptr` is a user-defined pointer passed unchanged to every callback. Typical uses:

| Interface | Typical `intf_ptr` content |
|-----------|---------------------------|
| I2C | Pointer to `uint8_t` holding the I2C slave address (`0x28`) |
| SPI | Pointer to chip-select GPIO pin descriptor, or `NULL` if managed in the callback |

---

## Firmware Upload

The BHI360 requires firmware to be loaded before any virtual sensors can be configured or data read. Pre-built firmware images are provided in the `firmware/bhi360/` directory as both binary (`.fw`) and C header (`.fw.h`) files.

**Selecting the right firmware:**

| Firmware variant | Use when... |
|------------------|-------------|
| `bsxsam_lite` | Accel + gyro only; smallest footprint |
| `bsxsam` | Accel + gyro with full BSX sensor fusion |
| `bsxsam_turbo` | High-ODR (up to 800 Hz) BSX fusion |
| `bsxsam_lite_Klio` | Accel + gyro + Klio pattern recognition |
| `*_BMM350*_ndof` | 9-DoF NDOF fusion (requires external BMM350 magnetometer) |
| `*_BMP58X*` | Adds pressure/altitude from external BMP58X |
| `*_BME688*` | Adds environmental sensing from external BME688 |

**Uploading from a C header (recommended for embedded):**

```c
/* Include the chosen firmware as a C array */
#include "firmware/bhi360/Bosch_Shuttle3_BHI360_bsxsam_lite.fw.h"

int8_t rslt = bhi360_upload_firmware_to_ram(bhi360_firmware_image,
                                             sizeof(bhi360_firmware_image),
                                             &dev);
```

> **Note:** The symbol name `bhi360_firmware_image` and array size are defined inside each `.fw.h` file.

**Uploading in chunks (for constrained hosts):**

Use `bhi360_upload_firmware_to_ram_partly()` to upload the firmware in smaller segments if your system cannot hold the entire image in RAM at once.

---

## API Usage Flow

Below is the recommended sequence for initializing and reading data from the BHI360:

```
┌─────────────────────────────────┐
│  1. Assign platform callbacks   │
│     (read, write, delay_us)     │
└──────────────┬──────────────────┘
               ▼
┌─────────────────────────────────┐
│  2. bhi360_init()               │
│     Links callbacks, validates  │
│     chip ID (0x7C)              │
└──────────────┬──────────────────┘
               ▼
┌─────────────────────────────────┐
│  3. bhi360_upload_firmware_     │
│     to_ram()                    │
│     Transfers firmware image    │
│     to device RAM               │
└──────────────┬──────────────────┘
               ▼
┌─────────────────────────────────┐
│  4. bhi360_boot_from_ram()      │
│     Starts the on-device MCU;   │
│     sensor algorithms active    │
└──────────────┬──────────────────┘
               ▼
┌─────────────────────────────────┐
│  5. bhi360_register_fifo_parse_ │
│     callback()                  │
│     Register per-sensor data    │
│     callbacks                   │
└──────────────┬──────────────────┘
               ▼
┌─────────────────────────────────┐
│  6. bhi360_set_virt_sensor_cfg()│
│     Enable sensors at desired   │
│     sample rate                 │
└──────────────┬──────────────────┘
               ▼
┌─────────────────────────────────┐
│  7. Application loop            │
│     bhi360_get_and_process_     │
│     fifo()                      │
│     → dispatches callbacks      │
│       with sensor data          │
└─────────────────────────────────┘
```

---

## Error Handling

All API functions return `int8_t` status codes:

| Code | Macro | Meaning |
|------|-------|---------|
| `0` | `BHI360_OK` | Success |
| `-1` | `BHI360_E_NULL_PTR` | Null pointer passed to API |
| `-2` | `BHI360_E_INVALID_PARAM` | Invalid parameter value |
| `-3` | `BHI360_E_IO` | Bus read/write communication failure |
| `-4` | `BHI360_E_MAGIC` | Firmware image magic number mismatch |
| `-5` | `BHI360_E_TIMEOUT` | Operation timed out |
| `-6` | `BHI360_E_BUFFER` | Invalid or insufficient buffer |
| `-7` | `BHI360_E_INVALID_FIFO_TYPE` | Unknown or unsupported FIFO type |
| `-8` | `BHI360_E_INVALID_EVENT_SIZE` | FIFO event size mismatch |
| `-9` | `BHI360_E_PARAM_NOT_SET` | Required parameter has not been configured |
| `-10` | `BHI360_E_INSUFFICIENT_MAX_SIMUL_SENSORS` | Too many simultaneous sensors requested |
| `-14` | `BHI360_E_INVALID_DATA` | Invalid data received from device |

**Best practice:** Always check the return value of every API call:

```c
rslt = bhi360_init(BHI360_SPI_INTERFACE, my_read, my_write, my_delay_us, 256, NULL, &dev);
if (rslt != BHI360_OK)
{
    /* Handle error — do not proceed */
    printf("Init failed with error: %d\n", rslt);
    return rslt;
}
```

---

## Interface Configuration

### I2C

| Parameter | Value |
|-----------|-------|
| Default slave address | `0x28` |
| Supported speed | Standard (100 kHz), Fast (400 kHz) |
| `intf` enum value | `BHI360_I2C_INTERFACE` |

### SPI

| Parameter | Value |
|-----------|-------|
| SPI mode | Mode 0 (CPOL=0, CPHA=0) |
| Bus width | 4-wire (full duplex) |
| Max clock speed | Refer to BHI360 datasheet |
| `intf` enum value | `BHI360_SPI_INTERFACE` |

> **Note:** The SPI read/write protocol is handled internally by the API. Your bus read/write callbacks should pass `reg_addr` as received without applying any read/write mask.

---

## Build Integration


### Clone BHI360 SensorAPI

To clone BHI360 SensorAPI from Github, please follow the below steps

```
1. Open your Git Bash on your local machine.
2. To clone BHI360 SensorAPI with coines from Github use the below command and press enter key
      git clone --recurse-submodules https://github.com/boschsensortec/BHI360_SensorAPI.git

```

**Note**

Ensure you have the necessary access rights to the repository.
If you encounter any errors related to repository access, please verify your SSH keys and user permissions.


### Examples Build and Execution with PC as host

To build and execute with PC as host, please follow the below steps

```
1. Connect the APP3.0 or APP3.1 board with BHI360 shuttle to PC via USB cable.
2. Open the command prompt and select the respective examples location like "../../examples/self_test"
3. Type the command build.bat to build the code and press enter key
4. Run the respective self_test.exe
5. Verify the results

```

### Examples Build and Execution with MCU as host

To build and execute with MCU as host, please follow the below steps

```
1. Connect the APP3.0 or APP3.1 board with BHI360 shuttle to PC via USB cable.
2. Open the command prompt and select the respective examples location like "../../examples/self_test"
3. Type the command build_app31.bat to build the code and press enter key
4. Type the command download_app31.bat and press enter key
5. Connect with port as per selection in hterm to verify the results

```

**Note**

```
1. You can modify I2C/SPI interface in Makefile
2. Type the command clean.bat in PC as host to clean the code and press enter key
3. Type the command clean_app31.bat in MCU as host to clean the code and press enter key

```

### Link to COINES-SDK
https://github.com/boschsensortec/COINES_SDK/


*For further details, refer to the BHI360 datasheet and the example source files in the `examples/` directory.*

---

#### Copyright (C) 2026 Bosch Sensortec GmbH. All rights reserved.