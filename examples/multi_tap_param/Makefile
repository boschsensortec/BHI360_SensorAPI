#CCE_Board_Definitions:BHI360

#Change file name if required
LOG_FILE = "multi_tap_param_log.csv"

COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= multi_tap_param.c

BHI360_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

API_LOCATION ?= ../..

FW_LOCATION ?= ../..

COMMON_LOCATION ?= ..
LOCATION ?= FLASH

C_SRCS += \
$(COMMON_LOCATION)/common/common.c \
$(COMMON_LOCATION)/common/verbose.c \
$(API_LOCATION)/bhi360.c \
$(API_LOCATION)/bhi360_hif.c \
$(API_LOCATION)/bhi360_logbin.c \
$(API_LOCATION)/bhi360_parse.c \
$(API_LOCATION)/bhi360_event_data.c \
$(API_LOCATION)/bhi360_virtual_sensor_info_param.c \
$(API_LOCATION)/bhi360_virtual_sensor_conf_param.c \
$(API_LOCATION)/bhi360_system_param.c \
$(API_LOCATION)/bhi360_multi_tap_param.c

INCLUDEPATHS += . \
$(COMMON_LOCATION)/common \
$(API_LOCATION) \
$(FW_LOCATION)/firmware

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHI360_INTF), I2C)
CFLAGS += -DBHI360_USE_I2C
endif

include $(COINES_INSTALL_PATH)/coines.mk