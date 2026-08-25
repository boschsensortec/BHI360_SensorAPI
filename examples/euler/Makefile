COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= euler.c

BHI360_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

API_LOCATION ?= ../../source

COMMON_LOCATION ?= ..
LOCATION ?= FLASH

FW_LOCATION ?= ../..

C_SRCS += \
$(COMMON_LOCATION)/common/common.c \
$(COMMON_LOCATION)/common/verbose.c \
$(API_LOCATION)/bhi360.c \
$(API_LOCATION)/bhi360_hif.c \
$(API_LOCATION)/bhi360_virtual_sensor_info_param.c \
$(API_LOCATION)/bhi360_virtual_sensor_conf_param.c \
$(API_LOCATION)/bhi360_system_param.c \
$(API_LOCATION)/bhi360_logbin.c \
$(API_LOCATION)/bhi360_parse.c \
$(API_LOCATION)/bhi360_klio_param.c \
$(API_LOCATION)/bhi360_event_data.c

INCLUDEPATHS += . \
$(COMMON_LOCATION)/common \
$(API_LOCATION) \
$(FW_LOCATION)/firmware

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHI360_INTF), I2C)
CFLAGS += -DBHI360_USE_I2C
endif

include $(COINES_INSTALL_PATH)/coines.mk