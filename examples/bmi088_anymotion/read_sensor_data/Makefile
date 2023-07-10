COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE = read_sensor_data.c

API_LOCATION ?= ../..

C_SRCS += \
$(API_LOCATION)/bmi08a.c \
$(API_LOCATION)/bmi08g.c \
$(API_LOCATION)/bmi088_anymotiona.c \
../common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
../common

TARGET ?= PC

include $(COINES_INSTALL_PATH)/coines.mk
