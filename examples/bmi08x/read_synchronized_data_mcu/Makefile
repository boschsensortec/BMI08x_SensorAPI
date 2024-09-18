COINES_INSTALL_PATH ?= ../../../../..

EXAMPLE_FILE ?= read_synchronized_data_mcu.c

API_LOCATION ?= ../../..

C_SRCS += \
../common/common.c \
$(API_LOCATION)/bmi08a.c \
$(API_LOCATION)/bmi08g.c \
$(API_LOCATION)/bmi08xa.c

INCLUDEPATHS += \
$(API_LOCATION) \
../common

ifndef TARGET
$(error TARGET is not defined; please specify a target)
endif
INVALID_TARGET = PC
$(if $(filter $(TARGET),PC), $(error TARGET has an invalid value '$(TARGET)'; 'PC' is not a valid target. Please use a MCU target (eg. MCU_APP30).), )

include $(COINES_INSTALL_PATH)/coines.mk
