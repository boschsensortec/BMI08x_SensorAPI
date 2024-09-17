COINES_INSTALL_PATH ?= ../../../../..

EXAMPLE_FILE ?= interrupt_streaming_pc.c

API_LOCATION ?= ../../..

C_SRCS += \
$(API_LOCATION)/bmi08a.c \
$(API_LOCATION)/bmi08g.c \
$(API_LOCATION)/bmi08xa.c \
../common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
../common

ifndef TARGET
    $(error TARGET is not defined; please specify a target)
endif

INVALID_TARGET = MCU
INVALID_STR := $(findstring $(INVALID_TARGET), $(TARGET))
ifeq ($(INVALID_STR),$(INVALID_TARGET))
    $(error TARGET has an invalid value '$(TARGET)'; 'MCU' targets are not a valid. Please use PC as target.)
endif

include $(COINES_INSTALL_PATH)/coines.mk