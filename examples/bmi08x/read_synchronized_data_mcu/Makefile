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

include $(COINES_INSTALL_PATH)/coines.mk
