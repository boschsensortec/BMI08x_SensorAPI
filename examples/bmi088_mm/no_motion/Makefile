COINES_INSTALL_PATH ?= ../../../../..

EXAMPLE_FILE = no_motion.c

API_LOCATION ?= ../../..

C_SRCS += \
$(API_LOCATION)/bmi08a.c \
$(API_LOCATION)/bmi08g.c \
$(API_LOCATION)/bmi088_mma.c \
../common/common.c

INCLUDEPATHS += \
$(API_LOCATION) \
../common

COINES_BACKEND ?= COINES_BRIDGE

include $(COINES_INSTALL_PATH)/coines.mk