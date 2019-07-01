mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS += \
   parameter.o \
    
APPL_CXXOBJS += \
    operation.o \
    yawrate_ctl.o \
    motor_ctl.o \
    line_tracer.o \
    util.o \
    recognition.o \
    judgment.o \


SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif

APPL_DIR += $(mkfile_path)app $(mkfile_path)Utility $(mkfile_path)Recognition $(mkfile_path)Judgment $(mkfile_path)Operation

INCLUDES += \
        -I$(mkfile_path)app \
        -I$(mkfile_path)Utility \
        -I$(mkfile_path)Recognition \
        -I$(mkfile_path)Judgment \
        -I$(mkfile_path)Operation \

# COPTS += -fno-use-cxa-atexit
