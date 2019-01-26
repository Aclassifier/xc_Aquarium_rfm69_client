# The TARGET variable determines what target system the application is
# compiled for. It either refers to an XN file in the source directories
# or a valid argument for the --target option when compiling
#
#TARGET = STARTKIT
TARGET = XCORE-200-EXPLORER
#         See http://www.xcore.com/viewtopic.php?f=26&t=6610 Passing TARGET to XC
# -------

# The APP_NAME variable determines the name of the final .xe file. It should
# not include the .xe postfix. If left blank the name will default to
# the project name
APP_NAME = _aquarium_rfm69_client

# The USED_MODULES variable lists other module used by the application.
USED_MODULES = lib_rfm69_xc lib_spi lib_i2c lib_xassert

# The flags passed to xcc when building the application
# You can also set the following to override flags for a particular language:
# XCC_XC_FLAGS, XCC_C_FLAGS, XCC_ASM_FLAGS, XCC_CPP_FLAGS
# If the variable XCC_MAP_FLAGS is set it overrides the flags passed to
# xcc for the final link (mapping) stage.
ifeq ($(TARGET),STARTKIT)
XCC_FLAGS  = -O2 -g -fxscope -save-temps -DMYTARGET=STARKIT -DISMASTER=1 -DWARNINGS=1 
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_USER=3 
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08=40
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_DEBUG_PRINT_GLOBAL=0
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_RADIO_IF_READALLREGS=0 
# -L/Users/teig/workspace/lib_rfm69_xc
# -I/Users/teig/workspace/lib_rfm69_xc/src -I/Users/teig/workspace/lib_rfm69_xc/api
# -v
else ifeq ($(TARGET),XCORE-200-EXPLORER)
XCC_FLAGS = -O2 -g -fxscope -save-temps -DMYTARGET=XCORE-200-EXPLORER -DISMASTER=0 -DWARNINGS=0
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_USER=3 
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08=40
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_DEBUG_PRINT_GLOBAL=1
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_DEBUG_PRINT_EXCEPTION=0
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_RADIO_IF_READALLREGS=0
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_GETDEBUG=1
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_GETDEBUG_TIMEOUT=0
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_GETDEBUG_BUTTON=1
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE=0
XCC_FLAGS += -D_USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_CRC_ERR_NO_IRQ=0
else
XCC_FLAGS = -O2 -g -fxscope -save-temps -DWARNINGS=0
endif
XCC_MAP_FLAGS = -Xmapper --map -Xmapper _app_rfm69_on_xmos_native.txt -report

# The XCORE_ARM_PROJECT variable, if set to 1, configures this
# project to create both xCORE and ARM binaries.
XCORE_ARM_PROJECT = 0

# The VERBOSE variable, if set to 1, enables verbose output from the make system.
VERBOSE = 0

XMOS_MAKE_PATH ?= ../..
-include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common
