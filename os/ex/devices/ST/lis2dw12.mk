# List of all the LIS3DSH device files.
LIS2DW12SRC := $(CHIBIOS)/os/ex/devices/ST/lis2dw12.c

# Required include directories
LIS2DW12INC := $(CHIBIOS)/os/ex/include \
               $(CHIBIOS)/os/ex/devices/ST

# Shared variables
ALLCSRC += $(LIS2DW12SRC)
ALLINC  += $(LIS2DW12INC)