# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/AT_START_F435/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/AT_START_F435

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
