# Preserve the newlib bindings historically provided by the C++ wrapper.
include $(CHIBIOS)/os/various/newlib_bindings/newlib.mk

# C++ wrapper files.
CHCPPSRC   =

CHCPPSRCPP = $(CHIBIOS)/os/various/cpp_wrappers/ch.cpp

CHCPPINC   = $(CHIBIOS)/os/various/cpp_wrappers

# Shared variables
ALLCSRC   += $(CHCPPSRC)
ALLCPPSRC += $(CHCPPSRCPP)
ALLINC    += $(CHCPPINC)
