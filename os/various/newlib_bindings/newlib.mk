# Both applications and chcpp.mk may include this module.
ifndef NEWLIBSSRC
# NewLib files.
NEWLIBSSRC = $(CHIBIOS)/os/various/newlib_bindings/syscalls.c

NEWLIBINC  = $(CHIBIOS)/os/various/newlib_bindings

# Shared variables
ALLCSRC += $(NEWLIBSSRC)
ALLINC  += $(NEWLIBINC)

endif
