target extended-remote localhost:3333
file build/ch.elf
#file build-openblt/openblt_microrusefi.elf
load
set breakpoint auto-hw on
set remote hardware-breakpoint-limit 8
set remote hardware-watchpoint-limit 4
