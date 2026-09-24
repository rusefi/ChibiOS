#!/bin/sh
openocd -f interface/cmsis-dap.cfg -f ./at32f4x.cfg \
	-c "init" \
	-c "halt" \
	-c "flash write_image erase build/ch.elf" \
	-c "flash verify_image build/ch.elf" \
	-c "reset" \
	-c "exit"
