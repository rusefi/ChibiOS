##############################################################################
# Multi-project makefile rules
#

all:
	@echo
	@echo === Building for STM32L476-Discovery ===============================
	+@make --no-print-directory -f ./make/stm32l476_discovery.make all
	+@make --no-print-directory -f ./make/at32f435_start.make all
	@echo ====================================================================
	@echo

clean:
	@echo
	+@make --no-print-directory -f ./make/stm32l476_discovery.make clean
	+@make --no-print-directory -f ./make/at32f435_start.make clean	
	@echo

#
##############################################################################
