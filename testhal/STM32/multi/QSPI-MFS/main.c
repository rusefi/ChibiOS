/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "m25q.h"
#include "mfs.h"

#include "mfs_test_root.h"

#include "portab.h"

/* 16MB device, 2 cycles delay after NCS.*/
const QSPIConfig qspicfg1 = {
  NULL,
  STM32_DCR_FSIZE(24) | STM32_DCR_CSHT(1)
};

qspi_command_t cmd_read_id = {
  QSPI_CFG_CMD(0x9E) | QSPI_CFG_CMD_MODE_ONE_LINE |
  QSPI_CFG_ADDR_MODE_NONE |
  QSPI_CFG_ALT_MODE_NONE |
  QSPI_CFG_DUMMY_CYCLES(0) |
  QSPI_CFG_DATA_MODE_ONE_LINE,
  0,
  0
};

/*
 * Generic buffer.
 */
uint8_t buffer[2048];

const uint8_t pattern[128] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};

M25QDriver m25q;

const M25QConfig m25qcfg1 = {
  &QSPID1,
  &qspicfg1
};

const MFSConfig mfscfg1 = {
  (BaseFlash *)&m25q,
  0xFFFFFFFFU,
  131072U,
  0,
  2,
  2,
  2
};
/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
    palToggleLine(PORTAB_LINE_LED1);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Board-dependent GPIO setup code.
   */
  portab_setup();

  /*
   * Starting a serial port for test report output.
   */
  sdStart(&PORTAB_SD1, NULL);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    if (palReadLine(PORTAB_LINE_BUTTON) == PORTAB_BUTTON_PRESSED) {
      test_execute((BaseSequentialStream *)&PORTAB_SD1, &mfs_test_suite);
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}