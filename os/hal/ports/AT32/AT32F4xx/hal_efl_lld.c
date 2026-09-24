/*
    ChibiOS - Copyright (C) 2023 Andrey Gusakov

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

/**
 * @file    hal_efl_lld.c
 * @brief   AT32F43X Embedded Flash subsystem low level driver source.
 *
 * @addtogroup HAL_EFL
 * @{
 */

#include <string.h>

#include "hal.h"

#if (HAL_USE_EFL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* The Flash memory can be programmed with 32 bits, 16 bits or 8 bits at a time. */
#define AT32_FLASH_LINE_SIZE                (4)
#define AT32_FLASH_LINE_MASK                (AT32_FLASH_LINE_SIZE - 1U)

#define FLASH_PDKEY1                        0x04152637U
#define FLASH_PDKEY2                        0xFAFBFCFDU

#define FLASH_KEY1                          0x45670123U
#define FLASH_KEY2                          0xCDEF89ABU

#define FLASH_OPTKEY1                       0x08192A3BU
#define FLASH_OPTKEY2                       0x4C5D6E7FU

#if !defined(FLASH_STS_OPERR)
#define FLASH_STS_OPERR                      FLASH_STS_SOP
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   EFL1 driver identifier - first bank.
 */
EFlashDriver EFLD1;

/**
 * @brief   EFL2 driver identifier - second bank.
 */
EFlashDriver EFLD2;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* TODO: add and use Artery defines */
#if defined(STM32F429_439xx) || defined(STM32F427_437xx) || \
    defined(__DOXYGEN__)

/* The descriptors for 4032K device. */
static const flash_descriptor_t efl_lld_size_4032k[AT32_FLASH_NUMBER_OF_BANKS] = {
  { /* Bank 1 organisation. */
   .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                        FLASH_ATTR_MEMORY_MAPPED,
   .page_size         = AT32_FLASH_LINE_SIZE,
   .sectors_count     = 32 * 16,    /* 32 blocks with 16 sectors in each */
   .sectors           = NULL,
   .sectors_size      = 4096,
   .address           = (uint8_t *)FLASH_BASE,
   .size              = 32 * 16 * 4096,
  },
  { /* Bank 2 organisation. */
   .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                        FLASH_ATTR_MEMORY_MAPPED,
   .page_size         = AT32_FLASH_LINE_SIZE,
   .sectors_count     = 31 * 16,    /* 31 blocks with 16 sectors in each */
   .sectors           = NULL,
   .sectors_size      = 4096,
   .address           = (uint8_t *)(FLASH_BASE + 32 * 16 * 4096),
   .size              = 31 * 16 * 4096,
  }
};

/* The descriptors for 1M device. */
static const flash_descriptor_t efl_lld_size_1m[AT32_FLASH_NUMBER_OF_BANKS] = {
  { /* Bank 1 organisation. */
   .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                        FLASH_ATTR_MEMORY_MAPPED,
   .page_size         = AT32_FLASH_LINE_SIZE,
   .sectors_count     = 8 * 32,    /* 8 blocks with 32 sectors in each */
   .sectors           = NULL,
   .sectors_size      = 2048,
   .address           = (uint8_t *)FLASH_BASE,
   .size              = 8 * 32 * 2048
  },
  { /* Bank 2 organisation. */
   .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                        FLASH_ATTR_MEMORY_MAPPED,
   .page_size         = AT32_FLASH_LINE_SIZE,
   .sectors_count     = 8 * 32,    /* 8 blocks with 32 sectors in each */
   .sectors           = NULL,
   .sectors_size      = 2048,
   .address           = (uint8_t *)(FLASH_BASE + 8 * 32 * 2048),
   .size              = 8 * 32 * 2048
  }
};

/* The descriptors for 448K device. */
static const flash_descriptor_t efl_lld_size_448k[AT32_FLASH_NUMBER_OF_BANKS] = {
  { /* Bank 1 organisation. */
   .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                        FLASH_ATTR_MEMORY_MAPPED,
   .page_size         = AT32_FLASH_LINE_SIZE,
   .sectors_count     = 7 * 16,    /* 7 blocks with 16 sectors in each */
   .sectors           = NULL,
   .sectors_size      = 4096,
   .address           = (uint8_t *)FLASH_BASE,
   .size              = 7 * 16 * 4096,
  },
  { /* Single bank */
   .size              = 0,
  }
};

/* The descriptors for 256K device. */
static const flash_descriptor_t efl_lld_size_256k[AT32_FLASH_NUMBER_OF_BANKS] = {
  { /* Bank 1 organisation. */
   .attributes        = FLASH_ATTR_ERASED_IS_ONE |
                        FLASH_ATTR_MEMORY_MAPPED,
   .page_size         = AT32_FLASH_LINE_SIZE,
   .sectors_count     = 4 * 32,    /* 7 blocks with 16 sectors in each */
   .sectors           = NULL,
   .sectors_size      = 2048,
   .address           = (uint8_t *)FLASH_BASE,
   .size              = 4 * 32 * 2048
  },
  { /* Single bank */
   .size              = 0,
  }
};
/* Table describing possible flash sizes and descriptors for this device. */
static const efl_lld_size_t efl_lld_flash_sizes[] = {
  {
   .desc = efl_lld_size_4032k
  },
  {
   .desc = efl_lld_size_1m
  },
  {
   .desc = efl_lld_size_448k
  },
  {
   .desc = efl_lld_size_256k
  }
};
#else
#error "This EFL driver does not support the selected device"
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void at32_flash_lock(EFlashDriver *eflp) {

  eflp->flash->CTRL |= FLASH_CTRL_LOCK;
}

static inline void at32_flash_unlock(EFlashDriver *eflp) {

  eflp->flash->KEYR |= FLASH_KEY1;
  eflp->flash->KEYR |= FLASH_KEY2;
}

static inline void at32_flash_enable_pgm(EFlashDriver *eflp) {

  /* Enable programming. */
  eflp->flash->CTRL |= FLASH_CTRL_PRGM;
}

static inline void at32_flash_disable_pgm(EFlashDriver *eflp) {

  eflp->flash->CTRL &= ~FLASH_CTRL_PRGM;
}

static inline void at32_flash_clear_status(EFlashDriver *eflp) {

  eflp->flash->STS =  (FLASH_STS_PRGMERR | FLASH_STS_EPPERR | FLASH_STS_ODF);
}

static inline void at32_flash_wait_busy(EFlashDriver *eflp) {

  /* Wait for busy bit clear.*/
  while ((eflp->flash->STS & FLASH_STS_OBF) != 0U) {
  }
}

static inline size_t at32_flash_get_size(void) {
  return (*(uint32_t*)(AT32_FLASH_SIZE_REGISTER)) * AT32_FLASH_SIZE_SCALE;
}

static inline flash_error_t at32_flash_check_errors(EFlashDriver *eflp) {
  uint32_t sr = eflp->flash->STS;

  /* Clearing error conditions.*/
  eflp->flash->STS = sr & (FLASH_STS_PRGMERR | FLASH_STS_EPPERR);

  /* Some errors are only caught by assertion.*/
  osalDbgAssert((sr & 0) == 0U, "unexpected flash error");

  /* Decoding relevant errors.*/
  if ((sr & FLASH_STS_PRGMERR) != 0U) {
    return FLASH_ERROR_HW_FAILURE;
  }

  if ((sr & FLASH_STS_EPPERR) != 0U) {
    return eflp->state == FLASH_PGM ? FLASH_ERROR_PROGRAM : FLASH_ERROR_ERASE;
  }

  return FLASH_NO_ERROR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level Embedded Flash driver initialization.
 *
 * @notapi
 */
void efl_lld_init(void) {

  /* Driver initialization.*/
  eflObjectInit(&EFLD1);
  eflObjectInit(&EFLD2);
  EFLD1.flash = FLASH1;
  /* Find the size of the flash and set descriptor reference. */
  uint8_t i;
  for (i = 0; i < (sizeof(efl_lld_flash_sizes) / sizeof(efl_lld_size_t)); i++) {
    if (efl_lld_flash_sizes[i].desc[0].size + efl_lld_flash_sizes[i].desc[1].size ==
        at32_flash_get_size()) {
      EFLD1.descriptor = &efl_lld_flash_sizes[i].desc[0];
      if (efl_lld_flash_sizes[i].desc[1].size) {
        /* We have second bank! */
        EFLD2.descriptor = &efl_lld_flash_sizes[i].desc[1];
        EFLD2.flash = FLASH2;
      } else {
        EFLD2.descriptor = NULL;
        EFLD2.flash = NULL;
      }
      return;
    }
  }
  osalDbgAssert(false, "invalid flash configuration");
}

/**
 * @brief   Configures and activates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_start(EFlashDriver *eflp) {
  at32_flash_unlock(eflp);
  eflp->flash->CTRL = 0x00000000U;
}

/**
 * @brief   Deactivates the Embedded Flash peripheral.
 *
 * @param[in] eflp      pointer to a @p EFlashDriver structure
 *
 * @notapi
 */
void efl_lld_stop(EFlashDriver *eflp) {

  at32_flash_lock(eflp);
}

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          A flash device descriptor.
 * @retval                          Pointer to single bank if DBM not enabled.
 * @retval                          Pointer to bank1 if DBM enabled.
 *
 * @notapi
 */
const flash_descriptor_t *efl_lld_get_descriptor(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  return devp->descriptor;
}

/**
 * @brief   Read operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be read
 * @param[out] rp                   pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_READ         if the read operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_read(void *instance, flash_offset_t offset,
                           size_t n, uint8_t *rp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));

  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  osalDbgCheck(bank != NULL);
  osalDbgCheck((size_t)offset + n <= (size_t)bank->size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No reading while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Clearing error status bits.*/
  at32_flash_clear_status(devp);

  /* Actual read implementation.*/
  memcpy((void *)rp, (const void *)efl_lld_get_descriptor(instance)->address
                                   + offset, n);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;

}

/**
 * @brief   Program operation.
 * @note    Successive write operations are possible without the need of
 *          an erase when changing bits from one to zero. Writing one requires
 *          an erase operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] offset                offset within full flash address space
 * @param[in] n                     number of bytes to be programmed
 * @param[in] pp                    pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM      if the program operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_program(void *instance, flash_offset_t offset,
                              size_t n, const uint8_t *pp) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  bank = efl_lld_get_descriptor(instance);
  osalDbgCheck(bank != NULL);
  osalDbgCheck((size_t)offset + n <= (size_t)bank->size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No programming while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_PGM;

  /* Clearing error status bits.*/
  at32_flash_clear_status(devp);

  /* Enabling PGM mode in the controller.*/
  at32_flash_enable_pgm(devp);

  /* Actual program implementation.*/
  while (n > 0U) {
    volatile uint32_t *address;

    /* Create an array of sufficient size to hold line(s). */
    union {
      uint32_t  w[AT32_FLASH_LINE_SIZE / sizeof(uint32_t)];
      uint16_t  h[AT32_FLASH_LINE_SIZE / sizeof(uint16_t)];
      uint8_t   b[AT32_FLASH_LINE_SIZE / sizeof(uint8_t)];
    } line;

    /* Unwritten bytes are initialized to all ones.*/
    uint8_t i;
    for (i = 0; i < bank->page_size; i++) {
      line.b[i] = 0xFF;
    }

    /* Programming address aligned to flash lines.*/
    address = (volatile uint32_t *)(bank->address +
                                    (offset & ~AT32_FLASH_LINE_MASK));

    /* Copying data inside the prepared line(s).*/
    do {
      line.b[offset & AT32_FLASH_LINE_MASK] = *pp;
      offset++;
      n--;
      pp++;
    }
    while ((n > 0U) & ((offset & AT32_FLASH_LINE_MASK) != 0U));

    /* Programming line according to parallelism.*/
    switch (AT32_FLASH_LINE_SIZE) {
    case 1:
      address[0] = line.b[0];
      break;

    case 2:
      address[0] = line.h[0];
      break;

    case 4:
      address[0] = line.w[0];
      break;

    case 8:
      address[0] = line.w[0];
      address[1] = line.w[1];
      break;

    default:
      osalDbgAssert(false, "invalid line size");
      break;
    }

    at32_flash_wait_busy(devp);
    err = at32_flash_check_errors(devp);
    if (err != FLASH_NO_ERROR) {
      break;
    }
  }

  /* Disabling PGM mode in the controller.*/
  at32_flash_disable_pgm(devp);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

/**
 * @brief   Starts a whole-device erase operation.
 * @note    This function only erases the unused bank if in dual bank mode. The
 *          currently in use bank is not allowed since it is normally where the
 *          currently running program is executing from.
 *          Sectors on the in-use bank can be individually erased.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_all(void *instance) {
  EFlashDriver *devp = (EFlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  /* Clearing error status bits.*/
  at32_flash_clear_status(devp);

  /* Bank erase */
  devp->flash->CTRL |= FLASH_CTRL_BANKERS;
  /* Start */
  devp->flash->CTRL |= FLASH_CTRL_ERSTR;

  return FLASH_NO_ERROR;
}

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be erased
 *                                  this is an index within the total sectors
 *                                  in a flash bank
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_start_erase_sector(void *instance,
                                         flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  const flash_descriptor_t *bank = efl_lld_get_descriptor(instance);
  osalDbgCheck(bank != NULL);
  osalDbgCheck(sector < bank->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No erasing while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_PGM state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  /* Clearing error status bits.*/
  at32_flash_clear_status(devp);

  /* Write the sector to be erased in the FLASH_ADDRx register; */
  devp->flash->ADDR = (flash_offset_t)bank->address +
                      flashGetSectorOffset(getBaseFlash(devp), sector);

  /* Enable sector erase.*/
  devp->flash->CTRL |= FLASH_CTRL_SECERS;

  /* Start the erase.*/
  devp->flash->CTRL |= FLASH_CTRL_ERSTR;

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in]  instance             pointer to a @p EFlashDriver instance
 * @param[out] msec                 recommended time, in milliseconds, that
 *                                  should be spent before calling this
 *                                  function again, can be @p NULL
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_ERASE        if the erase operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
flash_error_t efl_lld_query_erase(void *instance, uint32_t *msec) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  flash_error_t err;

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {

    /* Checking for operation in progress.*/
    if ((devp->flash->STS & FLASH_STS_OBF) == 0U) {

      /* Disabling the various erase control bits.*/
      devp->flash->CTRL &= ~(FLASH_CTRL_SECERS |
                             FLASH_CTRL_BLKERS |
                             FLASH_CTRL_BANKERS);

      /* No operation in progress, checking for errors.*/
      err = at32_flash_check_errors(devp);

      /* Back to ready state.*/
      devp->state = FLASH_READY;
    }
    else {
      /* Recommended time before polling again. This is a simplified
         implementation.*/
      if (msec != NULL) {
        *msec = (uint32_t)AT32_FLASH_WAIT_TIME_MS;
      }

      err = FLASH_BUSY_ERASING;
    }
  }
  else {
    err = FLASH_NO_ERROR;
  }

  return err;
}

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] ip                    pointer to a @p EFlashDriver instance
 * @param[in] sector                sector to be verified
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @notapi
 */
flash_error_t efl_lld_verify_erase(void *instance, flash_sector_t sector) {
  EFlashDriver *devp = (EFlashDriver *)instance;
  uint32_t *address;
  const flash_descriptor_t *bank;
  flash_error_t err = FLASH_NO_ERROR;
  unsigned i;

  osalDbgCheck(instance != NULL);
  bank = efl_lld_get_descriptor(instance);
  osalDbgCheck(bank != NULL);
  osalDbgCheck(sector < bank->sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  /* No verifying while erasing.*/
  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Address of the sector in the bank.*/
  address = (uint32_t *)(bank->address +
                        flashGetSectorOffset(getBaseFlash(devp), sector));

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Scanning the sector space.*/
  uint32_t sector_size = flashGetSectorSize(getBaseFlash(devp), sector);
  for (i = 0U; i < sector_size / sizeof(uint32_t); i++) {
    if (*address != 0xFFFFFFFFU) {
      err = FLASH_ERROR_VERIFY;
      break;
    }
    address++;
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return err;
}

#endif /* HAL_USE_EFL == TRUE */

/** @} */
