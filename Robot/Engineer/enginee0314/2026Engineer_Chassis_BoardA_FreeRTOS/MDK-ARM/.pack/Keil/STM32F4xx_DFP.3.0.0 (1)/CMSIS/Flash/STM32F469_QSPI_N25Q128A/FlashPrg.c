/* -----------------------------------------------------------------------------
 * Copyright (c) 2022 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        13. Dezember 2022
 * $Revision:    V1.0.0
 *
 * Project:      Flash Programming Functions for
 *               ST STM32F469 (32F469IDISCOVERY) with QSPI N25Q128A
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include <string.h>
#include "..\FlashOS.h"                /* FlashOS Structures */

#include "stm32f4xx_hal.h"
#include "stm32469i_discovery_qspi.h"


/* Private variables ---------------------------------------------------------*/
extern void SystemInit(void);
extern void SystemClock_Config(void);


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
  int32_t rc;

  __disable_irq();

  SystemInit();
  SystemClock_Config();          /* configure system core clock */
//  SystemCoreClockUpdate();

  rc = BSP_QSPI_Init();
  return ((rc == QSPI_OK) ? 0 : 1);
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {
  int32_t rc;

  rc = BSP_QSPI_DeInit();

  return ((rc == QSPI_OK) ? 0 : 1);
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {
  int32_t rc;

  rc = BSP_QSPI_Erase_Chip();

  if (rc != QSPI_OK) {
    return (1);
  }

  /* Wait the end of the current operation on memory side */
  do
  {
    rc = BSP_QSPI_GetStatus();
  } while(rc == QSPI_BUSY);

  return ((rc == QSPI_OK) ? 0 : 1);
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {
  int32_t rc;

  rc = BSP_QSPI_Erase_Sector((uint32_t)(adr & 0x0FFFFFFF));

  if (rc != QSPI_OK) {
    return (1);
  }

  /* Wait the end of the current operation on memory side */
  do
  {
    rc = BSP_QSPI_GetStatus();
  } while(rc == QSPI_BUSY);

  return ((rc == QSPI_OK) ? 0 : 1);
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
  int32_t rc;

  rc = BSP_QSPI_Write((uint8_t*)buf, (uint32_t)(adr & 0x0FFFFFFF), (uint32_t)sz);

  return ((rc == QSPI_OK) ? 0 : 1);
}


 /*
  *  Verify Flash Contents
  *    Parameter:      adr:  Start Address
  *                    sz:   Size (in bytes)
  *                    buf:  Data
  *    Return Value:   (adr+sz) - OK, Failed Address
 */
unsigned long Verify (unsigned long adr, unsigned long sz, unsigned char *buf){
  uint8_t * ptr = (uint8_t *)adr;
  uint32_t i;
  int32_t rc;

  rc = BSP_QSPI_EnableMemoryMappedMode();
  if (rc != QSPI_OK) {
    return (adr);
  }

  for(i = 0; i < sz; i++)
  {
    if (ptr[i] != buf[i])
      return (adr + i);                /* Verification Failed (return address) */
  }

 // rc = BSP_QSPI_DisableMemoryMappedMode();
  rc = BSP_QSPI_Init();
  if (rc != QSPI_OK) {
    return (adr);
  }

  return (adr + sz);                   /* Done successfully */
}


/*  Blank Check Block in Flash Memory
 *    Parameter:      adr:  Block Start Address
 *                    sz:   Block Size (in bytes)
 *                    pat:  Block Pattern
 *    Return Value:   0 - OK,  1 - Failed
 */
int BlankCheck  (unsigned long adr, unsigned long sz, unsigned char pat) {
  uint8_t * ptr = (uint8_t *)adr;
  uint32_t i = 0;
  int result = 0;
  int32_t rc;


  rc = BSP_QSPI_EnableMemoryMappedMode();

  for (i = 0; i < sz; i++)
  {
    if(ptr[i] != pat) {
      result = 1;
      break;
    }
  }

//  rc = BSP_QSPI_DisableMemoryMappedMode();
  rc = BSP_QSPI_Init();

  return (result);
}


/* -- helper functions for test application -- */
void SetOSPIMemMode(void) {

  BSP_QSPI_DeInit();
  BSP_QSPI_Init();
  BSP_QSPI_EnableMemoryMappedMode();
}

void ReSetOSPIMemMode(void) {

  BSP_QSPI_DeInit();
  BSP_QSPI_Init();
}
