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
 * Project:      Flash Device Description for
 *               ST STM32F469 (32F469IDISCOVERY) with QSPI N25Q128A
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "..\FlashOS.h"            /* FlashOS Structures */

#ifdef N25Q128A
  struct FlashDevice const FlashDevice  =  {
     FLASH_DRV_VERS,               /* Driver Version, do not modify! */
     "32F469IDISCOVERY  N25Q128A", /* Device Name */
     EXTSPI,                       /* Device Type */
     0x90000000,                   /* Device Start Address */
     0x01000000,                   /* Device Size in Bytes (16 MB) */
     0x00001000,                   /* Programming Page Size */
     0x00,                         /* Reserved, must be 0 */
     0xFF,                         /* Initial Content of Erased Memory */
     10000,                        /* Program Page Timeout 1000 mSec */
     10000,                        /* Erase Sector Timeout 1000 mSec */

     /* Specify Size and Address of Sectors */
     0x10000, 0x000000,            /* Sector Size  64kB */
     SECTOR_END
  };
#endif /* N25Q128A */
