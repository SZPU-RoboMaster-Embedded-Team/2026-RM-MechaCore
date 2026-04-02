#ifndef __RMRefereeSystemCRC_Hpp__
#define __RMRefereeSystemCRC_Hpp__

#include "main.h"

/* CRC8 */
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/* CRC16 */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif

