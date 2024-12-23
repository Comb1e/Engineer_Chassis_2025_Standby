//
// Created by CYK on 2024/11/21.
//

#ifndef CRC_H
#define CRC_H

#include "stm32f4xx_hal.h"

void CRC16_Update(uint16_t *currect_crc, const uint8_t *src, uint32_t len);

uint8_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

extern const uint8_t CRC8_INIT;
extern uint16_t CRC_INIT;

#endif //CRC_H
