

#ifndef SX1238_STM32_DRIVER_H
#define SX1238_STM32_DRIVER_H

#include "stdint.h"








//function prototypes
void SX1238_Init(void);
void SX1238_Reset(void);
void SX1238_Read_Register(uint8_t addr, uint8_t *pOut);
void SX1238_Write_Register(uint8_t addr, uint8_t value);









#endif
