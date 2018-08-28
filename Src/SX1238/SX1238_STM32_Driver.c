/* Includes ------------------------------------------------------------------*/
#include "SX1238_STM32_Driver.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Global Variables ------------------------------------------------------------------*/
//uint8_t spiDataRcv;
//uint8_t regVal = 0x24;

void SX1238_Reset(void)
{
	HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_SET); //SPI NSS setup
	HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10); //delay for 10ms.
	HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(10); //delay for 10ms.
	HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10); //delay for 10ms.
}

/*Reads a register at the specified address and returns its value*/
void SX1238_Read_Register(uint8_t addr, uint8_t *pOut)
{
	//uint8_t spiDataRcv;
	uint8_t regVal = 0;
	//1. pull cs low to activate spi
	HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_RESET);

	//2. transmit register address
	regVal = addr & 0x7F;
	HAL_SPI_Transmit(&hspi1, &regVal, 1, 10);

	//3. read
	HAL_SPI_Receive(&hspi1, pOut, 1, 10);

	//4. bring cs high to deactivate
	HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_SET);

	//return spiDataRcv;
}

/*Writes a value to the register at the given address*/
void SX1238_Write_Register(uint8_t addr, unsigned char value)
{
	uint8_t regVal = 10;
	uint8_t regAddr = 0;
	//1. pull cs low to activate spi
	HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_RESET);

	//2. transmit register address
	regAddr = addr | 0x80;
	HAL_SPI_Transmit(&hspi1, &regAddr, 1, 10);

	//3. transmit register address
	//regVal = value;
	HAL_SPI_Transmit(&hspi1, &regVal, 1, 10);

	//4. bring cs high to deactivate
	HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_SET);
}


/*Initialize SX1238 TRX*/
void SX1238_Init(void)
{
	//reset
	SX1238_Reset();

	//initialize registers


}
