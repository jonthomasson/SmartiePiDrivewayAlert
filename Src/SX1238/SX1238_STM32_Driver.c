/* Includes ------------------------------------------------------------------*/
#include "SX1238_STM32_Driver.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Global Variables ------------------------------------------------------------------*/
uint8_t spiDataRcv;
uint8_t regVal = 0x24;

void SX1238_Reset(void)
{
	  HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_RESET);
	  HAL_Delay(10); //delay for 10ms.
	  HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_SET);
	  HAL_Delay(10); //delay for 10ms.
	  HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_RESET);
	  HAL_Delay(10); //delay for 10ms.
}


/*Initialize SX1238 TRX*/
void SX1238_Init(void)
{
	//reset
	SX1238_Reset();

	//initialize registers
	  //read register
	  HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_SET); //SPI NSS setup
	  HAL_Delay(10); //delay for 10ms.
	  //read data
	  //1. pull cs low to activate spi
	  HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_RESET);
	  //2. transmit register address
	  //spiData[0] = REG_OPMODE;
	  //HAL_Delay(10); //delay for 10ms.
	  regVal = REG_SYNCCONFIG;
	  HAL_SPI_Transmit(&hspi1, &regVal, 1, 10);
	  //3. read
	  HAL_SPI_Receive(&hspi1, &spiDataRcv, 1, 10);
	  //4. bring cs high to deactivate
	  HAL_GPIO_WritePin(GPIOA, CE1_Pin, GPIO_PIN_SET);

	  //print out result
	  HAL_UART_Transmit(&huart2, &spiDataRcv, 1, 0xFFFF);
}
