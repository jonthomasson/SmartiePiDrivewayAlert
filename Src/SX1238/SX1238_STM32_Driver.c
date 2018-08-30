/* Includes ------------------------------------------------------------------*/
#include "SX1238_STM32_Driver.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Global Variables ------------------------------------------------------------------*/
uint8_t _mode; //current mode of trx


void SX1238_Reset(void)
{
	SX1238_SPI_Unselect(); //SPI NSS setup
	HAL_GPIO_WritePin(SX1238_RESET_PORT, SX1238_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(10); //delay for 10ms.
	HAL_GPIO_WritePin(SX1238_RESET_PORT, SX1238_RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(10); //delay for 10ms.
	HAL_GPIO_WritePin(SX1238_RESET_PORT, SX1238_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(10); //delay for 10ms.
}

void SX1238_Set_Mode(uint8_t newMode)
{
	uint8_t rcv;
	uint8_t foundFlag = 0;

	if(newMode == _mode) //if it's the same mode, just return.
	{
		return;
	}

	SX1238_Write_Register(REG_OPMODE, newMode | 0x08); //adding gausian filter too...



	while (foundFlag == 0) // wait for ModeReady
	{
		SX1238_Read_Register(REG_IRQFLAGS1, &rcv);

		if((rcv & RF_IRQFLAGS1_MODEREADY) == 0x00)
		{
			foundFlag = 1;
		}
	}
	_mode = newMode;

	switch (newMode) {
	case SX1238_MODE_TX:
		HAL_GPIO_WritePin(SX1238_RXEN_PORT, SX1238_RXEN_PIN, GPIO_PIN_RESET); //disable rx
		HAL_GPIO_WritePin(SX1238_MODE_PORT, SX1238_MODE_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SX1238_TXEN_PORT, SX1238_TXEN_PIN, GPIO_PIN_SET);   //enable power amp transmitter
		break;
	case SX1238_MODE_RX:
		HAL_GPIO_WritePin(SX1238_RXEN_PORT, SX1238_RXEN_PIN, GPIO_PIN_SET);   //enable rx
		HAL_GPIO_WritePin(SX1238_MODE_PORT, SX1238_MODE_PIN, GPIO_PIN_RESET); //low gain mode for now
		HAL_GPIO_WritePin(SX1238_TXEN_PORT, SX1238_TXEN_PIN, GPIO_PIN_RESET); //disable power amp transmitter
		break;
	case SX1238_MODE_SLEEP:
		HAL_GPIO_WritePin(SX1238_RXEN_PORT, SX1238_RXEN_PIN, GPIO_PIN_RESET); //disable rx
		HAL_GPIO_WritePin(SX1238_MODE_PORT, SX1238_MODE_PIN, GPIO_PIN_RESET); //low gain mode for now
		HAL_GPIO_WritePin(SX1238_TXEN_PORT, SX1238_TXEN_PIN, GPIO_PIN_RESET); //disable power amp transmitter
		break;
	case SX1238_MODE_STANDBY:
		HAL_GPIO_WritePin(SX1238_RXEN_PORT, SX1238_RXEN_PIN, GPIO_PIN_RESET); //disable rx
		HAL_GPIO_WritePin(SX1238_MODE_PORT, SX1238_MODE_PIN, GPIO_PIN_RESET); //low gain mode for now
		HAL_GPIO_WritePin(SX1238_TXEN_PORT, SX1238_TXEN_PIN, GPIO_PIN_RESET); //disable power amp transmitter
		break;
	default:
		return;
	}
}

void SX1238_SPI_Select()
{
	//pull ce low to activate spi
	HAL_GPIO_WritePin(SX1238_CE_PORT, SX1238_CE_PIN, GPIO_PIN_RESET);
}

void SX1238_SPI_Unselect()
{
	//bring ce high to deactivate
	HAL_GPIO_WritePin(SX1238_CE_PORT, SX1238_CE_PIN, GPIO_PIN_SET);
}

/*Reads a register at the specified address and returns its value*/
void SX1238_Read_Register(uint8_t addr, uint8_t *pOut)
{
	//uint8_t spiDataRcv;
	uint8_t regVal = 0;
	//1. pull cs low to activate spi
	SX1238_SPI_Select();

	//2. transmit register address
	regVal = addr & 0x7F;
	HAL_SPI_Transmit(HSPI_INSTANCE, &regVal, 1, 10);

	//3. read
	HAL_SPI_Receive(HSPI_INSTANCE, pOut, 1, 10);

	//4. bring cs high to deactivate
	SX1238_SPI_Unselect();

	//return spiDataRcv;
}

/*Writes a value to the register at the given address*/
void SX1238_Write_Register(uint8_t addr, unsigned char value)
{
	uint8_t regVal = 10;
	uint8_t regAddr = 0;
	//1. pull cs low to activate spi
	SX1238_SPI_Select();

	//2. transmit register address
	regAddr = addr | 0x80;
	HAL_SPI_Transmit(HSPI_INSTANCE, &regAddr, 1, 10);

	//3. transmit register address
	//regVal = value;
	HAL_SPI_Transmit(HSPI_INSTANCE, &regVal, 1, 10);

	//4. bring cs high to deactivate
	SX1238_SPI_Unselect();
}


/*Initialize SX1238 TRX*/
void SX1238_Init(void)
{
	//reset
	SX1238_Reset();

	//initialize registers
	SX1238_Write_Register(REG_PACONFIG, 0x01); //output power to default

	SX1238_Write_Register(REG_FIFOTHRESH, 0x8f); //fifo start condition not empty

	SX1238_Write_Register(REG_PACKETCONFIG1, 0x80); //turn off crc
	SX1238_Write_Register(REG_PACKETCONFIG2, 0x40); //packet mode

	SX1238_Write_Register(REG_PREAMBLEMSB, 0x00); //preamble length
	SX1238_Write_Register(REG_PREAMBLELSB, 0x03);

	SX1238_Write_Register(REG_FRFMSB, 0xe4); //frequency 915MHz
	SX1238_Write_Register(REG_FRFMID, 0xc0);
	SX1238_Write_Register(REG_FRFLSB, 0x00);

	SX1238_Write_Register(REG_SYNCCONFIG, 0x91); //auto restart, sync on, fill auto, sync size 2 bytes
	SX1238_Write_Register(REG_SYNCVALUE1, 0x5A);
	SX1238_Write_Register(REG_SYNCVALUE2, 0x5A);

	SX1238_Write_Register(REG_BITRATEMSB, 0x1a); //bit rates etc...
	SX1238_Write_Register(REG_BITRATELSB, 0x0b);

	SX1238_Write_Register(REG_FDEVMSB, 0x00); //frequency deviation (deviation in Hz = fdev * 61)
	SX1238_Write_Register(REG_FDEVLSB, 0x52); //see datasheet for max fdev limits (https://www.semtech.com/uploads/documents/sx1238.pdf page 22)

	SX1238_Write_Register(REG_RXBW, 0x05);

	SX1238_Write_Register(REG_NODEADRS, NODE_ADDRESS); //setting node address

	SX1238_Set_Mode(SX1238_MODE_STANDBY); //enter standby mode

}
