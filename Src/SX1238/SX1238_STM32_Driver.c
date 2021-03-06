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

	//disable mode pins of trx
	HAL_GPIO_WritePin(SX1238_RXEN_PORT, SX1238_RXEN_PIN, GPIO_PIN_RESET); //disable rx
	HAL_GPIO_WritePin(SX1238_MODE_PORT, SX1238_MODE_PIN, GPIO_PIN_RESET); //low gain mode for now
	HAL_GPIO_WritePin(SX1238_TXEN_PORT, SX1238_TXEN_PIN, GPIO_PIN_RESET); //disable power amp transmitter

	//reset trx
	HAL_GPIO_WritePin(SX1238_RESET_PORT, SX1238_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(10); //delay for 10ms.
	HAL_GPIO_WritePin(SX1238_RESET_PORT, SX1238_RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(10); //delay for 10ms.
	HAL_GPIO_WritePin(SX1238_RESET_PORT, SX1238_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(10); //delay for 10ms.
}

void SX1238_Set_Mode(uint8_t newMode)
{
	unsigned char rcv1 = 0;
	//unsigned char rcv2;
	unsigned char regVal;

	if(newMode == _mode) //if it's the same mode, just return.
	{
		return;
	}

	regVal = newMode | 0x08;
	SX1238_Write_Register(REG_OPMODE, regVal); //adding gausian filter too...
	//SX1238_Read_Register(REG_OPMODE, &rcv2);

	while ((rcv1 & RF_IRQFLAGS1_MODEREADY) == 0x00) // wait for ModeReady
	{
		SX1238_Read_Register(REG_IRQFLAGS1, &rcv1);
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

void SX1238_Handle_Interrupt()
{

	// Get the interrupt cause
	unsigned char irqflags2;
	SX1238_Read_Register(REG_IRQFLAGS2, &irqflags2);
	if (_mode == SX1238_MODE_TX && (irqflags2 & RF_IRQFLAGS2_PACKETSENT))
	{
		// A transmitter message has been fully sent
		SX1238_Set_Mode(SX1238_MODE_STANDBY);
	}
}

void SX1238_Send_Frame(uint8_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{

	unsigned char TempBuffer1;
	SX1238_Set_Mode(SX1238_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo

	SX1238_Write_Register(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
	if (bufferSize > MAX_DATA_LEN)
	{
		bufferSize = MAX_DATA_LEN;
	}

	// control byte. Note: we may not need this part.
	uint8_t CTLbyte = 0x00;
	if (sendACK)
	{
		CTLbyte = CTL_SENDACK;
	}else if(requestACK)
	{
		CTLbyte = CTL_REQACK;
	}

	// write to FIFO
	SX1238_SPI_Select();
	TempBuffer1 = REG_FIFO | 0x80;
	HAL_SPI_Transmit(HSPI_INSTANCE, &TempBuffer1, 1, 10);
	TempBuffer1 = bufferSize + 3;
	HAL_SPI_Transmit(HSPI_INSTANCE, &TempBuffer1, 1, 10);
	TempBuffer1 = toAddress;
	HAL_SPI_Transmit(HSPI_INSTANCE, &TempBuffer1, 1, 10);
	TempBuffer1 = NODE_ADDRESS;
	HAL_SPI_Transmit(HSPI_INSTANCE, &TempBuffer1, 1, 10);
	TempBuffer1 = CTLbyte;
	HAL_SPI_Transmit(HSPI_INSTANCE, &TempBuffer1, 1, 10);

	for (uint8_t i = 0; i < bufferSize; i++)
	{
		TempBuffer1 = ((uint8_t*) buffer)[i];
		HAL_SPI_Transmit(HSPI_INSTANCE, &TempBuffer1, 1, 10);
	}

	SX1238_SPI_Unselect();

	// no need to wait for transmit mode to be ready since its handled by the radio
	SX1238_Set_Mode(SX1238_MODE_TX);
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
	unsigned char TempBuffer2;

	//1. pull cs low to activate spi
	SX1238_SPI_Select();

	//2. transmit register address
	TempBuffer2 = addr & 0x7F;
	HAL_SPI_Transmit(HSPI_INSTANCE, &TempBuffer2, 1, 10);

	//3. read
	HAL_SPI_Receive(HSPI_INSTANCE, pOut, 1, 10);

	//4. bring cs high to deactivate
	SX1238_SPI_Unselect();
}

/*Writes a value to the register at the given address*/
void SX1238_Write_Register(uint8_t addr, unsigned char value)
{
	unsigned char regVal1 = 0;
	unsigned char regAddr1 = 0;
	//1. pull cs low to activate spi
	SX1238_SPI_Select();

	//2. transmit register address
	regAddr1 = addr | 0x80;
	HAL_SPI_Transmit(HSPI_INSTANCE, &regAddr1, 1, 10);

	//3. transmit register address
	regVal1 = value;
	HAL_SPI_Transmit(HSPI_INSTANCE, &regVal1, 1, 10);

	//4. bring cs high to deactivate
	SX1238_SPI_Unselect();
}


/*Initialize SX1238 TRX*/
void SX1238_Init(void)
{
	//reset
	SX1238_Reset();

	//initialize registers
	//unsigned char rcv2;

	SX1238_Write_Register(REG_PACONFIG, PACONFIG_OUTPUTPOWER_1); //output power to default
	//SX1238_Read_Register(REG_PACONFIG, &rcv2);

	SX1238_Write_Register(REG_FIFOTHRESH, FIFOTHRESH_TXSTARTCONDITION_FIFOEMPTY | FIFOTHRESH_FIFOTHRESHOLD); //0X8F fifo start condition not empty
	//SX1238_Read_Register(REG_FIFOTHRESH, &rcv2);

	SX1238_Write_Register(REG_PACKETCONFIG1, PACKETCONFIG1_PACKETFORMAT_VARIABLE); //0X80 turn off crc
	//SX1238_Read_Register(REG_PACKETCONFIG1, &rcv2);

	SX1238_Write_Register(REG_PACKETCONFIG2, PACKETCONFIG2_DATAMODE_PACKET); //0X40 packet mode
	//SX1238_Read_Register(REG_PACKETCONFIG2, &rcv2);

	SX1238_Write_Register(REG_PREAMBLEMSB, PREAMBLEMSB); //0X00 preamble length
	//SX1238_Read_Register(REG_PREAMBLEMSB, &rcv2);

	SX1238_Write_Register(REG_PREAMBLELSB, PREAMBLELSB); //0X03
	//SX1238_Read_Register(REG_PREAMBLELSB, &rcv2);

	SX1238_Write_Register(REG_FRFMSB, FRFMSB); // 0XE4 frequency 915MHz
	//SX1238_Read_Register(REG_FRFMSB, &rcv2);

	SX1238_Write_Register(REG_FRFMID, FRFMID);
	//SX1238_Read_Register(REG_FRFMID, &rcv2);

	SX1238_Write_Register(REG_FRFLSB, FRFLSB);
	//SX1238_Read_Register(REG_FRFLSB, &rcv2);

	SX1238_Write_Register(REG_SYNCCONFIG, SYNCCONFIG_AUTORESTARTRXMODE_ONWITHPLL | SYNCCONFIG_SYNC_ON | SYNCCONFIG_SYNCSIZE_1); //0x91 auto restart, sync on, fill auto, sync size 2 bytes
	//SX1238_Read_Register(REG_SYNCCONFIG, &rcv2);

	SX1238_Write_Register(REG_SYNCVALUE1, SYNCVALUE1);
	//SX1238_Read_Register(REG_SYNCVALUE1, &rcv2);

	SX1238_Write_Register(REG_SYNCVALUE2, SYNCVALUE2);
	//SX1238_Read_Register(REG_SYNCVALUE2, &rcv2);

	SX1238_Write_Register(REG_BITRATEMSB, BITRATEMSB); //bit rates etc...
	//SX1238_Read_Register(REG_BITRATEMSB, &rcv2);

	SX1238_Write_Register(REG_BITRATELSB, BITRATELSB);
	//SX1238_Read_Register(REG_BITRATELSB, &rcv2);

	SX1238_Write_Register(REG_FDEVMSB, FDEVMSB); //frequency deviation (deviation in Hz = fdev * 61)
	//SX1238_Read_Register(REG_FDEVMSB, &rcv2);

	SX1238_Write_Register(REG_FDEVLSB, FDEVLSB); //see datasheet for max fdev limits (https://www.semtech.com/uploads/documents/sx1238.pdf page 22)
	//SX1238_Read_Register(REG_FDEVLSB, &rcv2);

	SX1238_Write_Register(REG_RXBW, RXBW);
	//SX1238_Read_Register(REG_RXBW, &rcv2);

	SX1238_Write_Register(REG_NODEADRS, NODE_ADDRESS); //setting node address
	//SX1238_Read_Register(REG_NODEADRS, &rcv2);

	SX1238_Set_Mode(SX1238_MODE_STANDBY); //enter standby mode

}
