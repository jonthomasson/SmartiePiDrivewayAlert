

#ifndef SX1238_STM32_DRIVER_H
#define SX1238_STM32_DRIVER_H

#include "stdint.h"


// **********************************************************************************
// SX1238 Internal registers addresses
//**************************************************
#define REG_FIFO          0x00
#define REG_OPMODE        0x01
#define REG_BITRATEMSB    0x02
#define REG_BITRATELSB    0x03
#define REG_FDEVMSB       0x04
#define REG_FDEVLSB       0x05
#define REG_FRFMSB        0x06
#define REG_FRFMID        0x07
#define REG_FRFLSB        0x08
#define REG_PACONFIG      0x09
#define REG_PARAMP        0x0A
#define REG_OCP           0x0B
#define REG_LNA           0x0C
#define REG_RXCONFIG      0x0D
#define REG_RSSICONFIG    0x0E
#define REG_RSSICOLLISION 0x0F
#define REG_RSSITHRESH    0x10
#define REG_RSSIVALUE     0x11
#define REG_RXBW          0x12
#define REG_AFCBW         0x13
#define REG_OOKPEAK       0x14
#define REG_OOKFIX        0x15
#define REG_OOKAVG        0x16
#define REG_AFCFEI        0x1A
#define REG_AFCMSB        0x1B
#define REG_AFCLSB        0x1C
#define REG_FEIMSB        0x1D
#define REG_FEILSB        0x1E
#define REG_PREAMBLEDETECT  0x1F
#define REG_RXTIMEOUT1    0x20
#define REG_RXTIMEOUT2    0x21
#define REG_RXTIMEOUT3    0x22
#define REG_RXDELAY       0x23
#define REG_OSC           0x24
#define REG_PREAMBLEMSB   0x25
#define REG_PREAMBLELSB   0x26
#define REG_SYNCCONFIG    0x27
#define REG_SYNCVALUE1    0x28
#define REG_SYNCVALUE2    0x29
#define REG_SYNCVALUE3    0x2A
#define REG_SYNCVALUE4    0x2B
#define REG_SYNCVALUE5    0x2C
#define REG_SYNCVALUE6    0x2D
#define REG_SYNCVALUE7    0x2E
#define REG_SYNCVALUE8    0x2F
#define REG_PACKETCONFIG1 0x30
#define REG_PACKETCONFIG2 0x31
#define REG_PAYLOADLENGTH 0x32
#define REG_NODEADRS      0x33
#define REG_BROADCASTADRS 0x34
#define REG_FIFOTHRESH    0x35
#define REG_SEQCONFIG1    0x36
#define REG_SEQCONFIG2    0x37
#define REG_TIMERRESOL    0x38
#define REG_TIMER1COEF    0x39
#define REG_TIMER2COEF    0x3A
#define REG_IMAGECAL      0x3B
#define REG_TEMP          0x3C
#define REG_LOWBAT        0x3D
#define REG_IRQFLAGS1     0x3E
#define REG_IRQFLAGS2     0x3F
#define REG_DIOMAPPING1   0x40
#define REG_DIOMAPPING2   0x41
#define REG_VERSION       0x42
#define REG_AGCREF        0x43
#define REG_AGCTHRESH1    0x44
#define REG_AGCTHRESH2    0x45
#define REG_AGCTHRESH3    0x46
#define REG_TCXO          0x58
#define REG_PADAC         0x5A
#define REG_PLL           0x5C
#define REG_PLLLOWPN      0x5E
#define REG_FORMERTEMP    0x6C
#define REG_BITRATEFRAC   0x70

//#define SERIAL_BAUD       115200
//#define SS                4
//#define TX_EN             7
//#define RX_EN             9
//#define MODE              6
//#define MAX_DATA_LEN      61   //may not need later
//#define TX_LIMIT_MS       1000 //may not need later
//#define ADDRESS           10   //may not need later
//#define DIO0_INTERRUPT    3
//#define NODE_TO_ADDR      2    //node receiving data
//#define RESET         	  52


// RegIrqFlags1
#define RF_IRQFLAGS1_MODEREADY            0x80
#define RF_IRQFLAGS1_RXREADY              0x40
#define RF_IRQFLAGS1_TXREADY              0x20
#define RF_IRQFLAGS1_PLLLOCK              0x10
#define RF_IRQFLAGS1_RSSI                 0x08
#define RF_IRQFLAGS1_TIMEOUT              0x04
#define RF_IRQFLAGS1_PREAMBLEDETECT       0x02
#define RF_IRQFLAGS1_SYNCADDRESSMATCH     0x01

// RegIrqFlags2
#define RF_IRQFLAGS2_FIFOFULL             0x80
#define RF_IRQFLAGS2_FIFOEMPTY            0x40
#define RF_IRQFLAGS2_FIFOLEVEL            0x20
#define RF_IRQFLAGS2_FIFOOVERRUN          0x10
#define RF_IRQFLAGS2_PACKETSENT           0x08
#define RF_IRQFLAGS2_PAYLOADREADY         0x04
#define RF_IRQFLAGS2_CRCOK                0x02
#define RF_IRQFLAGS2_LOWBAT               0x01

// RegDioMapping1
#define RF_DIOMAPPING1_DIO0_00            0x00  // Default
#define RF_DIOMAPPING1_DIO0_01            0x40
#define RF_DIOMAPPING1_DIO0_10            0x80
#define RF_DIOMAPPING1_DIO0_11            0xC0

#define RF_DIOMAPPING1_DIO1_00            0x00  // Default
#define RF_DIOMAPPING1_DIO1_01            0x10
#define RF_DIOMAPPING1_DIO1_10            0x20
#define RF_DIOMAPPING1_DIO1_11            0x30

#define RF_DIOMAPPING1_DIO2_00            0x00  // Default
#define RF_DIOMAPPING1_DIO2_01            0x04
#define RF_DIOMAPPING1_DIO2_10            0x08
#define RF_DIOMAPPING1_DIO2_11            0x0C

#define RF_DIOMAPPING1_DIO3_00            0x00  // Default
#define RF_DIOMAPPING1_DIO3_01            0x01
#define RF_DIOMAPPING1_DIO3_10            0x02
#define RF_DIOMAPPING1_DIO3_11            0x03


//transceiver modes
#define SX1238_MODE_SLEEP         0 //none
#define SX1238_MODE_STANDBY       1 // Top regulator and crystal oscillator
#define SX1238_MODE_SYNTH_TX      2 // Frequency synthesizer at Tx frequency (Frf)
#define SX1238_MODE_TX            3 // Frequency synthesizer and transmitter
#define SX1238_MODE_SYNTH_RX      4 // Frequency synthesizer at frequency for reception (Frf-IF)
#define SX1238_MODE_RX            5 // Frequency synthesizer and receiver

//SPI INSTANCE
#define HSPI_INSTANCE				&hspi1

//SX1238 TX ENABLE PIN AND PORT
#define SX1238_TXEN_PORT			GPIOC
#define SX1238_TXEN_PIN				SX1238_TXEN_Pin

//SX1238 RX ENABLE PIN AND PORT
#define SX1238_RXEN_PORT			GPIOC
#define SX1238_RXEN_PIN				SX1238_RXEN_Pin

//SX1238 MODE PIN AND PORT
#define SX1238_MODE_PORT			GPIOA
#define SX1238_MODE_PIN				SX1238_MODE_Pin

//SX1238 RESET PIN AND PORT
#define SX1238_RESET_PORT			GPIOA
#define SX1238_RESET_PIN			RESET_Pin

//SX1238 CE PIN AND PORT
#define SX1238_CE_PORT				GPIOA
#define SX1238_CE_PIN				CE1_Pin

#define NODE_ADDRESS                10


//function prototypes
void SX1238_Init(void);
void SX1238_Reset(void);
void SX1238_Read_Register(uint8_t addr, uint8_t *pOut);
void SX1238_Write_Register(uint8_t addr, uint8_t value);
void SX1238_Set_Mode(uint8_t mode);
void SX1238_SPI_Select();
void SX1238_SPI_Unselect();









#endif
