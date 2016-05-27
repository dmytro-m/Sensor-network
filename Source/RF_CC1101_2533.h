#ifndef _RF_CC1101_2533_H_
#define _RF_CC1101_2533_H_

#include "Config.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Macro Declaration
//
////////////////////////////////////////////////////////////////////////////////////////////////////
// common register
#define RF_IOCFG2					0x00		// IOCFG2        - GDO2 output pin configuration
#define RF_IOCFG1					0x01		// IOCFG1        - GDO1 output pin configuration
#define RF_IOCFG0					0x02		// IOCFG1        - GDO0 output pin configuration
#define RF_FIFOTHR					0x03		// FIFOTHR       - RX FIFO and TX FIFO thresholds
#define RF_SYNC1					0x04		// SYNC1         - Sync word, high byte
#define RF_SYNC0					0x05		// SYNC0         - Sync word, low byte
#define RF_PKTLEN					0x06		// PKTLEN        - Packet length
#define RF_PKTCTRL1					0x07		// PKTCTRL1      - Packet automation control
#define RF_PKTCTRL0					0x08		// PKTCTRL0      - Packet automation control
#define RF_ADDR						0x09		// ADDR          - Device address
#define RF_CHANNR					0x0A		// CHANNR		 - Channel number
#define RF_FSCTRL1					0x0B		// FSCTRL1       - Frequency synthesizer control
#define RF_FSCTRL0					0x0C		// FSCTRL0       - Frequency synthesizer control
#define RF_FREQ2					0x0D		// FREQ2         - Frequency control word, high byte
#define RF_FREQ1					0x0E		// FREQ1         - Frequency control word, middle byte
#define RF_FREQ0					0x0F		// FREQ0         - Frequency control word, low byte
#define RF_MDMCFG4					0x10		// MDMCFG4       - Modem configuration
#define RF_MDMCFG3					0x11		// MDMCFG3       - Modem configuration
#define RF_MDMCFG2					0x12		// MDMCFG2       - Modem configuration
#define RF_MDMCFG1					0x13		// MDMCFG1       - Modem configuration
#define RF_MDMCFG0					0x14		// MDMCFG0       - Modem configuration
#define RF_DEVIATN					0x15		// DEVIATN       - Modem deviation setting
#define RF_MCSM2					0x16		// MCSM2         - Main Radio Control State Machine configuration
#define RF_MCSM1 					0x17		// MCSM1         - Main Radio Control State Machine configuration
#define RF_MCSM0					0x18		// MCSM0         - Main Radio Control State Machine configuration
#define RF_FOCCFG 					0x19		// FOCCFG        - Frequency Offset Compensation configuration
#define RF_BSCFG					0x1A		// BSCFG         - Bit Synchronization configuration
#define RF_AGCCTRL2					0x1B		// AGCCTRL2      - AGC control
#define RF_AGCCTRL1					0x1C		// AGCCTRL1      - AGC control
#define RF_AGCCTRL0					0x1D		// AGCCTRL0      - AGC control
#define RF_WOREVT1					0x1E		// WOREVT1		 - High byte Event0 timeout
#define RF_WOREVT0					0x1F		// WOREVT0		 - Low byte Event0 timeout
#define RF_WORCTRL					0x20		// WORCTRL		 - Wake On Radio control
#define RF_FREND1					0x21		// FREND1        - Front end RX configuration
#define RF_FREND0					0x22		// FREDN0        - Front end TX configuration
#define RF_FSCAL3					0x23		// FSCAL3        - Frequency synthesizer calibration
#define RF_FSCAL2					0x24		// FSCAL2        - Frequency synthesizer calibration
#define RF_FSCAL1					0x25		// FSCAL1        - Frequency synthesizer calibration
#define RF_FSCAL0 					0x26		// FSCAL0        - Frequency synthesizer calibration
#define RF_RCCTRL1					0x27		// RCCTRL1		 - RC oscillator configuration
#define RF_RCCTRL0					0x28		// RCCTRL0		 - RC oscillator configuration
#define RF_FSTEST					0x29		// FSTEST		 - Frequency synthesizer calibration control
#define RF_PTEST					0x2A		// PTEST		 - Production test
#define RF_AGCTEST					0x2B		// AGCTEST		 - AGC test
#define RF_TEST2					0x2C		// TEST2         - Various test settings
#define RF_TEST1					0x2D		// TEST1         - Various test settings
#define RF_TEST0					0x2E		// TEST0         - Various test settings

// status registers CC1101
#define RF_PARTNUM					0x30		// PARTNUM		 - Chip ID
#define RF_VERSION					0x31		// VERSION		 - Chip ID
#define RF_FREQEST					0x32		// FREQEST		 - Frequency Offset Estimate from demodulator
#define RF_LQI						0x33		// LQI			 - Demodulator estimate for Link Quality
#define RF_RSSI						0x34		// RSSI			 - Received signal strength indication
#define RF_MARCSTATE				0x35		// MARCSTATE	 - Main Radio Control State Machine state
#define RF_WORTIME1					0x36		// WORTIME1		 - High byte of WOR time
#define RF_WORTIME0					0x37		// WORTIME0		 - Low byte of WOR time
#define RF_PKTSTATUS				0x38		// PKTSTATUS	 - Current GDOx status and packet status
#define RF_VCO_VC_DAC				0x39		// VCO_VC_DAC	 - Current setting from PLL calibration module
#define RF_TXBYTES					0x3A		// TXBYTES		 - Underflow and number of bytes
#define RF_RXBYTES					0x3B		// RXBYTES		 - Overflow and number of bytes

// burst write registers
#define RF_PA_TABLE0				0x3E		// PA_TABLE0 - PA control settings table
#define RF_RXFIFO					0x3F		// FIFO  - Transmit FIFO
#define RF_TXFIFO					0x3F		// FIFO  - Receive FIFO

// Other register bit fields
#define RF_LQI_CRC_OK_BM			0x80
#define RF_LQI_EST_BM				0x7F

// CC110L Command strobe registers
#define RF_SRES						0x30		// SRES    - Reset chip.
#define RF_SFSTXON					0x31		// SFSTXON - Enable and calibrate frequency synthesizer.
#define RF_SXOFF					0x32		// SXOFF   - Turn off crystal oscillator.
#define RF_SCAL						0x33		// SCAL    - Calibrate frequency synthesizer and turn it off.
#define RF_SRX						0x34		// SRX     - Enable RX. Perform calibration if enabled.
#define RF_STX						0x35		// STX     - Enable TX. If in RX state, only enable TX if CCA passes.
#define RF_SIDLE					0x36		// SIDLE   - Exit RX / TX, turn off frequency synthesizer.
#define RF_SPWD						0x39		// SPWD    - Enter power down mode when CSn goes high.
#define RF_SFRX						0x3A		// SFRX    - Flush the RX FIFO buffer.
#define RF_SFTX						0x3B		// SFTX    - Flush the TX FIFO buffer.
#define RF_SNOP						0x3D		// SNOP    - No operation. Returns status byte.

// CC110L Chip states returned in status byte
#define RF_STATE_IDLE				0x00
#define RF_STATE_RX					0x10
#define RF_STATE_TX					0x20
#define RF_STATE_FSTXON				0x30
#define RF_STATE_CALIBRATE			0x40
#define RF_STATE_SETTLING			0x50
#define RF_STATE_RXFIFO_ERROR		0x60
#define RF_STATE_TXFIFO_ERROR 		0x70

#define RF_BURST_ACCESS   			0x40
#define RF_SINGLE_ACCESS  			0x00
#define RF_READ_ACCESS				0x80
#define RF_WRITE_ACCESS				0x00

//#define PATABLE_VAL			(0xC0)				// max output
//#define PATABLE_VAL			(0x50)				// 0 dbm
#define PATABLE_VAL			(0x27)				// -10 dbm

#define RF_STROBE_ERROR		0x80
#define RF_ACCESS_ERROR		0xFF

typedef struct
{
	unsigned char transmitting;
	unsigned char receiving;
	uint8 times;
}RF_STATE;

extern RF_STATE rf_state;

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Function Declaration
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void RF_Init(void);
uint8 RF_Transmit(uint8 *buffer, uint8 length, uint8 check);
void RF_Receive_On(void);
void RF_Receive_Off(void);
uint8 RF_Strobe(uint8 strobe);
uint8 RF_Read_SingleReg(uint8 addr);
uint8 RF_Write_SingleReg(uint8 addr, uint8 value);
uint8 RF_Read_BurstReg(uint8 addr, uint8 *buffer, uint8 count);
uint8 RF_Write_BurstReg(uint8 addr, uint8 *buffer, uint8 count);
void RF_Reset(void);
void RF_Setting(void);
uint8 RF_Write_SinglePATable(uint8 value);
uint8 RF_Write_BurstPATable(uint8 *buffer, uint8 count);
void RF_Re_Init(void);
uint8 RF_Channel_Idle(uint8 times, uint8 min, uint8 max);

#endif
