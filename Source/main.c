#include "Config.h"
#include "Mesh.h"
#include "Hal_2533.h"
#include <msp430g2553.h>
#include "RF_CC1101_2533.h"

#ifdef SLAVE
#include "onewire.h"
#include "delay.h"
const uint8 VCC_INTERVAL = 60; // time to measure VCC, unit is 10 s
uint8 counter=0;//count every 10s

void GetSupplyVoltage(void);
void GetAnalogValue(void);
#endif

uint8 flagnum;		// used to handle UART received bytes
uint8 c;		// cache for received UART byte
uint8 serial_ID;	// target ID of UART command
uint8 serial_Type;	// target command type of UART command
int8 serial_rt;		// return value in UART command progress
uint8 serial_Route[MAX_ROUTE_LENGTH];	// route for UART command
uint8 serial_p[3];	// UART Tx buffer
uint8 serial_i;

const uint8 LOCAL_NODE_ID = 5;				// 1 - Master node
												// other address - Slave node

const uint8 APPLICATION_DATA_LENGTH = 6;		// application data length, this contains the figure which will be sent from RF
uint8 demo_data[6] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22};

// interval for periodical events
const uint8 JOIN_INTERVAL = 5;				// interval between each actively Join Request frame, unit is second
const uint8 DATA_INTERVAL = 10;					// interval to actively report data, unit is second
const uint8 DISCOVERY_INTERVAL = 60;			// interval to discovery for the first Slave node, unit is second
const uint8 TOPOLOGY_INTERVAL = 60;				// interval to print current topology
const uint8 RESPONSE_INTERVAL = 180;			// interval to allow all slave to response to discovery broadcast again

// time for waiting for response
const uint8 RESPONSE_TIME_PERIOD = 100;			// time to wait for response after sending a packet, unit is 10 ms
const uint8 SLAVE_BROADCAST_TIME_PERIOD = 50;	// time for slave to wait before send back discovery nonresponse

const uint8 MAX_DATA_REPORT_TIMES = 10;
const uint8 MAX_JOIN_REPORT_TIMES = 10;
const uint8 MAX_REQUEST_TIMES = 1;

////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCT:
//		main
// DESCR:
//		This is the main entry.
// INPUTS:
//		None.
// OUTPUTS:
//		None.
// RETURN:
//		None.
// Considerations:
//		None.
////////////////////////////////////////////////////////////////////////////////////////////////////
void main(void)
{
	Hal_Init();
	Hal_ClearWDT();
	RF_Reset();
	RF_Init();
	RF_Receive_On();
	rf_state.receiving = 1;

	Mesh_Init();

	P2IFG = 0;
        demo_data[3]++;
        
#ifdef SLAVE
        /*** Timer1_A Set-Up ***/
        BCSCTL1 |= DIVA_3;
        TA1CCTL0 = CCIE;
        TA1CCR0 |=  511*DATA_INTERVAL; // 512 -> 1 sec
        TA1CTL |= TASSEL_1+ID_3+ MC_1;
        
        StartDS18b20();
        
        
#endif
        
	while(1)
	{
          
          
		_EINT();

		Hal_ClearWDT();

		if(rf_state.receiving != 1)
		{
			RF_Receive_On();
			rf_state.receiving = 1;
		}

		Mesh_Run();
               

#ifdef MASTER
		// UART command protocol
		// PC to MCU:	0xAA ID Type
		// MCU to PC:	0xCC ID Type	command is send by RF successfully
		//				0xFF ID Type	command is not send because of there is no available route
		if(hal_UART_Flag == TRUE)
		{
			Hal_DelayMs(10);
			flagnum = 0;
			while(Hal_UART_GetOneByte(&c) == TRUE)
			{
				switch(flagnum)
				{
					case 0:
						if(c == 0xAA)
						{
							flagnum ++;
						}
						break;

					case 1:
						flagnum ++;
						serial_ID = c;
						break;

					case 2:
						flagnum ++;
						serial_Type = c;
						serial_rt = Mesh_Find_Best_Path(MASTER_ADDRESS, serial_ID, serial_Route, link_quality);
						if(serial_rt == 0 || serial_rt == -1)
						{
							serial_p[0] = 0xFF;
							serial_p[1] = serial_ID;
							serial_p[2] = serial_Type;
							Hal_UART_Send(hal_UART_Head, 2);
							Hal_UART_Send(serial_p, 3);
							Hal_UART_Send(hal_UART_Tail, 2);
						}
						else
						{
							if(serial_Type == COMMAND_DATA_REQUEST)
							{
								attr.Master_Flag |= DATA_REQUEST_FLAG;
								attr.Data_Request_ID = serial_ID;
							}
							else if(serial_Type == COMMAND_DISCOVERY_REQUEST)
							{
								//attr.Data_Request_Flag = TRUE;
								//attr.Data_Request_ID = serial_ID;
							}
							serial_p[0] = 0xCC;
							serial_p[1] = serial_ID;
							serial_p[2] = serial_Type;
							Hal_UART_Send(hal_UART_Head, 2);
							Hal_UART_Send(serial_p, 3);
							Hal_UART_Send(hal_UART_Tail, 2);
						}
						break;

					default:
						flagnum = 0;
						break;
				}
			}
			hal_UART_Flag = FALSE;
		}
#endif
	}
}
#ifdef SLAVE

#pragma vector=TIMER1_A0_VECTOR
   __interrupt void Timer1_A0 (void) {
    
    GetData();
    
    if(counter==0){
      GetSupplyVoltage();
    }else if(counter==VCC_INTERVAL){
      counter=0;
    }
    counter++;
    GetAnalogValue();
    GetSupplyVoltage();
    StartDS18b20();
    
}


void ow_portsetup() {
	OWPORTDIR |= OWPORTPIN;
	OWPORTOUT |= OWPORTPIN;
	OWPORTREN |= OWPORTPIN;
}

uint16 ReadDS1820(void) {
	uint8 i;
	uint16 byte = 0;
	for (i = 16; i > 0; i--) {
		byte >>= 1;
		if (onewire_read_bit()) {
			byte |= 0x8000;
		}
	}
	return byte;
}

void StartDS18b20(void)
{
        onewire_reset();
	onewire_write_byte(0xcc); // skip ROM command
	onewire_write_byte(0x44); // convert T command
	OW_HI
        // DELAY_MS(750);at least 750 ms for the default 12-bit resolution
}
void GetData(void) {
        uint16 temp;
	onewire_reset();
	onewire_write_byte(0xcc); // skip ROM command
	onewire_write_byte(0xbe); // read scratchpad command
	temp= ReadDS1820();
	demo_data[5]=temp;
	demo_data[4]=temp>>8;
        
}

uint8 onewire_reset() {
	OW_LO
	DELAY_US(480);
	// 480us minimum
	OW_RLS
	DELAY_US(40);
	// slave waits 15-60us
	if (OWPORTIN & OWPORTPIN)
		return 1; // line should be pulled down by slave
	DELAY_US(300);
	// slave TX presence pulse 60-240us
	if (!(OWPORTIN & OWPORTPIN))
		return 2; // line should be "released" by slave
	return 0;
}

//#####################################################################

void onewire_write_bit(uint8 bit) {
//  DELAY_US(1); // recovery, min 1us
	OW_HI
	if (bit) {
		OW_LO
		DELAY_US(5);
		// max 15us
		OW_RLS
			// input
		DELAY_US(56);
	} else {
		OW_LO
		DELAY_US(60);
		// min 60us
		OW_RLS
			// input
		DELAY_US(1);
	}
}

//#####################################################################

uint8 onewire_read_bit() {
	int bit = 0;
//  DELAY_US(1); // recovery, min 1us
	OW_LO
	DELAY_US(5);
	// hold min 1us
	OW_RLS
	DELAY_US(10);
	// 15us window
	if (OWPORTIN & OWPORTPIN) {
		bit = 1;
	}
	DELAY_US(46);
	// rest of the read slot
	return bit;
}

//#####################################################################

void onewire_write_byte(uint8 byte) {
	int i;
	for (i = 0; i < 8; i++) {
		onewire_write_bit(byte & 1);
		byte >>= 1;
	}
}

//#####################################################################

uint8 onewire_read_byte() {
	uint8 i;
	uint8 byte = 0;
	for (i = 0; i < 8; i++) {
		byte >>= 1;
		if (onewire_read_bit())
			byte |= 0x80;
	}
	return byte;
}
void GetSupplyVoltage(void)
{
	uint16 raw_value;
        uint16 voltage;
	// first attempt - measure Vcc/2 with 1.5V reference (Vcc < 3V )
	ADC10CTL0 = SREF_1 | REFON | ADC10SHT_2 | ADC10SR | ADC10ON;
	ADC10CTL1 = INCH_11 | SHS_0 | ADC10DIV_0 | ADC10SSEL_0;
	// start conversion and wait for it
	ADC10CTL0 |= ENC | ADC10SC;
	while (ADC10CTL1 & ADC10BUSY) ;
	// stop conversion and turn off ADC
	ADC10CTL0 &= ~ENC;
	ADC10CTL0 &= ~(ADC10IFG | ADC10ON | REFON);
	raw_value = ADC10MEM;
	// check for overflow
	if (raw_value == 0x3ff) {
		// switch range - use 2.5V reference (Vcc >= 3V)
		ADC10CTL0 = SREF_1 | REF2_5V | REFON | ADC10SHT_2 | ADC10SR | ADC10ON;
		// start conversion and wait for it
		ADC10CTL0 |= ENC | ADC10SC;
		while (ADC10CTL1 & ADC10BUSY) ;
		raw_value = ADC10MEM;
		// end conversion and turn off ADC
		ADC10CTL0 &= ~ENC;
		ADC10CTL0 &= ~(ADC10IFG | ADC10ON | REFON);
		// convert value to mV
		voltage = ((uint32_t)raw_value * 5000) / 1024;
                demo_data[3]=voltage;
                demo_data[2]=voltage>>8;
	} else
		voltage = ((uint32_t)raw_value * 3000) / 1024;
                demo_data[3]=voltage;
                demo_data[2]=voltage>>8;
}
void GetAnalogValue(void)
{
                uint16 raw_value;
		// switch range - use 2.5V reference (Vcc >= 3V)
		//ADC10CTL0 = SREF_1 | REF2_5V | REFON | ADC10SHT_3 | ADC10SR | ADC10ON;
		
                ADC10CTL1 = INCH_3 + ADC10DIV_3 ; // Channel 3, ADC10CLK/4
                ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON; //Vcc & Vss as reference
                ADC10AE0 |= BIT3; //P1.3 ADC option

                // start conversion and wait for it
		ADC10CTL0 |= ENC | ADC10SC;
		while (ADC10CTL1 & ADC10BUSY) ;
		raw_value = ADC10MEM;
		// end conversion and turn off ADC
		ADC10CTL0 &= ~ENC;
		ADC10CTL0 &= ~(ADC10IFG | ADC10ON);
		// convert value to mV
		
                demo_data[1]=raw_value;
                demo_data[0]=raw_value>>8;
	
}
#endif
BCSCTL1 |= DIVA_3;
        TA1CCTL0 = CCIE;
        TA1CCR0 |=  511*DATA_INTERVAL; // 512 -> 1 sec
        TA1CTL |= TASSEL_1+ID_3+ MC_1;
        
