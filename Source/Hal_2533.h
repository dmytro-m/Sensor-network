#ifndef _HAL_2533_H
#define _HAL_2533_H

#include "Config.h"

#define MAX_UART_BUFFER_SIZE	32

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Variable Declaration
//
////////////////////////////////////////////////////////////////////////////////////////////////////
extern uint8 hal_SystemClock;
extern uint8 hal_KeyPressed;
extern uint8 hal_UART_RxBuffer[];
extern uint8 hal_UART_RxHead;
extern uint8 hal_UART_RxTail;
extern uint8 hal_UART_Flag;
extern const uint8 hal_UART_Head[];
extern const uint8 hal_UART_Tail[];

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Function Declaration
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void Hal_Init(void);
void Hal_ClearWDT(void);
void Hal_UCS_Init(void);
void Hal_Timer_Init(void);
void Hal_Timer_Start(void);
void Hal_Timer_Stop(void);
void Hal_DelayUs(uint16 us);
void Hal_DelayMs(uint16 ms);
void Hal_Button_Init(void);
void Hal_Led_Init(void);
void Hal_Led_Open(uint8 num);
void Hal_Led_Close(uint8 num);
void Hal_Led_Toggle(uint8 num);
void Hal_UART_Init(void);
void Hal_UART_Send(const uint8 *p, uint8 length);
uint8 Hal_UART_GetOneByte(uint8 *ch);
void Hal_SPI_Init(void);
uint8 Hal_SPI_Out(uint8 data);
uint8 Hal_SPI_In(void);
uint8 Hal_SPI_SendByte(uint8 cmd);
uint8 Hal_SPI_Send(uint8 addr, const uint8 *data, uint8 length);
uint8 Hal_SPI_Receive(uint8 addr, uint8 *data, uint8 length);
void Hal_GDO_Init(void);

#endif
