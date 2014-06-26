#ifndef __UARTTOOLS_H__
#define __UARTTOOLS_H__

#include "stm32f10x.h"

#define UART_ERROR				0xff
#define UART_TotalNumber	5

#define UART_ReceiveCommandClear(tool_usartx) {UART_ReceiveCommand[(tool_usartx)] = 0;}
#define UART_SetErrorCommand(tool_usartx)			{UART_ReceiveCommand[(tool_usartx)] = UART_ERROR;}
#define Hex2DecIndepedent(hex,end)	{Hex2Dec((hex),(uint8_t*)Hex2DecR,Hex2Dec_WithStart,(end));}

typedef enum
{
	Tool_USART1 = 0,
	Tool_USART2 = 1,
	Tool_USART3 = 2,
	Tool_UART4 = 3,
	Tool_UART5 = 4,
}UART_NameType;

typedef enum
{
	UART_SignPlus,
	UART_SignMinus,
}UART_SignType;

typedef enum
{
	Hex2Dec_EndEnter,
	Hex2Dec_EndOver,
	Hex2Dec_LooseEnd,
}Hex2Dec_EndType;

typedef enum
{
	Hex2Dec_WithStart,
	Hex2Dec_LooseStart,
}Hex2Dec_StartType;

extern __IO uint8_t UART_ReceiveCommand[];
extern __IO int32_t UART_ReceiveParameter[];
extern __IO uint8_t* UART_SendLastAddr[];
extern __IO uint8_t* UART_SendCurrentAddr[];
extern __IO UART_SignType UART_ReceiveSign[];
extern __IO uint8_t Hex2DecR[];
extern __IO USART_TypeDef* UART_Name[];

void usart_send_uint16(UART_NameType Tool_UsARTx,uint16_t d);
void UARTsendN(UART_NameType Tool_UsARTx,uint8_t* sendstart,uint32_t sendlength);
void UARTsendString(UART_NameType Tool_UsARTx,uint8_t* sendstring);
void UARTsendNext(UART_NameType Tool_UsARTx);
void UARTreceiveDnC(UART_NameType Tool_UsARTx,uint8_t receivedata);
uint16_t StringLength(uint8_t *str);
uint8_t *Hex2Dec(uint32_t hex,uint8_t* result,Hex2Dec_StartType start,Hex2Dec_EndType end);

#endif
