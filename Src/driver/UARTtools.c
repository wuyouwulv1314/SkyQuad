#include "UARTtools.h"

__IO uint8_t UART_ReceiveCommand[UART_TotalNumber] = {0,0,0,0,0};
__IO int32_t UART_ReceiveParameter[UART_TotalNumber] = {0,0,0,0,0};
__IO uint8_t* UART_SendLastAddr[UART_TotalNumber] = {0,0,0,0,0};
__IO uint8_t* UART_SendCurrentAddr[UART_TotalNumber] = {0,0,0,0,0};
__IO UART_SignType UART_ReceiveSign[UART_TotalNumber] = {UART_SignPlus,UART_SignPlus,UART_SignPlus,UART_SignPlus,UART_SignPlus};
__IO uint8_t Hex2DecR[13];
__IO USART_TypeDef* UART_Name[UART_TotalNumber] = {USART1,USART2,USART3,UART4,UART5};

void usart_send_uint16(UART_NameType Tool_UsARTx,uint16_t d)
{
	USART_TypeDef* UsARTx = (USART_TypeDef*)UART_Name[Tool_UsARTx];
	while((UART_SendCurrentAddr[Tool_UsARTx] != 0) || (USART_GetFlagStatus(UsARTx,USART_FLAG_TXE) != SET));
	USART_SendData(UsARTx,d);
}
void UARTsendN(UART_NameType Tool_UsARTx,uint8_t* sendstart,uint32_t sendlength)
{
	USART_TypeDef* UsARTx = (USART_TypeDef*)UART_Name[Tool_UsARTx];
	while((UART_SendCurrentAddr[Tool_UsARTx] != 0) || (USART_GetFlagStatus(UsARTx,USART_FLAG_TXE) != SET));
	UART_SendCurrentAddr[Tool_UsARTx] = sendstart;
	UART_SendLastAddr[Tool_UsARTx] = sendlength + sendstart - 1;
	USART_SendData(UsARTx,*(UART_SendCurrentAddr[Tool_UsARTx]));
	if(UART_SendCurrentAddr[Tool_UsARTx] == UART_SendLastAddr[Tool_UsARTx])
		UART_SendCurrentAddr[Tool_UsARTx] = 0;
	else
		UART_SendCurrentAddr[Tool_UsARTx] ++;
	USART_ITConfig(UsARTx, USART_IT_TXE, ENABLE);
}

void UARTsendString(UART_NameType Tool_UsARTx,uint8_t* sendstring)
{
	USART_TypeDef* UsARTx = (USART_TypeDef*)UART_Name[Tool_UsARTx];
	while((UART_SendCurrentAddr[Tool_UsARTx] != 0) || (USART_GetFlagStatus(UsARTx,USART_FLAG_TXE) != SET));
	UART_SendCurrentAddr[Tool_UsARTx] = sendstring;
	UART_SendLastAddr[Tool_UsARTx] = sendstring + StringLength(sendstring) - 1;
	USART_SendData(UsARTx,*(UART_SendCurrentAddr[Tool_UsARTx]));
	if(UART_SendCurrentAddr[Tool_UsARTx] == UART_SendLastAddr[Tool_UsARTx])
		UART_SendCurrentAddr[Tool_UsARTx] = 0;
	else
		UART_SendCurrentAddr[Tool_UsARTx] ++;
	USART_ITConfig(UsARTx, USART_IT_TXE, ENABLE);
}

void UARTsendNext(UART_NameType Tool_UsARTx)
{
	USART_TypeDef* UsARTx = (USART_TypeDef*)UART_Name[Tool_UsARTx];
	if(UART_SendCurrentAddr[Tool_UsARTx] != 0)
	{
		USART_SendData(UsARTx,*(UART_SendCurrentAddr[Tool_UsARTx]));
		if(UART_SendCurrentAddr[Tool_UsARTx] == UART_SendLastAddr[Tool_UsARTx])
			UART_SendCurrentAddr[Tool_UsARTx] = 0;
		else
			UART_SendCurrentAddr[Tool_UsARTx] ++;
	}
	else
	{
		USART_ITConfig(UsARTx, USART_IT_TXE, DISABLE);
	}
}

// Recevie Data And Command
void UARTreceiveDnC(UART_NameType Tool_UsARTx,uint8_t receivedata)
{
	if((receivedata >= '0') && (receivedata <= '9'))
	{
		UART_ReceiveParameter[Tool_UsARTx] = (10 * UART_ReceiveParameter[Tool_UsARTx]) + (receivedata - 0x30); 
	}
	else
	if(receivedata == '-')
	{
		UART_ReceiveSign[Tool_UsARTx] = UART_SignMinus;
	}
	else
	{
		UART_ReceiveCommand[Tool_UsARTx] = receivedata;
		if(UART_ReceiveSign[Tool_UsARTx] == UART_SignMinus)
			UART_ReceiveParameter[Tool_UsARTx] = -UART_ReceiveParameter[Tool_UsARTx];
	}
}

uint16_t StringLength(uint8_t *str)
{
	uint16_t len;
	for(len = 0;len < 0xfffb;len ++)
	{
		if(*str == '\0')return len;
		str ++;
	}
	return 0xffff;
}

uint8_t *Hex2Dec(uint32_t hex,uint8_t* result,Hex2Dec_StartType start,Hex2Dec_EndType end)
{
	static uint8_t dec[14];
	uint8_t *s = dec;
	uint8_t a = 0;
	uint8_t i;
	uint32_t ten = 1000000000;
	*s = 0;
	for(i = 1;i < 10;i ++){
		dec[i] = (hex / ten) + 0x30;
		hex %= ten;
		if(dec[i] == '0' && (a == (i - 1)))a ++;
		ten /= 10;
	}
	dec[10] = hex + 0x30;
	switch(end)
	{
		case Hex2Dec_EndEnter:
			dec[11] = '\r';
			dec[12] = '\n';
			i = 2;
			break;
		case Hex2Dec_EndOver:
			dec[11] = '\0';
			i = 1;
			break;
		case Hex2Dec_LooseEnd:
			i = 0;
			break;
		default:break;
	}
	s += a;
	*s = 10 + i - a;
	switch(start)
	{
		case Hex2Dec_WithStart:
			for(i = 0;i < (*s) + 1;i ++)
				result[i] = *(s + i);
			return s;
		case Hex2Dec_LooseStart:
			for(i = 0;i < (*s);i ++)
				result[i] = *(s + i + 1);
			return s;
		default:return s;
	}
}
