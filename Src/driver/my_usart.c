/*
 * =====================================================================================
 *
 *       Filename:  my_uart.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 20:38:59
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "my_usart.h"
#include "stm32f10x.h"
#include "UARTtools.h"
#include "DataReceiver.h"
#include <stdio.h>

void BT_IO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);			

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);			
	//GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
}
void BT_USART_Config(void)/*{{{*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				   //PA10为串口输入RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				   //PA9为串口输出TX 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		   //口线翻转速度为2MHz	 最大波特率只需115.2k，那么用2M的GPIO的引脚速度就够了，既省电也噪声小
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;	 //初始化串口设置
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART2,&USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate = 115200;				   //设置串口参数
	USART_InitStructure.USART_WordLength =USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode =USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART2,&USART_InitStructure);

	USART_Cmd(USART2,ENABLE);

	USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

}/*}}}*/
void USART_Config(void)/*{{{*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				   //PA10为串口输入RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				   //PA9为串口输出TX 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;		   //口线翻转速度为2MHz	 最大波特率只需115.2k，那么用2M的GPIO的引脚速度就够了，既省电也噪声小
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;	 //初始化串口设置
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1,&USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate = 115200;				   //设置串口参数
	USART_InitStructure.USART_WordLength =USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode =USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);

	USART_Cmd(USART1,ENABLE);

	USART_ITConfig(USART1,USART_IT_TXE,ENABLE);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

}/*}}}*/

void USART1_IRQHandler(void)/*{{{*/
{
	uint16_t USART1_RxData;
	uint8_t temp8 = 0;
	if(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == SET)
	{
		temp8 = 1;
		if((UART_SendCurrentAddr[Tool_USART1] != 0))
		{
			//user process
			UARTsendNext(Tool_USART1);
		}
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET)
	{
		temp8 = 1;
		USART1_RxData = USART_ReceiveData(USART1);
		//user process
		//loop
//		USART_SendData(USART1,USART1_RxData);
		DRReceiver(USART1_RxData);
//		UARTreceiveDnC(Tool_USART1,USART1_RxData);
	}
	if(temp8 == 0)
	{
		//user process
		USART_ReceiveData(USART1);
		UART_SetErrorCommand(Tool_USART1);
	}
}/*}}}*/
void USART2_IRQHandler(void)/*{{{*/
{
	uint16_t USART2_RxData;
	uint8_t temp8 = 0;
	if(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == SET)
	{
		temp8 = 1;
		if((UART_SendCurrentAddr[Tool_USART2] != 0))
		{
			//user process
			UARTsendNext(Tool_USART2);
		}
		else
		{
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
	}
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE) == SET)
	{
		temp8 = 1;
		USART2_RxData = USART_ReceiveData(USART2);
#ifdef Test_BT_USART
		flag_bt_recv=true;
		data_bt_recv=USART2_RxData;
		usart_send_uint16(Tool_USART2,data_bt_recv);
#endif
		//user process
		//loop
//		USART_SendData(USART1,USART1_RxData);
		DRReceiver(USART2_RxData);
//		UARTreceiveDnC(Tool_USART1,USART1_RxData);
	}
	if(temp8 == 0)
	{
		//user process
		USART_ReceiveData(USART2);
		UART_SetErrorCommand(Tool_USART2);
	}
}/*}}}*/

/*******************************************************************************
* Function Name  : int fputc(int ch, FILE *f)
* Description    : Retargets the C library printf function to the USART.printf重定向
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
  /* Write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while(!(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET))
  {
  }

  return ch;
}

/*******************************************************************************
* Function Name  : int fgetc(FILE *f)
* Description    : Retargets the C library printf function to the USART.fgetc重定向
* Input          : None
* Output         : None
* Return         : 读取到的字符
*******************************************************************************/
int fgetc(FILE *f)
{
  /* Loop until received a char */
  while(!(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET))
  {
  }
  
    /* Read a character from the USART and RETURN */
  return (USART_ReceiveData(USART1));
}

