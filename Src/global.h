/*
 * =====================================================================================
 *
 *       Filename:  global.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 19:55:43
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __GLOBAL__H__
#define __GLOBAL__H__

#include <stdio.h>
#include <stdint.h>

#include "global_def.h"

#define Tool_USART_BT Tool_USART2

#ifdef Test_BT_USART
extern bool flag_bt_recv;
extern uint16_t data_bt_recv;
#endif

#define FloatStrFormat "%6.2f "
#define IntStrFormat "%6d "
#define FSF FloatStrFormat
#define ISF IntStrFormat

enum{NoneEdge,RiseEdge,FallEdge};

// LED /*{{{*/
#define LED_On		Bit_RESET
#define LED_Off		Bit_SET
#define LED_Hang	(LEDRefreshTresh + 2)

#define LED_R_Set(x)	{GPIO_WriteBit(GPIOA,GPIO_Pin_1,x);}
#define LED_G_Set(x)	{GPIO_WriteBit(GPIOA,GPIO_Pin_7,x);}
#define LED_B_Set(x)	{GPIO_WriteBit(GPIOB,GPIO_Pin_12,x);}

#define LEDOnTimems_Mode1			100
#define LEDOnTimems_Mode2			500
#define LEDOnTimems_ModeOff		1
#define LEDOnTimems_ModeOn		1000
#define LEDTotalTimems				1000
#define LEDTurnTresh_Mode1		(LEDOnTimems_Mode1 * (TIM2Freq / 1000))
#define LEDTurnTresh_Mode2		(LEDOnTimems_Mode2 * (TIM2Freq / 1000))
#define LEDTurnTresh_ModeOff	(LEDOnTimems_ModeOff * (TIM2Freq / 1000))
#define LEDTurnTresh_ModeOn		(LEDOnTimems_ModeOn * (TIM2Freq / 1000))
#define LEDRefreshTresh				(LEDTotalTimems * (TIM2Freq / 1000))
/*}}}*/
#define	TIM2Freq				5000
#define TIM2Period			(1000000 / TIM2Freq)
#define TIM3PWMDutyMax	1000
#define TIM3Period			TIM3PWMDutyMax
#define	TIM3Freq				SystemCoreClock / TIM3Period

#define TIM4PWMDutyMax	1000
#define TIM4Period			TIM4PWMDutyMax
#define	TIM4Freq				SystemCoreClock / TIM4Period

#define Delay1msTresh	(1 * (TIM2Freq / 1000))
#define Delay_Hang		INT16_MAX

#define SendDataLength 8
#define TransferFreq		50
#define	TransferTresh		(TIM2Freq / TransferFreq)
#define TransferHang		(TransferTresh + 2)

#define MPU6050ScanFreq		500
#define MPU6050ScanTresh2	(TIM2Freq / MPU6050ScanFreq)
#define MPU6050ScanTresh1	(MPU6050ScanTresh2 * 80 / 100)
#define MPU6050_Hang			(MPU6050ScanTresh2 + 2)

#define MPU6050Addr					0xD0	// B11010000
#define MPU6050InitStartAddr1		107
#define MPU6050InitQueueLength1		1
#define MPU6050InitQueueValue1		{3}
#define MPU6050InitStartAddr2		25
#define MPU6050InitQueueLength2		4
#define MPU6050InitQueueValue2		{15,0,16,16}
#define MPU6050ScanStartAddr1		59
#define MPU6050ScanQueueLength1		6
#define MPU6050ScanStartAddr2		67
#define MPU6050ScanQueueLength2		6

extern volatile uint16_t gDelayCounter  ;
extern volatile uint16_t gDelayTicks  ;
extern volatile uint16_t gDelayms  ;

extern uint16_t gLEDCounterR ;
extern uint16_t gLEDCounterG ;
extern uint16_t gLEDCounterB ;
extern volatile uint16_t gLEDTurnTreshR ;
extern volatile uint16_t gLEDTurnTreshG ;
extern volatile uint16_t gLEDTurnTreshB ;

extern uint16_t gTransferCounter;
extern uint8_t gSend;
extern volatile uint8_t gControl;
extern uint8_t gsPrintfBuffer[];
extern uint32_t gsPrintfActualLength;
extern uint8_t* gUARTpointer;

extern volatile uint8_t gpwmen;
extern volatile uint8_t gbutton;
extern volatile uint8_t gbuttonedge;

extern bool flag_recv_instruct;
extern int g_check_recv_instruct_cnt;
#endif  /*__GLOBAL__H__*/
