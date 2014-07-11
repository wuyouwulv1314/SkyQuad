/*
 * =====================================================================================
 *
 *       Filename:  my_timer.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 11:49:45
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "my_timer.h"
#include "global.h"
#include "my_i2c.h"
#include "my_sensor.h"

#define CheckRecvInstrctCntThreshold TIM2Freq/2

void TIM_Interrupt_Config(void)/*{{{*/
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	//	TIM_ICInitTypeDef  TIM_ICInitStructure;
	//1KHz timer TIM2
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInitStructure.TIM_Period = TIM2Period;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 71;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	//timer2 interrupt enable
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	//timer2 enable
	TIM_Cmd(TIM2,ENABLE);

}/*}}}*/
void TIM_Motor_Config(void)/*{{{*/
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);/* GPIOB clock enable */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* Time base configuration */
	TIM_TimeBaseInitStructure.TIM_Period = TIM3Period;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseInitStructure.TIM_Period = TIM4Period;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM4, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);

}/*}}}*/
void TIM2_IRQHandler(void)/*{{{*/
{
	//clear flag
	if(TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) == SET)
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);

#if 1
	//LED Heartbeat Green
	if(gLEDCounterG == LED_Hang);
	//		LED_G_Set(LED_On)
	else	gLEDCounterG ++;
	if(gLEDCounterG == gLEDTurnTreshG)
		LED_G_Set(LED_Off)
	else if(gLEDCounterG == LEDRefreshTresh)
	{
		gLEDCounterG = 0;
		LED_G_Set(LED_On)
	}

	//LED Heartbeat Red
	if(gLEDCounterR == LED_Hang);
	//		LED_R_Set(LED_On)
	else	gLEDCounterR ++;
	if(gLEDCounterR == gLEDTurnTreshR)
		LED_R_Set(LED_Off)
	else if(gLEDCounterR == LEDRefreshTresh)
	{
		gLEDCounterR = 0;
		LED_R_Set(LED_On)
	}

	//LED Heartbeat Blue
	if(gLEDCounterB == LED_Hang);
	//		LED_B_Set(LED_On)
	else	gLEDCounterB ++;
	if(gLEDCounterB == gLEDTurnTreshB)
		LED_B_Set(LED_Off)
	else if(gLEDCounterB == LEDRefreshTresh)
	{
		gLEDCounterB = 0;
		LED_B_Set(LED_On)
	}
#endif
	g_check_recv_instruct_cnt ++;
	if(g_check_recv_instruct_cnt == CheckRecvInstrctCntThreshold)
	{
		g_check_recv_instruct_cnt = 0;
		flag_recv_instruct = false;
	}
	//delay ms service
	gDelayCounter ++;
	if(gDelayCounter == Delay1msTresh)
	{
		gDelayCounter = 0;
		if(gDelayTicks < Delay_Hang)gDelayTicks ++;
	}
#if 1
	//printf send once
	if(gTransferCounter == TransferHang);
	else	gTransferCounter ++;
	if(gTransferCounter == TransferTresh)
	{
		gTransferCounter = 0;
		if(gSend == 0)gSend = 1;
	}

	//MPU6050 Data Scan Routine
	if(gMPU6050Counter != MPU6050_Hang)gMPU6050Counter ++;
	if(gMPU6050Counter == MPU6050ScanTresh1)
	{
		gMPU6050Swap = 1;
		gMPU6050DataPointer = gMPU6050ScanQueue;
		IIC_MasterRW(IIC_Stat_Read,MPU6050Addr,MPU6050ScanStartAddr1,MPU6050ScanQueueLength1,gMPU6050DataPointer);
	}
	else if(gMPU6050Counter == MPU6050ScanTresh2)
	{
		gMPU6050Counter = 0;
		gMPU6050Swap = 2;
		gMPU6050DataPointer = gMPU6050ScanQueue + MPU6050ScanQueueLength1;
		IIC_MasterRW(IIC_Stat_Read,MPU6050Addr,MPU6050ScanStartAddr2,MPU6050ScanQueueLength2,gMPU6050DataPointer);
	}
#endif
}/*}}}*/
