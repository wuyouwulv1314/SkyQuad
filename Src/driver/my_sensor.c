/*
 * =====================================================================================
 *
 *       Filename:  my_sensor.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 21:20:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "my_sensor.h"
#include "AHRS.h"
#include "global.h"

__IO uint16_t gSendDataQueue[SendDataLength];
uint16_t gTempGetData[SendDataLength];

volatile uint16_t gMPU6050Counter = MPU6050_Hang;
volatile uint16_t gMPU6050InitDelayCounter = MPU6050InitDelay_Hang;

volatile uint8_t gMPU6050Swap = 0;
volatile uint8_t* gMPU6050DataPointer = 0;
uint8_t gMPU6050InitQueue1[MPU6050InitQueueLength1] = MPU6050InitQueueValue1;
uint8_t gMPU6050InitQueue2[MPU6050InitQueueLength2] = MPU6050InitQueueValue2;
volatile uint8_t gMPU6050ScanQueue[(MPU6050ScanQueueLength1 + MPU6050ScanQueueLength2)];


void MPU6050_RawProcess(void)/*{{{*/
{
	if(gMPU6050Swap == 1)
	{
		gMPU6050RawAccel.x = gMPU6050DataPointer[XL] + (gMPU6050DataPointer[XH] << 8);
		gMPU6050RawAccel.y = gMPU6050DataPointer[YL] + (gMPU6050DataPointer[YH] << 8);
		gMPU6050RawAccel.z = gMPU6050DataPointer[ZL] + (gMPU6050DataPointer[ZH] << 8);
	}
	else if(gMPU6050Swap == 2)
	{
		gMPU6050RawGyro.x = gMPU6050DataPointer[XL] + (gMPU6050DataPointer[XH] << 8);
		gMPU6050RawGyro.y = gMPU6050DataPointer[YL] + (gMPU6050DataPointer[YH] << 8);
		gMPU6050RawGyro.z = gMPU6050DataPointer[ZL] + (gMPU6050DataPointer[ZH] << 8);

		//END OF ONE SCAN SEQUENCE
		gControl = 1;
	}
}/*}}}*/
void calib_sensor(void)/*{{{*/
{
	if(gMPU6050RawAccel.z < - 3000)
	{
		gAHRSStat = AHRS_Cali_Accel;//plane placed upside down,goto Accel Cali
		gLEDTurnTreshB = LEDTurnTresh_Mode1;
		gLEDCounterG = LED_Hang;
		LED_G_Set(LED_On);
		gLEDCounterR = LED_Hang;
		LED_R_Set(LED_Off);
		gAHRSCounter = ACCEL_LPF_TRESH + 2;

		//gsPrintfActualLength = sprintf(gsPrintfBuffer,"Accel Calibration\r\n");
		//UARTsendString(Tool_USART0,(uint8_t*)gsPrintfBuffer);
	}
	else
	{
		gAHRSStat = AHRS_Cali_Gyro;
		gLEDTurnTreshG = LEDTurnTresh_Mode2;
		gLEDTurnTreshB = LEDTurnTresh_Mode2;
		gLEDTurnTreshR = LEDTurnTresh_Mode2;
		//gsPrintfActualLength = sprintf(gsPrintfBuffer,"Gyro Calibration\r\n");
		//UARTsendString(Tool_USART0,(uint8_t*)gsPrintfBuffer);
	}


}/*}}}*/
