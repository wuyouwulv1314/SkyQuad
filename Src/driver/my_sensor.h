/*
 * =====================================================================================
 *
 *       Filename:  my_sensor.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 21:20:55
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __MY_SENSOR__H__
#define __MY_SENSOR__H__

#include "stm32f10x.h"

#define MPU6050InitDelayms		1000
#define MPU6050InitDelayTresh	(MPU6050InitDelayms * (TIM2Freq / 1000))
#define MPU6050InitDelay_Hang	(MPU6050InitDelayTresh + 2)

////America Hand
enum{Roll,Pitch,Throttle,Yaw,CH5,CH6,CH7};
//enum{CHN_Hand,JAP_Hand,USA_Hand};
////China Hand - Standard
//enum{Yaw,Pitch,Throttle,Roll,CH5,CH6,CH7};
////Japan Hand
//enum{Roll_JAP,Pitch_JAP,Throttle_JAP,Yaw_JAP};
////America Hand
//enum{Roll_USA,Throttle_USA,Pitch_USA,Yaw_USA};
//
enum{XH,XL,YH,YL,ZH,ZL};
enum{ACCEL_XH,ACCEL_XL,ACCEL_YH,ACCEL_YL,ACCEL_ZH,ACCEL_ZL,GYRO_XH,GYRO_XL,GYRO_YH,GYRO_YL,GYRO_ZH,GYRO_ZL};

enum{RL,FL,RR,FR,NumOfOCChannel};
#define NumOfFlyChannel	4

//#define FLZero	300//377
#define MotorZero 0
#define FLZero	MotorZero//377
#define	RLZero	MotorZero//374
#define	RRZero	MotorZero//400
#define	FRZero	MotorZero//333

extern __IO uint16_t gSendDataQueue[];
extern uint16_t gTempGetData[];

extern volatile	uint16_t gMPU6050Counter;
extern volatile	uint16_t gMPU6050InitDelayCounter;

extern volatile uint8_t gMPU6050Swap;
extern volatile uint8_t* gMPU6050DataPointer;
extern uint8_t gMPU6050InitQueue1[];
extern uint8_t gMPU6050InitQueue2[];
extern volatile uint8_t gMPU6050ScanQueue[];

void MPU6050_RawProcess(void);
void calib_sensor(void);

#endif  /*__MY_SENSOR__H__*/
