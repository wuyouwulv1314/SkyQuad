/*
 * AHRS.c
 *
 * Created: 2014/2/23 13:24:44
 *  Author: function
 */ 
#include "AHRS.h"
#include "MadgwickAHRS.h"
#include "filter.h"
#include "Control.h"
#include "my_control.h"
#include <math.h>
extern _Control_Loop_t my_loop;

uint8_t gAHRSStat = AHRS_WaitInit;
volatile uint32_t gAHRSCounter = 0;

volatile uint8_t gGyroCaliPrintEN = 1;
volatile uint8_t gAccelCaliPrintEN = 0;

volatile uint32_t gGyroCaliPrintCounter = 0;
volatile uint32_t gAccelCaliPrintCounter = 0;

volatile uint8_t gRunningPrint = 1;

//uint8_t gAHRSSend[] = "FUN +0     +0     +0    \r\n";

_3Axis16i_t gMPU6050RawGyro;
_3Axis32i_t gMPU6050RawGyroStoredFilterValues;
_3Axis16i_t gMPU6050LPFGyro;
_3Axis16i_t gMPU6050BiasGyro;
_3Axisf_t	gGyro;
_3Axisf_t	gGyroTmp;
_3Axisf_t	gAccelTmp;

_3Axis16i_t gMPU6050RawAccel;
_3Axis32i_t gMPU6050RawAccelStoredFilterValues = {0,0,0};
_3Axis16i_t gMPU6050LPFAccel;
_3Axis16i_t gMPU6050BiasAccel = {0,0,0};
_3Axisf_t	gAccel;

float eulerRollActual;
float eulerPitchActual;
float eulerYawActual;

void AHRS_AccelIIRLPFilter(void)
{
	gMPU6050LPFAccel.x = iirLPFilterSingle(gMPU6050RawAccel.x, IMU_ACC_IIR_LPF_ATT_FACTOR, &(gMPU6050RawAccelStoredFilterValues.x));
	gMPU6050LPFAccel.y = iirLPFilterSingle(gMPU6050RawAccel.y, IMU_ACC_IIR_LPF_ATT_FACTOR, &(gMPU6050RawAccelStoredFilterValues.y));
	gMPU6050LPFAccel.z = iirLPFilterSingle(gMPU6050RawAccel.z, IMU_ACC_IIR_LPF_ATT_FACTOR, &(gMPU6050RawAccelStoredFilterValues.z));
}

void AHRS_GyroIIRLPFilter(void)
{
	gMPU6050LPFGyro.x = iirLPFilterSingle(gMPU6050RawGyro.x, IMU_GYRO_IIR_LPF_ATT_FACTOR, &(gMPU6050RawGyroStoredFilterValues.x));
	gMPU6050LPFGyro.y = iirLPFilterSingle(gMPU6050RawGyro.y, IMU_GYRO_IIR_LPF_ATT_FACTOR, &(gMPU6050RawGyroStoredFilterValues.y));
	gMPU6050LPFGyro.z = iirLPFilterSingle(gMPU6050RawGyro.z, IMU_GYRO_IIR_LPF_ATT_FACTOR, &(gMPU6050RawGyroStoredFilterValues.z));
}

void AHRS_GyroBiasSet(void)
{
	gMPU6050BiasGyro = gMPU6050LPFGyro;
}

#define Half_Sqrt2 0.7071067811865475
void AHRS_Normalize(void)
{
	gGyroTmp.x = (gMPU6050RawGyro.x - gMPU6050BiasGyro.x) * IMU_DEG_PER_LSB_CFG * M_PI / 180;
	gGyroTmp.y = (gMPU6050RawGyro.y - gMPU6050BiasGyro.y) * IMU_DEG_PER_LSB_CFG * M_PI / 180;
	gGyro.z = (gMPU6050RawGyro.z - gMPU6050BiasGyro.z) * IMU_DEG_PER_LSB_CFG * M_PI / 180;
	gAccelTmp.x = (gMPU6050LPFAccel.x - gMPU6050BiasAccel.x) * MPU6050_G_PER_LSB_8 * 1.0f;
	gAccelTmp.y = (gMPU6050LPFAccel.y - gMPU6050BiasAccel.y) * MPU6050_G_PER_LSB_8 * 1.0f;
	gAccel.z = (gMPU6050LPFAccel.z - gMPU6050BiasAccel.z) * MPU6050_G_PER_LSB_8 * 1.0f;

	gGyro.x = (gGyroTmp.x + gGyroTmp.y)*Half_Sqrt2;
	gGyro.y = (-gGyroTmp.x + gGyroTmp.y)*Half_Sqrt2;
	gAccel.x = (gAccelTmp.x + gAccelTmp.y)*Half_Sqrt2;
	gAccel.y = (-gAccelTmp.x + gAccelTmp.y)*Half_Sqrt2;
}

void AHRS_GetEulerRPY(void)
{
	float gx, gy, gz; // estimated gravity direction

	gx = 2 * (q1*q3 - q0*q2);
	gy = 2 * (q0*q1 + q2*q3);
	gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	eulerYawActual = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 180 / M_PI;
	eulerRollActual = atan(gx / sqrt(gy*gy + gz*gz)) * 180 / M_PI;
	eulerPitchActual = atan(gy / sqrt(gx*gx + gz*gz)) * 180 / M_PI;

	eulerYawActual = - eulerYawActual;
	eulerRollActual = - eulerRollActual;
}

void BYYS_Quat2Angle(void)
{
	float c32;
	float c33;
	float c31;
	float c11;
	float c21;

	c32=2*(q2*q3+q0*q1);
	c33=q0*q0-q1*q1-q2*q2+q3*q3;
	c31=2*(q1*q3-q0*q2);
	c11=q0*q0+q1*q1-q2*q2-q3*q3;
	c21=2*(q1*q2+q0*q3);
	
	eulerYawActual=atan2(c21,c11) * 180 / M_PI;  //-PI~PI
	eulerPitchActual=asin(-c31) * 180 / M_PI;	 //-PI/2~PI/2
	eulerRollActual=atan2(c32,c33) * 180 / M_PI;	//-PI~PI

	if(eulerYawActual<0) eulerYawActual+=360;
}

void Accl_Cali_Print(void)
{
	gsPrintfActualLength = sprintf(gsPrintfBuffer,"funAccel%d:	%d	%d	%d\r\n",\
						gAccelCaliPrintCounter,gMPU6050LPFAccel.x,gMPU6050LPFAccel.y,gMPU6050LPFAccel.z);
	gUARTpointer = gsPrintfBuffer;
}

void Gyro_Cali_Print(void)
{
	gsPrintfActualLength = sprintf(gsPrintfBuffer,"funGyro:	%d	%d	%d\r\n",\
							gMPU6050LPFGyro.x,gMPU6050LPFGyro.y,gMPU6050LPFGyro.z);
	gUARTpointer = gsPrintfBuffer;
}

void Running_Print(void)
{
#ifdef Test_Tune_PID
	gsPrintfActualLength = sprintf(gsPrintfBuffer,"pid %d %d %d  "FSF FSF FSF ISF ISF ISF "\r\n",
		adjustPID_mode,isTune,isCompareOK,my_loop.Kp.pitch,my_loop.Ki.pitch,my_loop.Kd.pitch,
			gMotor_delta_pitch,gMotor_delta_roll,gMotor_delta_yaw);
	gUARTpointer = gsPrintfBuffer;
#endif
#ifndef Test_Tune_PID
#if 1
	gsPrintfActualLength = sprintf(gsPrintfBuffer,"control  "FSF FSF FSF FSF FSF FSF ISF ISF ISF "\r\n",
			eulerPitchActual,eulerRollActual,eulerYawActual,
			my_loop.desired.pitch,my_loop.desired.roll,my_loop.desired.yaw,
			gMotor_delta_pitch,gMotor_delta_roll,gMotor_delta_yaw);
	gUARTpointer = gsPrintfBuffer;
#endif
#endif
#if 0
	gsPrintfActualLength = sprintf(gsPrintfBuffer,"control  "FSF FSF FSF ISF ISF ISF ISF ISF ISF ISF "\r\n",
			eulerPitchActual,eulerRollActual,eulerYawActual,
			gMotor_pulse[FL],gMotor_pulse[FR],gMotor_pulse[RL],gMotor_pulse[RR],
			gMotor_delta_pitch,gMotor_delta_roll,gMotor_delta_yaw);
	gUARTpointer = gsPrintfBuffer;
#endif
#if 0
	float tempf = (float)Control_ThrottleDesired / (float)Control_ThrottleOutputMax * 10.0f;
	gsPrintfActualLength = sprintf(gsPrintfBuffer,\
						"==========================\r\n"\
//						"Gyro:	X=%4.4f	Y=%4.4f	Z=%4.4f\r\n"\
//						"Accel:	X=%4.4f	Y=%4.4f	Z=%4.4f\r\n"\
//						"Gyroint:	X=%4.4f	Y=%4.4f	Z=%4.4f\r\n"\
	
						"Eular:	X=%4.4f	Y=%4.4f	Z=%4.4f\r\n"\
//						"Quad:	q0 = %4.4ff,q1 = %4.4ff,q2 = %4.4ff,q3 = %4.4ff\r\n"\
	
//						"ThrottleInput=%d\r\n"\
	
//						"OmegaInput:	Pitch=%4.4f	Roll=%4.4f	Yaw=%4.4f\r\n"\
	
						"AngleInput:	Pitch=%4.4f	Roll=%4.4f	Yaw=%4.4f\r\n"\
	
//						"OmegaOut:	Pitch=%d	Roll=%d	Yaw=%d\r\n"\
	
						"AngleOut:	Pitch=%d	Roll=%d	Yaw=%d\r\n"\
	
						"Motor On=%d\r\n",
//						Control_OmegaLoop.actual.pitch,Control_OmegaLoop.actual.roll,Control_OmegaLoop.actual.yaw,
//						gGyro.x,gGyro.y,gGyro.z,
//						gAccel.x,gAccel.y,gAccel.z,
//						gtestpitchint,gtestrollint,gtestyawint,
						eulerPitchActual,eulerRollActual,eulerYawActual,
//						q0, q1, q2, q3,
//						Control_ThrottleDesired,
//						Control_OmegaLoop.desired.pitch,Control_OmegaLoop.desired.roll,Control_OmegaLoop.desired.yaw,
						Control_AngleLoop.desired.pitch,Control_AngleLoop.desired.roll,Control_AngleLoop.desired.yaw,
//						Control_OmegaLoop.output.pitch,Control_OmegaLoop.output.roll,Control_OmegaLoop.output.yaw,
						Control_AngleLoop.output.pitch,Control_AngleLoop.output.roll,Control_AngleLoop.output.yaw,
//						Control_RollOmegaDesired,Control_PitchOmegaDesired,Control_ThrottleLast,Control_YawOmegaDesired,
						gpwmen);

//#ifdef Control_Default
//	gsPrintfActualLength = sprintf(gsPrintfBuffer,\
//	"%3.2f	%3.2f	%1.2f	%2.2f	%2.2f\r\n",
//	Control_AngleLoop.actual.roll,Control_AngleLoop.actual.pitch,tempf,Control_AngleLoop.desired.roll,Control_AngleLoop.desired.pitch);
//#endif
//#ifdef Control_TestMotor
//	gsPrintfActualLength = sprintf(gsPrintfBuffer,\
//	"%d\r\n",
//	Control_ThrottleDesired);
//#endif
	
	gUARTpointer = gsPrintfBuffer;
#endif
}
