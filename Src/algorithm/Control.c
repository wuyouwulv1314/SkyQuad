/*
 * Control.c
 *
 * Created: 2014/3/5 17:32:02
 *  Author: function
 */ 
#include "Control.h"
#include "DataReceiver.h"
#include "my_sensor.h"
#include "AHRS.h"
#include "stm32f10x.h"

_Control_Loop_t Control_OmegaLoop = {\
	{0,0,0},//desired,
	{0,0,0},//actual,
	{0,0,0},//difference,
	{0,0,0},//lastdifference,
	{0,0,0},//integration,
//	{0 * 180 / M_PI,0 * 180 / M_PI,0 * 180 / M_PI},//Kp,
	{0.8 * 180 / M_PI,0.8 * 180 / M_PI,0.8 * 180 / M_PI},//Kp,
	{0,0,0},//Ki,
	{0.1 * 180 / M_PI * Control_Timestep,0.1 * 180 / M_PI * Control_Timestep,0.1 * 180 / M_PI * Control_Timestep},//Kd,
	{1 * M_PI / 180,1 * M_PI / 180,1 * M_PI / 180},//inputscale,
	{0,0,0}};//output;}

_Control_Loop_t Control_AngleLoop = {\
	{0,0,0},//desired,
	{0,0,0},//actual,
	{0,0,0},//difference,
	{0,0,0},//lastdifference,
	{0,0,0},//integration,
//	{0,0,0},//Kp,
	{5,5,10},//Kp,
	{0,0,0},//Ki,
//	{0.05f,0.05f,0.0f},//Ki,
	{0,0,0},//Kd,
	{-30.0f/500.0f,-30.0f/500.0f,0},//inputscale,
	{0,0,0}};//output;}

//Throttle
volatile int16_t Control_ThrottleLast = 0;
//InputNormalize
volatile float Control_ThrottleKp = 0.6;
volatile float Control_ThrottleKd = 0.0;
//PWM Output
volatile int16_t Control_ThrottleDesired = 0;
//#ifdef Test_Control
volatile int16_t gMotor_pulse[NumOfOCChannel];
volatile int16_t gMotor_delta_pitch;
volatile int16_t gMotor_delta_roll;
volatile int16_t gMotor_delta_yaw;
//#endif

void Control_GetThrottleDesired(void)
{
	if(DRChannelBottom(DRDataPointerDone[Throttle]))
	Control_ThrottleDesired = 0;
	else Control_ThrottleDesired = (DRDataPointerDone[Throttle] - DRDataOffset) * Control_ThrottleKp + (DRDataPointerDone[Throttle] - Control_ThrottleLast) * Control_ThrottleKd;
	Control_ThrottleLast = DRDataPointerDone[Throttle];
}

void Control_GetOmegaLoopDesired(void)
{
//	//Single Loop Operation
//	//pitch
//	if(DRChannelMiddle(DRDataPointerDone[Pitch]))
//	Control_OmegaLoop.desired.pitch = 0;
//	else Control_OmegaLoop.desired.pitch = (DRDataPointerDone[Pitch] - DRDataCenter) * Control_OmegaLoop.inputscale.pitch;
//	
//	//Roll
//	if(DRChannelMiddle(DRDataPointerDone[Roll]))
//	Control_OmegaLoop.desired.roll = 0;
//	else Control_OmegaLoop.desired.roll = (DRDataPointerDone[Roll] - DRDataCenter) * Control_OmegaLoop.inputscale.roll;
	
	//Yaw
	if(DRChannelMiddleLock(DRDataPointerDone[Yaw]))
//	Control_OmegaLoop.desired.yaw = 0;
	Control_OmegaLoop.desired.yaw = Control_AngleLoop.output.yaw * Control_OmegaLoop.inputscale.yaw;
	else Control_OmegaLoop.desired.yaw = (DRDataPointerDone[Yaw] - DRDataCenter) * Control_OmegaLoop.inputscale.yaw;
	
	//pitch
	Control_OmegaLoop.desired.pitch = Control_AngleLoop.output.pitch * Control_OmegaLoop.inputscale.pitch;
	
	//roll
	Control_OmegaLoop.desired.roll = Control_AngleLoop.output.roll * Control_OmegaLoop.inputscale.roll;
}

void Control_GetOmegaLoopActual(void)
{
	//Yaw
	Control_OmegaLoop.actual.yaw = gGyro.z;
	
	//Roll
	Control_OmegaLoop.actual.roll = - gGyro.x;
	//Control_OmegaLoop.actual.roll =  gGyro.x;
	
	//Pitch
	Control_OmegaLoop.actual.pitch = gGyro.y;
}

void Control_GetAngleLoopDesired(void)
{
	//pitch
	if(DRChannelMiddle(DRDataPointerDone[Pitch]))
	Control_AngleLoop.desired.pitch = 0;
	else Control_AngleLoop.desired.pitch = (DRDataPointerDone[Pitch] - DRDataCenter) * Control_AngleLoop.inputscale.pitch;
	
	//Roll
	if(DRChannelMiddle(DRDataPointerDone[Roll]))
	Control_AngleLoop.desired.roll = 0;
	else Control_AngleLoop.desired.roll = (DRDataPointerDone[Roll] - DRDataCenter) * Control_AngleLoop.inputscale.roll;
	
	//Yaw
	if(DRChannelMiddleLock(DRDataPointerDone[Yaw]));
	else Control_AngleLoop.desired.yaw = eulerYawActual;
//		Control_AngleLoop.desired.yaw = eulerYawActual;
//	Control_AngleLoop.desired.yaw = 0;
	if(!gpwmen)Control_AngleLoop.desired.yaw = eulerYawActual;
//	else Control_AngleLoop.desired.yaw = (DRDataPointerDone[Yaw] - DRDataCenter) * Control_AngleLoop.inputscale.yaw;
//	else Control_AngleLoop.desired.yaw = eulerYawActual;
}

void Control_GetAngleLoopActual(void)
{
	//Yaw
	Control_AngleLoop.actual.yaw = eulerYawActual;
	
	//Roll
	Control_AngleLoop.actual.roll = eulerPitchActual;
	// BUG XXX 这两种写法好像都可以。。
	//Control_AngleLoop.actual.roll = eulerRollActual;
	
	//Pitch
	Control_AngleLoop.actual.pitch = eulerRollActual;
	//Control_AngleLoop.actual.pitch = eulerPitchActual;
}

void Control_PID(_Control_Loop_t* loop)/*{{{*/
{
	//Yaw
	//difference
	loop->difference.yaw = loop->desired.yaw - loop->actual.yaw;
	//I
	if(loop->integration.yaw > Control_IntegrationMax)loop->integration.yaw = Control_IntegrationMax;
	else if(loop->integration.yaw < - Control_IntegrationMax)loop->integration.yaw = - Control_IntegrationMax;
	else loop->integration.yaw += loop->difference.yaw * Control_Timestep;
	//PID clac
	loop->output.yaw =  - (loop->difference.yaw * loop->Kp.yaw\
	+ (loop->difference.yaw - loop->lastdifference.yaw) * loop->Kd.yaw / Control_Timestep\
	+ loop->integration.yaw * loop->Ki.yaw);
	//D use
	loop->lastdifference.yaw = loop->difference.yaw;
	
	//Pitch
	//difference
	loop->difference.pitch = loop->desired.pitch - loop->actual.pitch;
	//I
	if(loop->integration.pitch > Control_IntegrationMax)loop->integration.pitch = Control_IntegrationMax;
	else if(loop->integration.pitch < - Control_IntegrationMax)loop->integration.pitch = - Control_IntegrationMax;
	else loop->integration.pitch += loop->difference.pitch * Control_Timestep;
	//PID clac
	loop->output.pitch =  - (loop->difference.pitch * loop->Kp.pitch\
	+ (loop->difference.pitch - loop->lastdifference.pitch) * loop->Kd.pitch / Control_Timestep\
	+ loop->integration.pitch * loop->Ki.pitch);
	//D use
	loop->lastdifference.pitch = loop->difference.pitch;
	
	//Roll
	//difference
	loop->difference.roll = loop->desired.roll - loop->actual.roll;
	//I
	if(loop->integration.roll > Control_IntegrationMax)loop->integration.roll = Control_IntegrationMax;
	else if(loop->integration.roll < - Control_IntegrationMax)loop->integration.roll = - Control_IntegrationMax;
	else loop->integration.roll += loop->difference.roll * Control_Timestep;
	//PID clac
	loop->output.roll =  - (loop->difference.roll * loop->Kp.roll\
	+ (loop->difference.roll - loop->lastdifference.roll) * loop->Kd.roll / Control_Timestep\
	+ loop->integration.roll * loop->Ki.roll);
	//D use
	loop->lastdifference.roll = loop->difference.roll;
}/*}}}*/

void Control_ClearI(_Control_Loop_t* loop)
{
	loop->integration.yaw = 0;
	loop->integration.pitch = 0;
	loop->integration.roll = 0;
}


void Control_Calc_PWM(_Control_Loop_t* loop)/*{{{*/
{
	int16_t yaw = 0;
	int16_t roll = 0;
	int16_t pitch = 0;
	int16_t throttle = 0;
	int16_t temp;
	
	int16_t realPWM[NumOfOCChannel];
	
	//Yaw
	//	for debug,not consider Yaw
	if(loop->output.yaw > Control_YawOutputMax)yaw = Control_YawOutputMax;
	else if(loop->output.yaw < - Control_YawOutputMax)yaw = - Control_YawOutputMax;
	else yaw = loop->output.yaw;
	
	//Roll
	if(loop->output.roll > Control_RollOutputMax)roll = Control_RollOutputMax;
	else if(loop->output.roll < - Control_RollOutputMax)roll = - Control_RollOutputMax;
	else roll = loop->output.roll;
	
	//Pitch
	if(loop->output.pitch > Control_PitchOutputMax)pitch = Control_PitchOutputMax;
	else if(loop->output.pitch < - Control_PitchOutputMax)pitch = - Control_PitchOutputMax;
	else pitch = loop->output.pitch;
	
	//Throttle
	if(Control_ThrottleDesired > Control_ThrottleOutputMax)throttle = Control_ThrottleOutputMax;
	else if(Control_ThrottleDesired < - Control_ThrottleOutputMax)throttle = - Control_ThrottleOutputMax;
	else throttle = Control_ThrottleDesired;
	
	// yaw 弄反了
//	yaw = -yaw;
	temp= roll;
	roll = -pitch;
	pitch=temp;
	//FL
	//realPWM[FL] = throttle + yaw + roll + pitch;
	realPWM[FL] = throttle + yaw + roll + pitch + FLZero;
	if(realPWM[FL] > Control_TotalOutputMax)realPWM[FL] = Control_TotalOutputMax;
	//else if(realPWM[FL] < 0)realPWM[FL] = 0;
	else if(realPWM[FL] < 0)realPWM[FL] = FLZero;
	
	//FR
	//realPWM[FR] = throttle - yaw - roll + pitch;
	realPWM[FR] = throttle - yaw - roll + pitch + FRZero;
	if(realPWM[FR] > Control_TotalOutputMax)realPWM[FR] = Control_TotalOutputMax;
	//else if(realPWM[FR] < 0)realPWM[FR] = 0;
	else if(realPWM[FR] < 0)realPWM[FR] = FRZero;
	
	//RL
	//realPWM[RL] = throttle - yaw + roll - pitch;
	realPWM[RL] = throttle - yaw + roll - pitch + RLZero;
	if(realPWM[RL] > Control_TotalOutputMax)realPWM[RL] = Control_TotalOutputMax;
	//else if(realPWM[RL] < 0)realPWM[RL] = 0;
	else if(realPWM[RL] < 0)realPWM[RL] = RLZero;
	
	//RR
	realPWM[RR] = throttle + yaw - roll - pitch + RRZero;
	if(realPWM[RR] > Control_TotalOutputMax)realPWM[RR] = Control_TotalOutputMax;
	else if(realPWM[RR] < 0)realPWM[RR] = RRZero;
	
//#ifdef Test_Control
	gMotor_pulse[FL]=realPWM[FL] ;//+ FLZero;
	gMotor_pulse[FR]=realPWM[FR] ;//+ FRZero;
	gMotor_pulse[RL]=realPWM[RL] ;//+ RLZero;
	gMotor_pulse[RR]=realPWM[RR] ;//+ RRZero;
	gMotor_delta_pitch=2*pitch;
	gMotor_delta_roll=2*roll;
	gMotor_delta_yaw=2*yaw;
//#endif
}/*}}}*/
void Control_Output_PWM(void)/*{{{*/
{
	if(gpwmen)
	{
#ifdef Control_Default
#ifndef NoMotorOut
	MOTOR_OUT_FL(gMotor_pulse[FL]);
	MOTOR_OUT_FR(gMotor_pulse[FR]);
	MOTOR_OUT_RL(gMotor_pulse[RL]);
	MOTOR_OUT_RR(gMotor_pulse[RR]);

//	MOTOR_OUT_FL(realPWM[FL] + FLZero);
//	MOTOR_OUT_FR(realPWM[FR] + FRZero);
//	MOTOR_OUT_RL(realPWM[RL] + RLZero);
//	MOTOR_OUT_RR(realPWM[RR] + RRZero);
#endif
#endif
#ifdef Control_TestMotor
		Control_TestMotors();
#endif
	}
	else
	{
		MOTOR_OUT_FL(0);
		MOTOR_OUT_FR(0);
		MOTOR_OUT_RL(0);
		MOTOR_OUT_RR(0);
	}
}/*}}}*/
void Control_TestMotors(void)/*{{{*/
{
	if((DRDataPointerDone[CH6] > 1200) && (DRDataPointerDone[CH6] < 1400))
	{
		MOTOR_OUT_FL(Control_ThrottleDesired);
		MOTOR_OUT_FR(0);
		MOTOR_OUT_RL(0);
		MOTOR_OUT_RR(0);
	}
	else if((DRDataPointerDone[CH6] > 1400) && (DRDataPointerDone[CH6] < 1600))
	{
		MOTOR_OUT_FL(0);
		MOTOR_OUT_FR(0);
		MOTOR_OUT_RL(Control_ThrottleDesired);
		MOTOR_OUT_RR(0);
	}
	else if((DRDataPointerDone[CH6] > 1600) && (DRDataPointerDone[CH6] < 1800))
	{
		MOTOR_OUT_FL(0);
		MOTOR_OUT_FR(0);
		MOTOR_OUT_RL(0);
		MOTOR_OUT_RR(Control_ThrottleDesired);
	}
	else if((DRDataPointerDone[CH6] > 1800) && (DRDataPointerDone[CH6] < 2000))
	{
		MOTOR_OUT_FL(0);
		MOTOR_OUT_FR(Control_ThrottleDesired);
		MOTOR_OUT_RL(0);
		MOTOR_OUT_RR(0);
	}
	else
	{
		MOTOR_OUT_FL(0);
		MOTOR_OUT_FR(0);
		MOTOR_OUT_RL(0);
		MOTOR_OUT_RR(0);
	}
}/*}}}*/
