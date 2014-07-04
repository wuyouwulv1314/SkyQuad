/*
 * =====================================================================================
 *
 *       Filename:  my_control.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/18 10:36:40
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "my_control.h"
#include "AHRS.h"
#include "DataReceiver.h"

//#define MyKp  25
#define MyKp  15
#define MyKi 0
//#define MyKd 20
#define MyKd 6

//#define RC_Scale 0.2
#define RC_Scale 0.075
//#define RC_Scale_Throttle 0.8
#define RC_Scale_Throttle 1
_Control_Loop_t my_loop={
	{0,0,0},//desired,
	{0,0,0},//actual,
	{0,0,0},//difference,
	{0,0,0},//lastdifference,
	{0,0,0},//integration,
	{MyKp,MyKp,MyKp},//kp,
	{MyKi,MyKi,MyKi},//ki,
	{MyKd,MyKd,MyKd*0.2},//kd,
	{RC_Scale,RC_Scale,RC_Scale*0.006},//inputscale,
	//{1 * M_PI / 180,1 * M_PI / 180,1 * M_PI / 180},//inputscale,
	{0,0,0}//output;}
};

enum AdjustPIDMode adjustPID_mode;
bool isCompareOK=false;
bool isTune=false;

void get_attitude_actual(_Control_Loop_t * loop)/*{{{*/
{
//	static float lastEulerYawActual=0.0f;
	loop->actual.pitch=eulerPitchActual;
	loop->actual.roll =eulerRollActual;
//#define Yaw180MaxThrehold 160
//	if((eulerYawActual < -Yaw180MaxThrehold) &&(lastEulerYawActual > Yaw180MaxThrehold))
//			eulerYawActual +=360;
//	else if((eulerYawActual > Yaw180MaxThrehold) &&(lastEulerYawActual < -Yaw180MaxThrehold))
//		eulerYawActual -=360;
	loop->actual.yaw=eulerYawActual;
//	lastEulerYawActual = eulerYawActual;
	//loop->actual.yaw  =eulerYawActual;
}/*}}}*/
int16_t rc_throttle,rc_pitch,rc_roll,rc_yaw;
void get_remote_control_desired(_Control_Loop_t * loop)/*{{{*/
{
	int16_t temp;

	temp = DRDataPointerDone[Throttle];
	rc_throttle = (DRChannelBottom(temp))? 0:(temp- DRDataOffset)*RC_Scale_Throttle;
#ifdef Test_Tune_PID
	if( rc_throttle == 0) // XXX 没有油门时，认为不让电机输出
		gpwmen = 0;
#endif

#define ReversePitchControl -1
#define ReverseRollhControl -1
#define ReverseYawhControl -1
	temp = DRDataPointerDone[Pitch];
	rc_pitch = (DRChannelMiddle(temp))? 0:(temp-DRDataCenter)*ReversePitchControl;

	temp = DRDataPointerDone[Roll];
	rc_roll = (DRChannelMiddle(temp))? 0:(temp-DRDataCenter)*ReverseRollhControl;

	temp = DRDataPointerDone[Yaw];
	rc_yaw = (DRChannelMiddleLock(temp))? 0:(temp-DRDataCenter)*ReverseYawhControl;

	loop->desired.pitch = - rc_pitch*loop->inputscale.pitch;
	loop->desired.roll  = - rc_roll *loop->inputscale.roll;
	if(gpwmen)//加油门起来后，直接用当前角度作为 desired.yaw.这样才能锁航
	{
		//如果rc_yaw不为0，应调整 desired yaw.注意rc_yaw是角速度，所以inputscale.yaw要保护时间dt在里面，所以yaw 方向的inputscale比其他两个方向的小很多。
		loop->desired.yaw += rc_yaw * loop->inputscale.yaw;
		//期望的角度也应该限制在 -180 ~ +180 之间
		if(loop->desired.yaw > 180)
			loop->desired.yaw -=360;
		else if(loop->desired.yaw < -180)
			loop->desired.yaw +=360;
	}
	else
		loop->desired.yaw   = eulerYawActual;
}/*}}}*/
void calc_pid(_Control_Loop_t * loop)/*{{{*/
{
	//Yaw
	//difference
	loop->difference.yaw = loop->desired.yaw - loop->actual.yaw;
	if(loop->difference.yaw < -180)
		loop->difference.yaw +=360;
	else if(loop->difference.yaw > 180)
		loop->difference.yaw -=360;
	//I
	if(loop->integration.yaw > Control_IntegrationMax)loop->integration.yaw = Control_IntegrationMax;
	else if(loop->integration.yaw < - Control_IntegrationMax)loop->integration.yaw = - Control_IntegrationMax;
	else loop->integration.yaw += loop->difference.yaw * Control_Timestep;
	//PID clac
	loop->output.yaw =   (loop->difference.yaw * loop->Kp.yaw\
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
	loop->output.pitch =   (loop->difference.pitch * loop->Kp.pitch\
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
	loop->output.roll =   (loop->difference.roll * loop->Kp.roll\
	+ (loop->difference.roll - loop->lastdifference.roll) * loop->Kd.roll / Control_Timestep\
	+ loop->integration.roll * loop->Ki.roll);
	//D use
	loop->lastdifference.roll = loop->difference.roll;

}/*}}}*/
void calc_motor_pwm(_Control_Loop_t * loop)/*{{{*/
{
	int16_t yaw = 0;
	int16_t roll = 0;
	int16_t pitch = 0;
	int16_t throttle = 0;
	//int16_t temp;
	
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
	if(rc_throttle > Control_ThrottleOutputMax)throttle = Control_ThrottleOutputMax;
	else if(rc_throttle < - Control_ThrottleOutputMax)throttle = - Control_ThrottleOutputMax;
	else throttle = rc_throttle;
	
	// yaw 弄反了
//	yaw = -yaw;
//	temp= roll;
//	roll = - roll;
//	pitch = - pitch;
//	pitch=temp;
	//FL
	realPWM[FL] = throttle - yaw + roll + pitch;
	if(realPWM[FL] > Control_TotalOutputMax)realPWM[FL] = Control_TotalOutputMax;
	else if(realPWM[FL] < 0)realPWM[FL] = 0;
	
	//FR
	realPWM[FR] = throttle + yaw - roll + pitch;
	if(realPWM[FR] > Control_TotalOutputMax)realPWM[FR] = Control_TotalOutputMax;
	else if(realPWM[FR] < 0)realPWM[FR] = 0;
	
	//RL
	realPWM[RL] = throttle + yaw + roll - pitch;
	if(realPWM[RL] > Control_TotalOutputMax)realPWM[RL] = Control_TotalOutputMax;
	else if(realPWM[RL] < 0)realPWM[RL] = 0;
	
	//RR
	realPWM[RR] = throttle - yaw - roll - pitch;
	if(realPWM[RR] > Control_TotalOutputMax)realPWM[RR] = Control_TotalOutputMax;
	else if(realPWM[RR] < 0)realPWM[RR] = 0;
	
//#ifdef Test_Control
	gMotor_pulse[FL]=realPWM[FL] + FLZero;
	gMotor_pulse[FR]=realPWM[FR] + FRZero;
	gMotor_pulse[RL]=realPWM[RL] + RLZero;
	gMotor_pulse[RR]=realPWM[RR] + RRZero;
	gMotor_delta_pitch=2*pitch;
	gMotor_delta_roll=2*roll;
	gMotor_delta_yaw=2*yaw;
//#endif
}/*}}}*/
void output_motor_pwm(void)
{
	if(gpwmen)
	{
#ifndef NoMotorOut
		MOTOR_OUT_FL(gMotor_pulse[FL]);
		MOTOR_OUT_FR(gMotor_pulse[FR]);
		MOTOR_OUT_RL(gMotor_pulse[RL]);
		MOTOR_OUT_RR(gMotor_pulse[RR]);
#endif
	}
	else
	{
		MOTOR_OUT_FL(0);
		MOTOR_OUT_FR(0);
		MOTOR_OUT_RL(0);
		MOTOR_OUT_RR(0);
	}
}
