/*
 * Control.h
 *
 * Created: 2014/3/5 17:32:17
 *  Author: function
 */ 


#ifndef CONTROL_H_
#define CONTROL_H_

#include "global.h"
#include "my_sensor.h"

#define MOTOR_OUT_FL(x) TIM_SetCompare3(TIM3,x)
#define MOTOR_OUT_FR(x) TIM_SetCompare2(TIM4,x)
#define MOTOR_OUT_RR(x) TIM_SetCompare1(TIM4,x)
#define MOTOR_OUT_RL(x) TIM_SetCompare4(TIM3,x)


#define Control_IntegrationMax		1000000

#define Control_YawOutputMax			1000
#define Control_RollOutputMax			1000
#define Control_PitchOutputMax		1000
#define Control_ThrottleOutputMax	1000
#define Control_TotalOutputMax		1000

#define Control_Timestep					IMU_UPDATE_DT

typedef struct{float pitch;float roll;float yaw;} _RPYf_t;
typedef struct{int16_t pitch;int16_t roll;int16_t yaw;} _RPY16i_t;
typedef struct{_RPYf_t desired;_RPYf_t actual;_RPYf_t difference;_RPYf_t lastdifference;_RPYf_t integration;\
							_RPYf_t Kp;_RPYf_t Ki;_RPYf_t Kd;_RPYf_t inputscale;\
							_RPY16i_t output;}	_Control_Loop_t;

//Throttle
extern volatile int16_t Control_ThrottleLast;
//InputNormalize
extern volatile float Control_ThrottleKp;
extern volatile float Control_ThrottleKd;
//PWM Output
extern volatile int16_t Control_ThrottleDesired;

extern _Control_Loop_t Control_OmegaLoop;
extern _Control_Loop_t Control_AngleLoop;
//#ifdef Test_Control
extern volatile int16_t gMotor_pulse[NumOfOCChannel];
extern volatile int16_t gMotor_delta_pitch;
extern volatile int16_t gMotor_delta_roll;
extern volatile int16_t gMotor_delta_yaw;
//#endif

void Control_GetThrottleDesired(void);
void Control_GetOmegaLoopDesired(void);//OmegaLoop Feedforward
void Control_GetOmegaLoopActual(void);//OmegaLoop Feedback
void Control_GetAngleLoopDesired(void);//AngleLoop Feedforward
void Control_GetAngleLoopActual(void);//AngleLoop Feedback
void Control_PID(_Control_Loop_t* loop);
void Control_ClearI(_Control_Loop_t* loop);
void Control_SetMotorPWM(_Control_Loop_t* loop);
void Control_Output_PWM(void);
void Control_Calc_PWM(_Control_Loop_t* loop);
void Control_TestMotors(void);

#endif /* CONTROL_H_ */
