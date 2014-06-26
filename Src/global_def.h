/*
 * =====================================================================================
 *
 *       Filename:  global_def.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/17 10:33:48
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __GLOBAL_DEF__H__
#define __GLOBAL_DEF__H__

#ifndef bool
#define bool uint8_t
#define true 1
#define false 0
#endif
//#define Test_BT_USART
//#define Test_I2C
//#define Test_Sensor
//#define Test_Control
//#define NoMotorOut
//#define Test_Tune_PID

#define Control_TestMotor
#define Control_Default

#ifdef Control_Default
#undef Control_TestMotor
#endif



#endif  /*__GLOBAL_DEF__H__*/
