/*
 * =====================================================================================
 *
 *       Filename:  my_control.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/18 10:51:05
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __MY_CONTROL__H__
#define __MY_CONTROL__H__

#include "Control.h"
extern _Control_Loop_t my_loop;
enum AdjustPIDMode{AdjustNone=0,AdjustP,AdjustI,AdjustD};
extern enum AdjustPIDMode adjustPID_mode;
extern bool isCompareOK;
extern bool isTune;
void get_remote_control_desired(_Control_Loop_t * loop);
void get_attitude_actual(_Control_Loop_t * loop);
void calc_pid(_Control_Loop_t * loop);
void calc_motor_pwm(_Control_Loop_t * loop);
void output_motor_pwm(void);
#endif  /*__MY_CONTROL__H__*/
