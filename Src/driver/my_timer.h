/*
 * =====================================================================================
 *
 *       Filename:  my_timer.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 11:49:50
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __MY_TIMER__H__
#define __MY_TIMER__H__

#include "stm32f10x.h"

void TIM_Interrupt_Config(void);
void TIM_Motor_Config(void);

#endif  /*__MY_TIMER__H__*/
