/*
 * =====================================================================================
 *
 *       Filename:  my_misc.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 20:26:55
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "my_misc.h"
#include "global.h"
void delay_ms(uint16_t ms)
{
	gDelayms = ms;
	gDelayTicks = 0;
	while(gDelayTicks < gDelayms);
	gDelayms = 0;
	gDelayTicks = Delay_Hang;
}

