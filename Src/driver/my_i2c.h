/*
 * =====================================================================================
 *
 *       Filename:  my_i2c.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 10:49:39
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __MY_I2C__H__
#define __MY_I2C__H__

#include "stm32f10x.h"
#include "cpal_i2c.h"

//I2C2_DevStructure is defined in CPAL library
#define MPU6050_I2C_DEV       I2C2_DevStructure


typedef enum{IIC_Stat_Write,IIC_Stat_Read,IIC_Stat_Done,}IIC_StatType;

void IIC_WaitForRWDone(void);
void IIC_MasterRW(IIC_StatType stat,uint8_t slave_addr,uint8_t reg_addr,uint16_t length,uint8_t* data);
void I2C_Config(void);
void my_I2C_Init(void);


#endif  /*__MY_I2C__H__*/
