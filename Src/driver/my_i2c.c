/*
 * =====================================================================================
 *
 *       Filename:  my_i2c.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 10:48:31
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "global.h"
#include "my_i2c.h"
#include "my_sensor.h"
#include "my_misc.h"
#include "UARTtools.h"

static CPAL_TransferTypeDef CPAL_Transfer;

void IIC_WaitForRWDone(void)
{
	while ((MPU6050_I2C_DEV.CPAL_State != CPAL_STATE_READY) && (MPU6050_I2C_DEV.CPAL_State != CPAL_STATE_ERROR) )
    { }
}

void IIC_MasterRW(IIC_StatType stat,uint8_t slave_addr,uint8_t reg_addr,uint16_t length,uint8_t* data)
{
	IIC_WaitForRWDone();
	CPAL_Transfer.wNumData = length;
	CPAL_Transfer.pbBuffer = data;
	CPAL_Transfer.wAddr1 = (uint32_t)slave_addr;
	CPAL_Transfer.wAddr2 = (uint32_t)reg_addr;

	MPU6050_I2C_DEV.wCPAL_Options = 0;

	if(stat == IIC_Stat_Write){
		MPU6050_I2C_DEV.pCPAL_TransferTx = &CPAL_Transfer;
		if(CPAL_I2C_Write(&MPU6050_I2C_DEV) == CPAL_PASS){
			// IIC_WaitForRWDone();
		}
	}
	else if(stat == IIC_Stat_Read){
		MPU6050_I2C_DEV.pCPAL_TransferRx = &CPAL_Transfer;
		if(CPAL_I2C_Read(&MPU6050_I2C_DEV) == CPAL_PASS){
			// IIC_WaitForRWDone();
		}
	}

}

void I2C_Config(void)/*{{{*/
{
	CPAL_I2C_StructInit(&MPU6050_I2C_DEV);

	// MPU6050_I2C_DEV.CPAL_Dev is initialized when defined
	MPU6050_I2C_DEV.pCPAL_I2C_Struct->I2C_ClockSpeed = 350000;
	MPU6050_I2C_DEV.pCPAL_I2C_Struct->I2C_OwnAddress1 = 0xAA;

	CPAL_I2C_Init(&MPU6050_I2C_DEV);
}/*}}}*/
void my_I2C_Init(void)/*{{{*/
{
	//IICTool Init
	delay_ms(100);
	//while(1)
	{
		IIC_MasterRW(IIC_Stat_Write,MPU6050Addr,MPU6050InitStartAddr1,MPU6050InitQueueLength1,gMPU6050InitQueue1);
	//	Delay(0xfff);
	}
	IIC_WaitForRWDone();

	delay_ms(100);

	IIC_MasterRW(IIC_Stat_Write,MPU6050Addr,MPU6050InitStartAddr2,MPU6050InitQueueLength2,gMPU6050InitQueue2);
	IIC_WaitForRWDone();

}/*}}}*/
void CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError)/*{{{*/
{
	//user defined error routine
	gsPrintfActualLength = sprintf((uint8_t*)gsPrintfBuffer,"IIC ERROR.Code=%d\r\n",DeviceError);
	UARTsendString(Tool_USART1,(uint8_t*)gsPrintfBuffer);
	while(1);
}/*}}}*/
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)/*{{{*/
{
	MPU6050_RawProcess();
}/*}}}*/