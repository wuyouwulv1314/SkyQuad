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

#include "my_i2c.h"
#include "IICtools.h"
#include "my_sensor.h"
#include "my_misc.h"
void I2C_IO_Config(void)/*{{{*/
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}/*}}}*/
void I2C_Config(void)/*{{{*/
{
	I2C_InitTypeDef  I2C_InitStructure; 
	I2C_StructInit(&I2C_InitStructure);
	I2C_IO_Config();
	RCC_APB1PeriphClockCmd(I2C_Sensor_Clk, ENABLE);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xAA;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 350000;
	//I2C_InitStructure.I2C_ClockSpeed = 100000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C_Sensor, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C_Sensor, &I2C_InitStructure);

	I2C_ITConfig(I2C_Sensor,I2C_IT_EVT,ENABLE);
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
	IIC_LetInitPass;

	delay_ms(100);

	IIC_MasterRW(IIC_Stat_Write,MPU6050Addr,MPU6050InitStartAddr2,MPU6050InitQueueLength2,gMPU6050InitQueue2);
	IIC_LetInitPass;

}/*}}}*/
//void I2C_Sensor_EV_IRQHandler(void)
void I2C2_EV_IRQHandler(void)
{
	if(I2C_GetFlagStatus(I2C_Sensor,I2C_FLAG_SB) == SET)
		//start sent
	{
		//clear flag
		I2C_ReadRegister(I2C_Sensor,I2C_Register_SR1);
		IIC_StartEvent();
	}else
	if(I2C_GetFlagStatus(I2C_Sensor,I2C_FLAG_ADDR) == SET)
		//address sent
	{
		//clear flag
		I2C_ReadRegister(I2C_Sensor,I2C_Register_SR1);
		I2C_ReadRegister(I2C_Sensor,I2C_Register_SR2);
		IIC_AddressEvent();
	}else
	if(I2C_GetFlagStatus(I2C_Sensor,I2C_FLAG_BTF) == SET)
		//data transferred
	{
		//clear flag
		I2C_ReadRegister(I2C_Sensor,I2C_Register_SR1);
		IIC_DataEvent();
	}else
	{
		//clear flag
		I2C_ReadRegister(I2C_Sensor,I2C_Register_SR1);
		I2C_ReadRegister(I2C_Sensor,I2C_Register_SR2);
		I2C_ReadRegister(I2C_Sensor,I2C_Register_DR);
	}
}
