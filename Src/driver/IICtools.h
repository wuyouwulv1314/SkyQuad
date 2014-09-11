#ifndef __IICTOOLS_H__
#define	__IICTOOLS_H__

#include "global.h"
#include "stm32f10x.h"

#define I2C_Sensor I2C2
#define I2C_Sensor_Clk RCC_APB1Periph_I2C2
#define I2C_Sensor_EV_IRQHandler I2C2_EV_IRQHandler


#define IIC_CallRead	1
#define IIC_CallWrite	0

//I2C Inline Operation Commands
#define IIC_Scan_Enable					{IIC_Status.sw = IIC_Switch_Go;}
#define IIC_Scan_Disable				{IIC_Status.sw = IIC_Switch_Pause;IIC_I2C_STOP();}
	
#define	IIC_WaitForRWDone				{while(IIC_Status.stat != IIC_Stat_Done);}
#define	IIC_LetInitPass					IIC_WaitForRWDone

//#define IIC_UserProcedureAfterScanOnce	{I2CSensorScanHalfDone();}
#define IIC_UserProcedureAfterScanOnce	{MPU6050_RawProcess();}
	
#define IIC_IsStatRead					(IIC_Status.stat == IIC_Stat_Read)
#define IIC_IsStatWrite					(IIC_Status.stat == IIC_Stat_Write)
	
//MCU Depedent Command
//User Define
#define IIC_MCU_ENABLEACK				{I2C_AcknowledgeConfig(I2C2,ENABLE);}
#define IIC_MCU_DISABLEACK				{I2C_AcknowledgeConfig(I2C2,DISABLE);}

#define IIC_MCU_STOP					{I2C_GenerateSTOP(I2C2,ENABLE);}
#define IIC_MCU_START					{I2C_GenerateSTART(I2C2,ENABLE);}

#define IIC_MCU_STOP1					{IIC_MCU_DISABLEACK;}
#define IIC_MCU_STOP2 					{IIC_MCU_STOP;}
#define	IIC_MCU_STOP3					{IIC_Status.i2cstat = IIC_I2CStat_Free;}
	
#define IIC_MCU_RESTART					{IIC_MCU_START;}

#define IIC_MCU_SendSlaveAddrW			{I2C_Send7bitAddress(I2C2,IIC_Status.slave_addr,I2C_Direction_Transmitter);}
#define IIC_MCU_SendSlaveAddrR			{I2C_Send7bitAddress(I2C2,IIC_Status.slave_addr,I2C_Direction_Receiver);}

#define IIC_MCU_SENDDATA(data)			{I2C_SendData(I2C2,(uint8_t)data);}
#define IIC_MCU_RECEIVEDATA(addr)		{*((uint8_t*)addr) = I2C_ReceiveData(I2C2);}

#define IIC_MCU_DUMMYREAD(dumpaddr)		{IIC_MCU_RECEIVEDATA(dumpaddr);}
//end of user define

typedef enum{IIC_Switch_Go,IIC_Switch_Pause,}IIC_SwitchType;
typedef enum{IIC_I2CStat_Busy,IIC_I2CStat_Free,}IIC_I2CStatType;
typedef enum{IIC_Stat_Write,IIC_Stat_Read,IIC_Stat_Done,}IIC_StatType;
typedef enum{IIC_Step_Call = 1,IIC_Step_Sub = 2,IIC_Step_RW = 3,}IIC_StepType;
typedef struct{
	volatile IIC_StatType stat;
	volatile IIC_StepType step;
	volatile IIC_SwitchType sw;
	volatile IIC_I2CStatType i2cstat;
	uint8_t slave_addr;
	uint8_t reg_addr;
	uint16_t queuelength;
	}	IIC_StatusType;

extern uint8_t* IIC_Queue;
extern volatile uint8_t IIC_QueuePointer;
extern volatile IIC_StatusType IIC_Status;

void IIC_MasterRW(IIC_StatType stat,uint8_t slave_addr,uint8_t reg_addr,uint16_t length,uint8_t* data);
void IIC_STOP(void);
void IIC_START(void);
void IIC_StartEvent(void);
void IIC_AddressEvent(void);
void IIC_DataEvent(void);
void IIC_StructInit(void);
void IIC_Error(uint8_t errorcode);

#endif
