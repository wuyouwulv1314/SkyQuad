#include "IICtools.h"
#include "my_sensor.h"
#include "UARTtools.h"


uint8_t* IIC_Queue = 0;
volatile uint8_t IIC_QueuePointer = 0;

volatile IIC_StatusType IIC_Status = {IIC_Stat_Done,IIC_Step_Call,IIC_Switch_Go,IIC_I2CStat_Free,0,0,0};


void IIC_MasterRW(IIC_StatType stat,uint8_t slave_addr,uint8_t reg_addr,uint16_t length,uint8_t* data)
{
	if(IIC_Status.sw == IIC_Switch_Pause)return;
	//wait until free
	while(IIC_Status.i2cstat == IIC_I2CStat_Busy);
	IIC_Status.stat = stat;
	IIC_Status.step = IIC_Step_Call;
	IIC_Status.slave_addr = slave_addr;
	IIC_Status.reg_addr = reg_addr;
	IIC_Status.queuelength = length;
	IIC_QueuePointer = 0;
	IIC_Queue = data;
	IIC_START();
}

void IIC_STOP(void)
{
	IIC_MCU_DISABLEACK;
	IIC_MCU_STOP;
	IIC_Status.i2cstat = IIC_I2CStat_Free;
}

void IIC_START(void)
{
	IIC_Status.i2cstat = IIC_I2CStat_Busy;
	IIC_MCU_START;
	IIC_MCU_ENABLEACK;
}

void IIC_StartEvent(void)
{
	if((IIC_Status.step == IIC_Step_Call))
	{
			IIC_MCU_SendSlaveAddrW;
	}
	else
	{
			IIC_MCU_SendSlaveAddrR;
	}
}

void IIC_AddressEvent(void)
{
	uint8_t data;
	uint8_t send = 0;
	if(IIC_Status.step == IIC_Step_Call)
	{
		IIC_Status.step = IIC_Step_Sub;
		send = 1;
		data = IIC_Status.reg_addr;
	}
	else
	{
		IIC_Status.step = IIC_Step_RW;
		IIC_QueuePointer = 0;
		//read mode in scan stat
		{
			//dummy read
			IIC_MCU_DUMMYREAD(&data);
		}
	}
	if(send)
	{
		IIC_MCU_SENDDATA(data);
	}
}

void IIC_DataEvent(void)
{
	uint8_t data;
	uint8_t send = 0;
	if((IIC_Status.sw == IIC_Switch_Pause) || (IIC_Status.i2cstat == IIC_I2CStat_Free))
	{
		//dummy read
		IIC_MCU_DUMMYREAD(&data);
		IIC_STOP();
	}else
	{
		if(IIC_Status.step == IIC_Step_Sub)
			//restart
		{
			if(IIC_Status.stat == IIC_Stat_Write)
			{
				IIC_Status.step = IIC_Step_RW;
				IIC_QueuePointer = 0;
				//start register Write
				send = 1;
				data = IIC_Queue[IIC_QueuePointer];
			}else
			{
				IIC_MCU_RESTART;
			}
		}else
		{
			if(IIC_Status.stat == IIC_Stat_Write)
				//send Write queue
			{
				IIC_QueuePointer ++;
				if(IIC_QueuePointer == IIC_Status.queuelength)
					//one unit Write done
				{
					IIC_STOP();
					//dummy read
					IIC_MCU_DUMMYREAD(&data);
					//auto pause
					//IIC_Status.sw = IIC_Switch_Pause;
					IIC_Status.stat = IIC_Stat_Done;
				}else
				{
					send = 1;
					data = IIC_Queue[IIC_QueuePointer];
				}
			}else
			{
				if((IIC_QueuePointer + 2) == IIC_Status.queuelength)
				{
					IIC_MCU_STOP1;
				}
				else if((IIC_QueuePointer + 1) == IIC_Status.queuelength)
				{
					IIC_MCU_STOP2;
				}
				if(IIC_QueuePointer < IIC_Status.queuelength)
						IIC_MCU_RECEIVEDATA(&IIC_Queue[IIC_QueuePointer]);
				IIC_QueuePointer ++;
				if(IIC_QueuePointer == IIC_Status.queuelength)
				{
					IIC_MCU_STOP3;
					IIC_MCU_DUMMYREAD(&data);
					IIC_Status.stat = IIC_Stat_Done;
					IIC_UserProcedureAfterScanOnce;
				}
			}
		}
		if(send)IIC_MCU_SENDDATA(data);
	}
}

void IIC_StructInit(void)
{
	if(IIC_Status.i2cstat == IIC_I2CStat_Busy)return;
	IIC_Status.stat = IIC_Stat_Done;
	IIC_Status.step = IIC_Step_Call;
	IIC_Status.sw = IIC_Switch_Go;
	IIC_Status.slave_addr = 0;
	IIC_Status.reg_addr = 0;
	IIC_Status.queuelength = 0;
	IIC_QueuePointer = 0;
	IIC_Queue = 0;
}

void IIC_Error(uint8_t errorcode)
{
	//user defined error routine
	gsPrintfActualLength = sprintf((uint8_t*)gsPrintfBuffer,"IIC ERROR.Code=%d\r\n",errorcode);
	UARTsendString(Tool_USART1,(uint8_t*)gsPrintfBuffer);
	while(1);
}
