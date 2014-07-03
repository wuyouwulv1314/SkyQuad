/*
 * DataReceiver.c
 *
 * Created: 2014/2/15 1:24:07
 *  Author: Administrator
 */ 
#include "DataReceiver.h"
#include "global_def.h"
#include "my_sensor.h"

enum{DRnull,f,u,n,data0,data1,data2,data3,data4,data5,data6};
	
volatile uint8_t DRProcess = DRnull;
volatile uint16_t DRData[DRNumOfDataInPackage * 2] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile uint16_t* DRDataPointerDone = DRData;
volatile uint16_t* DRDataPointerProcess = DRData + DRNumOfDataInPackage;
const uint8_t DRCheckChar[] = "fun-";
	
void AdaptControlHobby(void);

void DRReceiver(uint8_t data)
{
	uint8_t step;
	uint8_t i;
	uint16_t* temp;
	uint16_t tempdata;//tempdata才16位，所以要求传进来的数据不超过65535.
	
	if(DRProcess < DRNumOfCheckChar)
	{
		if(data == DRCheckChar[DRProcess])DRProcess ++;
		else DRProcess = DRnull;
	}else
	{
		step = DRProcess - DRNumOfCheckChar;
		if((data >= '0') && (data <= '9'))
		{
			tempdata = (10 * DRDataPointerProcess[step]) + (data - 0x30);
			if(tempdata < DRDataReceiveMax)DRDataPointerProcess[step] = tempdata;
		}else
		{
			if(DRProcess == data6)
			{
				temp = DRDataPointerProcess;
				DRDataPointerProcess = DRDataPointerDone;
				DRDataPointerDone = temp;
				for(i = 0;i < DRNumOfDataInPackage;i ++)
					DRDataPointerProcess[i] = DRDataNull;
				DRProcess = DRnull;
				flag_recv_instruct=true;
				g_check_recv_instruct_cnt=0;
				DRUserProcedureAfterReceiveOnce;
			}else
			{
				if(data == '-')
					DRProcess ++;
				else
				{
					for(i = 0;i < DRNumOfDataInPackage;i ++)
						DRDataPointerProcess[i] = DRDataNull;
					DRProcess = DRnull;
				} 
			}
		}
	}
}

void AdaptControlHobby(void)
{
#ifdef Control_Default
//	uint16_t temp16[NumOfFlyChannel];
//	uint8_t	i;
//
//	//China Hand - Standard
//	if((DRDataPointerDone[CH6] > 1000) && (DRDataPointerDone[CH6] < 1333))
//	{
//		;
//	}
//	//Japan Hand
//	else if((DRDataPointerDone[CH6] > 1333) && (DRDataPointerDone[CH6] < 1667))
//	{
//		temp16[Yaw] = DRDataPointerDone[Yaw_JAP];//(1000 - (DRDataPointerDone[Yaw_JAP] - 1000)) + 1000
//		temp16[Pitch] = DRDataPointerDone[Pitch_JAP];
//		temp16[Throttle] = DRDataPointerDone[Throttle_JAP];
//		temp16[Roll] = DRDataPointerDone[Roll_JAP];
//		for(i = 0;i < NumOfFlyChannel;i ++)DRDataPointerDone[i] = temp16[i];
//	}
//	//America Hand
//	else if((DRDataPointerDone[CH6] > 1667) && (DRDataPointerDone[CH6] < 2000))
//	{
//		temp16[Yaw] = DRDataPointerDone[Yaw_USA];
//		temp16[Pitch] = DRDataPointerDone[Pitch_USA];
//		temp16[Throttle] = DRDataPointerDone[Throttle_USA];
//		temp16[Roll] = DRDataPointerDone[Roll_USA];
//		for(i = 0;i < NumOfFlyChannel;i ++)DRDataPointerDone[i] = temp16[i];
//	}
#endif
}
