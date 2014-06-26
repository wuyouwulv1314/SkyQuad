/*
 * DataReceiver.h
 *
 * Created: 2014/2/15 1:24:44
 *  Author: Administrator
 */ 
#ifndef DATARECEIVER_H_
#define DATARECEIVER_H_

#include "global.h"

#define DRNumOfDataInPackage						7
#define DRDataOffset										1000
#define DRDataCenter										1500
#define DRDataTolerance									50
#define DRChannelMiddle(channel)				(((channel - DRDataCenter) < 50) && ((DRDataCenter - channel) < 50))
#define DRChannelBottom(channel)				(((channel - DRDataOffset) < 50) && ((DRDataOffset - channel) < 100))
#define DRChannelMiddleLock(channel)		(((channel - DRDataCenter) < 100) && ((DRDataCenter - channel) < 100))
#define DRDataReceiveMax								2100
#define DRDataMax												(1 << 10)
#define DRDataNull											0
#define DRDataCheckChar									'-'
#define DRNumOfCheckChar								4
#define	DRUserProcedureAfterReceiveOnce	{AdaptControlHobby();}
	
extern volatile uint16_t* DRDataPointerDone;
	
void DRReceiver(uint8_t data);

#endif /* DATARECEIVER_H_ */
