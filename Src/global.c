/*
 * =====================================================================================
 *
 *       Filename:  global.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014/6/15 20:28:39
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "global.h"

#ifdef Test_BT_USART
bool flag_bt_recv=false;
uint16_t data_bt_recv;
#endif

volatile uint16_t gDelayCounter = 0;
volatile uint16_t gDelayTicks = Delay_Hang;
volatile uint16_t gDelayms = 0;

uint16_t gLEDCounterR = LED_Hang;
uint16_t gLEDCounterG = LED_Hang;
uint16_t gLEDCounterB = LED_Hang;
volatile uint16_t gLEDTurnTreshR = LEDTurnTresh_Mode1;
volatile uint16_t gLEDTurnTreshG = LEDTurnTresh_Mode1;
volatile uint16_t gLEDTurnTreshB = LEDTurnTresh_Mode1;

uint16_t gTransferCounter = TransferHang;
uint8_t gSend = 0;
volatile uint8_t gControl = 0;
uint8_t gsPrintfBuffer[255];
uint32_t gsPrintfActualLength = 0;
uint8_t* gUARTpointer = 0;

volatile uint8_t gpwmen = 0;
volatile uint8_t gbutton = 0;
volatile uint8_t gbuttonedge = NoneEdge;

bool flag_recv_instruct=false;
int g_check_recv_instruct_cnt = 0;

