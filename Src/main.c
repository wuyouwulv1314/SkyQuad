/*     BYYB   SKYWORKS   TSINGHUA     */
/*	   USING  STM32  LIB V3.5.0		  */
/*	   2011.07.03    @C#410			  */
#include "stm32f10x.h"

#include "nvic.h"
#include "my_usart.h"
#include "my_i2c.h"
#include "global_def.h"
#include "global.h"
#include "my_timer.h"
#include "DataReceiver.h"
#include "AHRS.h"
#include "UARTtools.h"
#include "Control.h"
#include "my_sensor.h"
#include "MadgwickAHRS.h"
#include "my_control.h"
#include "my_misc.h"
#include <math.h>

#define LED_Blink_Delay_Cnt 0x7ffff
#define LED1_OFF GPIO_SetBits(GPIOA, GPIO_Pin_1)  
#define LED1_ON GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define LED2_OFF GPIO_SetBits(GPIOA, GPIO_Pin_7)  
#define LED2_ON GPIO_ResetBits(GPIOA, GPIO_Pin_7)
#define LED3_OFF GPIO_SetBits(GPIOB, GPIO_Pin_12)  
#define LED3_ON GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define LED4_OFF GPIO_SetBits(GPIOB, GPIO_Pin_3)  
#define LED4_ON GPIO_ResetBits(GPIOB, GPIO_Pin_3)

void Delay(u32 nCount);	

void LED_Config(void);
void periph_init(void);
void ControlProcess(void);
void RC_Process(void);
void PrintProcess(void);

extern int16_t rc_throttle;

int main()/*{{{*/
{
	u32 send_cnt=0;
	uint8_t send_type=0;

	periph_init();
#ifdef Test_I2C/*{{{*/
	while(1)
	{
		LED1_ON;
		LED2_ON;
		Delay(LED_Blink_Delay_Cnt);
		LED1_OFF;
		LED2_OFF;
		Delay(LED_Blink_Delay_Cnt);
	}
#endif/*}}}*/
#ifdef Test_Sensor/*{{{*/
	while(1)
	{
		if(gControl==1)
		{
			gControl=0;
			AHRS_AccelIIRLPFilter();
			//AHRS_GyroIIRLPFilter();
			AHRS_Normalize();
//				gsPrintfActualLength = sprintf(gsPrintfBuffer,"fun:	%d	%d	%d	%d	%d	%d\r\n",\
//							gMPU6050LPFAccel.x,gMPU6050LPFAccel.y,gMPU6050LPFAccel.z,
//							gMPU6050LPFGyro.x,gMPU6050LPFGyro.y,gMPU6050LPFGyro.z);
//
//			gsPrintfActualLength = sprintf(gsPrintfBuffer,"func-normal:%4.2f	%4.2f	%4.2f	%4.2f	%4.2f	%4.2f\r\n",
//					gAccel.x,gAccel.y,gAccel.z,gGyro.x,gGyro.y,gGyro.z);
//			UARTsendString(Tool_USART_BT,(uint8_t*)gsPrintfBuffer);
		
		}
	}
#endif/*}}}*/
#ifdef Test_Control/*{{{*/
	//gMPU6050Counter = MPU6050_Hang;
	//UARTsendString(TO)
	while(1)
	{
		if(gControl==1)/*{{{*/
		{
			gControl=0;
			gpwmen=1;
			AHRS_AccelIIRLPFilter();
			AHRS_Normalize();
			MadgwickAHRSupdateIMU(gGyro.x,gGyro.y,gGyro.z,gAccel.x,gAccel.y,gAccel.z);
			AHRS_GetEulerRPY();
			Control_GetThrottleDesired();

			Control_GetAngleLoopDesired();
			Control_GetAngleLoopActual();
			Control_PID(&Control_AngleLoop);

			Control_GetOmegaLoopDesired();
			Control_GetOmegaLoopActual();
			Control_PID(&Control_OmegaLoop);
			if(gpwmen)
			{
#ifdef Control_Default
				Control_SetMotorPWM(&Control_OmegaLoop);
#endif
#ifdef Control_TestMotor
				Control_TestMotors();
#endif
#if 0
			if(gSend)
			{
				gSend=0;
				//gsPrintfActualLength = sprintf(gsPrintfBuffer,"func-normal:%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\r\n",
				gsPrintfActualLength = sprintf(gsPrintfBuffer,"func:"FSF FSF FSF FSF FSF FSF FSF FSF FSF"\r\n",
						gAccel.x,gAccel.y,gAccel.z,gGyro.x,gGyro.y,gGyro.z,eulerPitchActual,eulerRollActual,eulerYawActual);
				UARTsendString(Tool_USART_BT,(uint8_t*)gsPrintfBuffer);
			}
#endif
#if 1
				gsPrintfActualLength=sprintf(gsPrintfBuffer,"func:"FSF FSF FSF ISF ISF ISF ISF ISF ISF"\r\n",
						eulerPitchActual,eulerRollActual,eulerYawActual,
									gMotor_pulse[FL],gMotor_pulse[FR],gMotor_pulse[RL],gMotor_pulse[RR],
									gMotor_delta_pitch,gMotor_delta_roll);
				UARTsendString(Tool_USART_BT,(uint8_t*)gsPrintfBuffer);
#endif
#if 0
				gsPrintfActualLength=sprintf(gsPrintfBuffer,"%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f "
						"%4.2f %4.2f %4.2f %4.2f ""%4.2f %4.2f %4.2f\r\n",
						gAccel.x,gAccel.y,gAccel.z,gGyro.x,gGyro.y,gGyro.z,
						q0,q1,q2,q3,eulerRollActual,eulerPitchActual,eulerYawActual);
				UARTsendString(Tool_USART_BT,(uint8_t*)gsPrintfBuffer);
#endif
#if 0				
				send_cnt++;
				if(send_cnt>20)
				{
					send_cnt=0;
					switch(send_type)
					{
						case 0:
							gsPrintfActualLength=sprintf(gsPrintfBuffer,"func-angle:%4.2f	%4.2f	%4.2f\r\nomega-desire:%4.2f	%4.2f	%4.2f\r\n",
									eulerRollActual,eulerPitchActual,eulerYawActual,
									Control_OmegaLoop.desired.pitch,Control_OmegaLoop.desired.roll,
									Control_OmegaLoop.desired.yaw);
							break;
						case 1:
							gsPrintfActualLength=sprintf(gsPrintfBuffer,"omega-pid:%4.2f	%4.2f\r\n",
									Control_OmegaLoop.Kp.pitch,Control_OmegaLoop.Kp.roll);
							break;
						case 2:
							gsPrintfActualLength=sprintf(gsPrintfBuffer,"func-motor:%d	%d	%d	%d\r\n",
									gMotor_pulse[FL],gMotor_pulse[FR],gMotor_pulse[RL],gMotor_pulse[RR]);
							break;
						default:
							break;
					}
					if(send_type<=2)
						UARTsendString(Tool_USART_BT,(uint8_t*)gsPrintfBuffer);
					send_type++;
					if(send_type>80)
						send_type=0;
				}
#endif
			}
		}/*}}}*/

	}
#endif/*}}}*/
	while (1)
	{
		RC_Process();
		PrintProcess();
		ControlProcess();
	}
}/*}}}*/

void periph_init(void)/*{{{*/
{
	RCC_ClocksTypeDef systemclocks;
	LED_Config();
	USART_Config();
	BT_USART_Config();
	I2C_Config();
	TIM_Interrupt_Config();
	TIM_Motor_Config();
	NVIC_Config();
	RCC_GetClocksFreq(&systemclocks);
	UARTsendString(Tool_USART2,"hello\r\n");
#ifdef Test_BT_USART/*{{{*/
	while(1)
	{
		if(flag_bt_recv)
		{
			delay_ms(2000);
			flag_bt_recv=false;
		}
		else
		{
			UARTsendString(Tool_USART2,"hello\r\n");
			//printf("hello\r\n");
			delay_ms(300);
		}
	}
#endif/*}}}*/
	my_I2C_Init();
	gMPU6050Counter = 0;
	gLEDCounterR = 0;
	gLEDCounterG = 0;
	gLEDCounterB = 0;

	delay_ms(500);
	calib_sensor();
	gsPrintfActualLength = sprintf((uint8_t*)gsPrintfBuffer,(uint8_t*)"Boot Success.\r\n"
			"System Freq:%dHz\r\n""Bulid""	"__DATE__"	"__TIME__"\r\n"
			"Powered by function\r\n",systemclocks.HCLK_Frequency);
	UARTsendString(Tool_USART_BT,(uint8_t*)gsPrintfBuffer);

	gTransferCounter = 0;// allow send data to bluetooth
}/*}}}*/
void RC_Process(void)
{
#ifdef Test_Tune_PID
	float tf;
	static int tune_denoise_cnt=0;
	static int nottune_denoise_cnt=0;
	// RY , adjust pid /*{{{*/
#define Distance(x,y) abs((x)-(y))
	gpwmen = 0;
	if((DRDataPointerDone[CH5] > 1750))
	{
		gpwmen = 1;
		if(DRDataPointerDone[Pitch] < 1250 && DRDataPointerDone[Roll] > 1750)
			adjustPID_mode = AdjustNone;
		else if(DRDataPointerDone[Pitch] > 1750 && DRDataPointerDone[Roll] > 1750)
			adjustPID_mode = AdjustP;
		else if(DRDataPointerDone[Pitch] > 1750 && DRDataPointerDone[Roll] < 1250)
			adjustPID_mode = AdjustI;
		else if(DRDataPointerDone[Pitch] < 1250 && DRDataPointerDone[Roll] < 1250)
			adjustPID_mode = AdjustD;
		isTune = false;
//		tf=(DRDataPointerDone[CH6]-DRDataOffset)*0.05;
//		my_loop.Kp.pitch=my_loop.Kp.roll=my_loop.Kp.yaw=tf;
	}
	else if((DRDataPointerDone[CH7] < 1250) )
	{
		tune_denoise_cnt++;
		if(tune_denoise_cnt > 5)
		{
			tune_denoise_cnt=0;
			nottune_denoise_cnt=0;
			isTune = false;
		}
		//Control_ClearI(&Control_AngleLoop); // XXX
	}
	else if(DRDataPointerDone[CH7] > 1750)
	{
		nottune_denoise_cnt++;
		if(nottune_denoise_cnt > 5)
		{
			tune_denoise_cnt=0;
			nottune_denoise_cnt=0;
			isTune  = true;
		}
	}
	if(AdjustP == adjustPID_mode)/*{{{*/
	{
		tf=(DRDataPointerDone[CH6]-DRDataOffset)*0.05;
		if(!isTune)
		{
			if(Distance(tf,my_loop.Kp.pitch) < 2)
				isCompareOK=true;
			else 
				isCompareOK=false;
		}
		else
		{
			my_loop.Kp.pitch=my_loop.Kp.roll=my_loop.Kp.yaw=tf;
		}
	}/*}}}*/
	else if(AdjustI == adjustPID_mode)/*{{{*/
	{
		tf=(DRDataPointerDone[CH6]-DRDataOffset)*0.05;
		if(!isTune)
		{
			if(Distance(tf,my_loop.Ki.pitch) < 2)
				isCompareOK=true;
			else 
				isCompareOK=false;
		}
		else
		{
			my_loop.Ki.pitch=my_loop.Ki.roll=my_loop.Ki.yaw=tf;
		}
	}/*}}}*/
	else if(AdjustD == adjustPID_mode)/*{{{*/
	{
		tf=(DRDataPointerDone[CH6]-DRDataOffset)*0.05;
		if(!isTune)
		{
			if(Distance(tf,my_loop.Kd.pitch) < 2)
				isCompareOK=true;
			else 
				isCompareOK=false;
		}
		else
		{
			my_loop.Kd.pitch=my_loop.Kd.roll=my_loop.Kd.yaw=tf;
		}
	}/*}}}*/
//	my_loop.Kd.pitch=my_loop.Kd.roll=my_loop.Kd.yaw=tf;

/*}}}*/
#endif
#ifndef Test_Tune_PID
#if 1
	//FX/*{{{*/
	if((DRDataPointerDone[CH5] > 1750) && ((Control_ThrottleDesired != 0) || (rc_throttle!=0)))
		//motor enable
	{
		gpwmen = 1;
		gLEDCounterR = LED_Hang;
		LED_R_Set(LED_On);

		//gRunningPrint = 1;
		gLEDTurnTreshR = LEDTurnTresh_Mode2;
	}
	else
	{
		gpwmen = 0;
		if(gLEDCounterR == LED_Hang)gLEDCounterR = 0;

		//gRunningPrint = 0;
		gLEDTurnTreshR = LEDTurnTresh_ModeOff;

		Control_ClearI(&Control_AngleLoop);
	}
	if((DRDataPointerDone[CH7] < 1250) && (DRDataPointerDone[CH7] > DRDataOffset))
		//button pressed
	{
		//gMPU6050Counter = MPU6050_Hang;
		if(gbutton == 0)
		{
			gbuttonedge = RiseEdge;
		}
		gbutton = 1;
	}
	else if(DRDataPointerDone[CH7] > 1750)
	{
		//gMPU6050Counter = 0;
		if(gbutton == 1)
		{
			gbuttonedge = FallEdge;
		}
		gbutton = 0;
	}	
/*}}}*/
#endif
#endif
}
void PrintProcess(void)/*{{{*/
{
	if(gSend == 1)/*{{{*/
	{
		gSend = 0;
		if(gAHRSStat == AHRS_Cali_Accel)
		{
			if(gAccelCaliPrintEN == 1)
			{
				gAccelCaliPrintEN = 0;
				gAccelCaliPrintCounter ++;
				Accl_Cali_Print();
				gSend = 2;
			}
		}
		else if(gAHRSStat == AHRS_Cali_Gyro)
		{
			if(gGyroCaliPrintEN == 1)
			{
				//	gGyroCaliPrintEN = 0;

				gGyroCaliPrintCounter ++;
				Gyro_Cali_Print();
				gSend = 2;
			}
		}
		else if(gAHRSStat == AHRS_Running)
		{
			if(gRunningPrint)
			{
				Running_Print();
				gSend = 3;
			}
		}
		if(gSend == 2)
		{
			UARTsendString(Tool_USART_BT,(uint8_t*)gUARTpointer);
			gSend = 0;
		}else if(gSend == 3)
		{
			UARTsendN(Tool_USART_BT,(uint8_t*)gUARTpointer,gsPrintfActualLength);
			gSend = 0;
		}
	}/*}}}*/
}/*}}}*/

void ControlProcess(void)/*{{{*/
{

	if(gControl == 1)/*{{{*/
	{
		gControl = 0;
		if(gAHRSStat == AHRS_Cali_Accel)/*{{{*/
		{
			AHRS_AccelIIRLPFilter();
			if(gbuttonedge == RiseEdge)
			{
				gbuttonedge = NoneEdge;
				gAHRSCounter = 0;
				gLEDTurnTreshB = LEDTurnTresh_Mode2;
			}
			if(gAHRSCounter < ACCEL_LPF_TRESH)
			{
				gAHRSCounter ++;
			}else if(gAHRSCounter == ACCEL_LPF_TRESH)
			{
				gLEDTurnTreshB = LEDTurnTresh_Mode1;
				gAHRSCounter ++;
				gAccelCaliPrintEN = 1;
			}
		}/*}}}*/
		else if(gAHRSStat == AHRS_Cali_Gyro)/*{{{*/
		{
			if((gAHRSCounter < ACCEL_LPF_TRESH) || (gAHRSCounter < GYRO_LPF_TRESH))
			{
				if(gAHRSCounter == ACCEL_LPF_TRESH)gLEDTurnTreshB = LEDTurnTresh_Mode1;
				if(gAHRSCounter == GYRO_LPF_TRESH)gLEDTurnTreshG = LEDTurnTresh_Mode1;
				AHRS_AccelIIRLPFilter();
				AHRS_GyroIIRLPFilter();
				gAHRSCounter ++;
			}
			else
			{
				AHRS_GyroBiasSet();
				gAHRSStat = AHRS_Running;
				gLEDTurnTreshB = LEDTurnTresh_Mode1;
				gLEDTurnTreshG = LEDTurnTresh_Mode1;
				gAHRSCounter = 0;
			}
		}/*}}}*/
#if 1		// Single Loop Control
		else if(gAHRSStat == AHRS_Running)/*{{{*/
		{
			AHRS_AccelIIRLPFilter();
			AHRS_Normalize();
			MadgwickAHRSupdateIMU(gGyro.x,gGyro.y,gGyro.z,gAccel.x,gAccel.y,gAccel.z);
			AHRS_GetEulerRPY();

			get_remote_control_desired(&my_loop);
			get_attitude_actual(&my_loop);
			calc_pid(&my_loop);
			calc_motor_pwm(&my_loop);
			output_motor_pwm();
		
		}/*}}}*/
#endif
#if 0
		else if(gAHRSStat == AHRS_Running)/*{{{*/
		{
	//		GPIO_WriteBit(GPIOA,GPIO_Pin_0,1);

			AHRS_AccelIIRLPFilter();
			AHRS_Normalize();
			MadgwickAHRSupdateIMU(gGyro.x,gGyro.y,gGyro.z,gAccel.x,gAccel.y,gAccel.z);
			AHRS_GetEulerRPY();

//			GPIO_WriteBit(GPIOA,GPIO_Pin_0,0);
//			GPIO_WriteBit(GPIOA,GPIO_Pin_0,1);
//			GPIO_WriteBit(GPIOA,GPIO_Pin_0,0);

			//			gtestpitchint += gGyro.x / 500;
			//			gtestrollint += gGyro.y / 500;
			//			gtestyawint += gGyro.z / 500;

			//PLACE PID CONTROL ROUTINE HERE..
		//	GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);

			Control_GetThrottleDesired();

			Control_GetAngleLoopDesired();
			Control_GetAngleLoopActual();
			Control_PID(&Control_AngleLoop);

			Control_GetOmegaLoopDesired();
			Control_GetOmegaLoopActual();
			Control_PID(&Control_OmegaLoop);

//			GPIO_WriteBit(GPIOA,GPIO_Pin_1,0);
//			GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);
//			GPIO_WriteBit(GPIOA,GPIO_Pin_1,0);
//			GPIO_WriteBit(GPIOA,GPIO_Pin_1,1);
//			GPIO_WriteBit(GPIOA,GPIO_Pin_1,0);
			Control_Calc_PWM(&Control_OmegaLoop);
			Control_Output_PWM();
		
		}/*}}}*/
#endif
	}/*}}}*/
}/*}}}*/
void LED_Config(void)				  //初始化LED/*{{{*/
{
	GPIO_InitTypeDef GPIO_InitStructure;  //定义GPIO结构体
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);		 //开启GPIO B的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7;				     //LED管脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			 //将PB5 配置为通用推挽输出  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //IO口翻转速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);					 //将上述设置赋予GPIOB

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_12;				     //LED管脚 
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //将上述设置赋予GPIOB
}/*}}}*/
void Delay(u32 nCount)
{																		              
   for(; nCount != 0; nCount--);
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
