/*
 * AHRS.h
 *
 * Created: 2014/2/23 22:38:34
 *  Author: function
 */ 


#ifndef AHRS_H_
#define AHRS_H_

#include "global.h"

//#define Accel_Cali
//#define AHRS_Running

/**
 * IMU update frequency dictates the overall update frequency.
 */
#define IMU_UPDATE_FREQ   MPU6050ScanFreq
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

/**
 * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
 * The highest cut-off freq that will have any affect is fs /(2*pi).
 * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
 */
#define IMU_ACC_WANTED_LPF_CUTOFF_HZ  10
/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation ->
 * attenuation = fs / 2*pi*f0
 */
#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
#define IMU_ACC_IIR_LPF_ATT_FACTOR  (int32_t)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)


#define IMU_GYRO_WANTED_LPF_CUTOFF_HZ  1

#define IMU_GYRO_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_GYRO_WANTED_LPF_CUTOFF_HZ))
#define IMU_GYRO_IIR_LPF_ATT_FACTOR  (int32_t)(((1<<IIR_SHIFT) / IMU_GYRO_IIR_LPF_ATTENUATION) + 0.5)

#define IMU_GYRO_CALIBRATION_COUNTMAX	((IMU_UPDATE_FREQ / IMU_GYRO_WANTED_LPF_CUTOFF_HZ) * 3)

#define MPU6050_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6050_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6050_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6050_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6050_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6050_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

#define IMU_DEG_PER_LSB_CFG   MPU6050_DEG_PER_LSB_1000
#define IMU_G_PER_LSB_CFG     MPU6050_G_PER_LSB_8

#define GYRO_LPF_TRESH	((IMU_UPDATE_FREQ / IMU_GYRO_WANTED_LPF_CUTOFF_HZ) * 3)
#define ACCEL_LPF_TRESH	((IMU_UPDATE_FREQ / IMU_ACC_WANTED_LPF_CUTOFF_HZ) * 10)

#ifndef M_PI
/** The constant \a pi.	*/
#define M_PI		3.14159265358979323846	/* pi */
#endif

typedef struct{volatile int16_t x;volatile int16_t y;volatile int16_t z;} _3Axis16i_t;
typedef struct{volatile int32_t x;volatile int32_t y;volatile int32_t z;} _3Axis32i_t;
typedef struct{volatile float x;volatile float y;volatile float z;} _3Axisf_t;
	
enum{AHRS_WaitInit,AHRS_Cali_Gyro,AHRS_Cali_Accel,AHRS_Running,};
	
extern uint8_t gAHRSSend[];

extern uint8_t gAHRSStat;
extern volatile uint32_t gAHRSCounter;

extern volatile uint8_t gRunningPrint;

extern _3Axis16i_t gMPU6050RawGyro;
extern _3Axis32i_t gMPU6050RawGyroStoredFilterValues;
extern _3Axis16i_t gMPU6050LPFGyro;
extern _3Axis16i_t gMPU6050BiasGyro;
extern _3Axisf_t	gGyro;

extern _3Axis16i_t gMPU6050RawAccel;
extern _3Axis32i_t gMPU6050RawAccelStoredFilterValues;
extern _3Axis16i_t gMPU6050LPFAccel;
extern _3Axis16i_t gMPU6050BiasAccel;
extern _3Axisf_t	gAccel;

extern float eulerRollActual;
extern float eulerPitchActual;
extern float eulerYawActual;

extern volatile uint8_t gGyroCaliPrintEN;
extern volatile uint8_t gAccelCaliPrintEN;

extern volatile uint32_t gGyroCaliPrintCounter;
extern volatile uint32_t gAccelCaliPrintCounter;

void AHRS_AccelIIRLPFilter(void);
void AHRS_GyroIIRLPFilter(void);
void AHRS_GyroBiasSet(void);
void AHRS_Normalize(void);
void AHRS_GetEulerRPY(void);
void BYYS_Quat2Angle(void);
void Accl_Cali_Print(void);
void Gyro_Cali_Print(void);
void Running_Print(void);

#endif /* AHRS_H_ */
