#pragma once
#include "../Util/buffer.h"
#include "../i2c_int.h"
#include "../Util/filter_1350.h"
#include "../Util/filter_380.h"

typedef struct{
	buff_type accel[3],magno[3],gyro[3],temp;
}	Sparkfun_9DOF_buff;

typedef struct{
	buff_type accel[3],gyro[3],temp;
}	Forehead_sensor_buff;

enum{FOREHEAD_ACCEL_FIFO=0,FOREHEAD_ACCEL,FOREHEAD_GYRO_FIFO,FOREHEAD_GYRO,SFE_1_ACCEL,SFE_1_MAGNO,SFE_1_GYRO,FOREHEAD_TEMP,SFE_2_ACCEL,\
SFE_2_MAGNO,SFE_2_GYRO};//sensor enums for sensor detect
enum{LSM330_ACCEL_CONFIG_JOB=SFE_2_GYRO+1,LSM330_ACCEL_FIFO_JOB,LSM330_GYRO_CONFIG_JOB,LSM330_GYRO_FIFO_JOB,\
ADXL_CONFIG_JOB,HMC_CONFIG_JOB,ITG3200_CONFIG_JOB};//config job numbers for i2c driver

#define SECOND_BUS_READS (uint32_t)((1<<SFE_2_ACCEL)|(1<<SFE_2_MAGNO)|(1<<SFE_2_GYRO))
#define FIRST_BUS_READS (uint32_t)((1<<FOREHEAD_ACCEL_FIFO)|(1<<FOREHEAD_GYRO_FIFO)|(1<<SFE_1_ACCEL)|(1<<SFE_1_MAGNO)|(1<<SFE_1_GYRO)|(1<<FOREHEAD_TEMP))
#define SCHEDULE_CONFIG_FIRST_BUS (uint32_t)((1<<LSM330_ACCEL_CONFIG_JOB)|(1<<LSM330_ACCEL_FIFO_JOB)|(1<<LSM330_GYRO_CONFIG_JOB)|\
				(1<<LSM330_GYRO_FIFO_JOB)|(1<<ADXL_CONFIG_JOB)|(1<<HMC_CONFIG_JOB)|(1<<ITG3200_CONFIG_JOB))
#define SCHEDULE_CONFIG_SECOND_BUS (uint32_t)((1<<ADXL_CONFIG_JOB)|(1<<HMC_CONFIG_JOB)|(1<<ITG3200_CONFIG_JOB))

#define LAST_JOB ITG3200_CONFIG_JOB

//All the sensors are configured for 100Hz data output

#define LSM_330_ACCEL_ADDR 0x30					/*this will be 0x32 if address pin is floating*/
#define LSM_330_ACCEL_DATA_BYTES 8
#define LSM_330_ACCEL_DATA_SUB 0xA8
#define LSM_330_ACCEL_CONFIG_SUB 0xA0
#define LSM_330_ACCEL_CONFIG {0x97,0x00,0x00,0x38,0x40,0x00}	/*normal mode with 1344 sps and 150hz bandwidth, +-16G*/
#define LSM_330_ACCEL_FIFO_SUB 0xAE
#define LSM_330_ACCEL_FIFO_CONFIG {0x80}

#define LSM_330_GYRO_ADDR 0xD2
#define LSM_330_GYRO_DATA_BYTES 8
#define LSM_330_GYRO_TEMP_BYTES 1
#define LSM_330_GYRO_DATA_SUB 0xA8
#define LSM_330_GYRO_TEMP_SUB 0xA6
#define LSM_330_GYRO_CONFIG_SUB 0xA0
#define LSM_330_GYRO_CONFIG {0xBF,0x20,0x00,0xB0,0x40}		/*380sps with 100hz bandwidth*/
#define LSM_330_GYRO_FIFO_SUB 0xAE
#define LSM_330_GYRO_FIFO_CONFIG {0x40}

#define ADXL_ADDR 0xA6
#define ADXL_DATA_BYTES 6
#define ADXL_DATA_SUB 0x32
#define ADXL_CONFIG_SUB 0x2C
#define ADXL_CONFIG {0x0A,0x08,0x00,0x00,0x00,0x0B}

#define HMC_ADDR 0x3C
#define HMC_DATA_BYTES 6
#define HMC_DATA_SUB 0x03
#define HMC_CONFIG_SUB 0x00
#define HMC_CONFIG {0x18,0x00,0x00}

#define ITG3200_ADDR 0xD0
#define ITG3200_DATA_BYTES 8
#define ITG3200_DATA_SUB 0x1B
#define ITG3200_CONFIG_SUB 0x15
#define ITG3200_CONFIG {0x09,0x1B,0x00}

#define I2C_JOBS_INITIALISER {	/*Device reads*/\
{LSM_330_ACCEL_ADDR,	I2C_Direction_Receiver,	1,				LSM_330_ACCEL_FIFO_SUB+1,NULL}, \
{LSM_330_ACCEL_ADDR,	I2C_Direction_Receiver,	LSM_330_ACCEL_DATA_BYTES,	LSM_330_ACCEL_DATA_SUB,	NULL}, \
{LSM_330_GYRO_ADDR,	I2C_Direction_Receiver,	1,				LSM_330_GYRO_FIFO_SUB+1,NULL}, \
{LSM_330_GYRO_ADDR,	I2C_Direction_Receiver,	LSM_330_GYRO_DATA_BYTES,	LSM_330_GYRO_DATA_SUB,	NULL}, \
{ADXL_ADDR,		I2C_Direction_Receiver,	ADXL_DATA_BYTES,		ADXL_DATA_SUB,		NULL}, \
{HMC_ADDR,		I2C_Direction_Receiver,	HMC_DATA_BYTES,			HMC_DATA_SUB,		NULL}, \
{ITG3200_ADDR,		I2C_Direction_Receiver,	ITG3200_DATA_BYTES,		ITG3200_DATA_SUB,	NULL}, \
{LSM_330_GYRO_ADDR,	I2C_Direction_Receiver,	LSM_330_GYRO_TEMP_BYTES,	LSM_330_GYRO_TEMP_SUB,	NULL}, \
{ADXL_ADDR,		I2C_Direction_Receiver,	ADXL_DATA_BYTES,		ADXL_DATA_SUB,		NULL}, \
{HMC_ADDR,		I2C_Direction_Receiver,	HMC_DATA_BYTES,			HMC_DATA_SUB,		NULL}, \
{ITG3200_ADDR,		I2C_Direction_Receiver,	ITG3200_DATA_BYTES,		ITG3200_DATA_SUB,	NULL}, \
				/*Device configuration*/\
{LSM_330_ACCEL_ADDR,	I2C_Direction_Transmitter,	sizeof(LSM_330_ACCEL_config),	LSM_330_ACCEL_CONFIG_SUB,	LSM_330_ACCEL_config}, \
{LSM_330_ACCEL_ADDR,	I2C_Direction_Transmitter,	sizeof(LSM_330_ACCEL_FIFO_config),LSM_330_ACCEL_FIFO_SUB,	LSM_330_ACCEL_FIFO_config}, \
{LSM_330_GYRO_ADDR,	I2C_Direction_Transmitter,	sizeof(LSM_330_GYRO_config),	LSM_330_GYRO_CONFIG_SUB,	LSM_330_GYRO_config}, \
{LSM_330_GYRO_ADDR,	I2C_Direction_Transmitter,	sizeof(LSM_330_GYRO_FIFO_config),LSM_330_GYRO_FIFO_SUB,		LSM_330_GYRO_FIFO_config}, \
{ADXL_ADDR,		I2C_Direction_Transmitter,	sizeof(ADXL_config),		ADXL_CONFIG_SUB,		ADXL_config}, \
{HMC_ADDR,		I2C_Direction_Transmitter,	sizeof(HMC_config),		HMC_CONFIG_SUB,			HMC_config}, \
{ITG3200_ADDR,		I2C_Direction_Transmitter,	sizeof(ITG3200_config),		ITG3200_CONFIG_SUB,		ITG3200_config}, \
}

#define LSM330_GYRO_RAW_SAMPLE_RATE 380   /* Sampling rates in samples per second */
#define LSM330_ACCEL_RAW_SAMPLE_RATE 1350 /* These are rounded up to the nearest 10hz interval */

extern I2C_Job_Type I2C_jobs[];

extern volatile uint8_t LSM330_Accel_Reads;
extern volatile uint8_t LSM330_Gyro_Reads;

extern volatile uint8_t Rawdata[9][8];
extern volatile uint8_t RawFifo[2][84];

extern SampleFilter	LSM330_Accel_Filter[3],LSM330_Gyro_Filter[3];

void Allocate_Sensor_Buffers(uint8_t samples);
void Configure_I2C_Driver(void);
void Fill_Sample_Buffers(void);
