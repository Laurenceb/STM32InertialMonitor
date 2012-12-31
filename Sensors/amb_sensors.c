#include "amb_sensors.h"
#include "../i2c_int.h"
#include "../main.h"

volatile uint8_t Rawdata[8][8];			//This is an array for 6 sensors + Gyro data/temp, each one having up to 8 (4*16bit) bytes of data per sample
volatile uint8_t RawFifo[30];			//For the forehead Accel
int16_t LastFifo[3];				//Stores previous data for interpolation

const uint8_t LSM_330_ACCEL_config[]=LSM_330_ACCEL_CONFIG;//Config arrays for the sensors
const uint8_t LSM_330_ACCEL_FIFO_config[]=LSM_330_ACCEL_FIFO_CONFIG;
const uint8_t LSM_330_GYRO_config[]=LSM_330_GYRO_CONFIG;
const uint8_t ADXL_config[]=ADXL_CONFIG;
const uint8_t HMC_config[]=HMC_CONFIG;
const uint8_t ITG3200_config[]=ITG3200_CONFIG;


//We have to keep the sensor samples syncronised with the system timer rather than the sensor clocks, so no clock squew
volatile uint8_t I2C_Transactions_State;

SampleFilter	LSM330_Accel_Filter[3],LSM330_Gyro_Filter[3];//Filters used for downsampling the forehead sensor to 100Sps

I2C_Job_Type I2C_jobs[]=I2C_JOBS_INITIALISER;	//Defines the I2C transactions

void Allocate_Sensor_Buffers(uint8_t samples){
	for(uint8_t m=0;m<2;m++) {		//Init the two sparkfun sensor buffers
		for(uint8_t n=0;n<3;n++) {
			init_buffer(&(sfe_sensor_buffers[m].accel[n]),samples);
			init_buffer(&(sfe_sensor_buffers[m].magno[n]),samples);
			init_buffer(&(sfe_sensor_buffers[m].gyro[n]),samples);
		}
		init_buffer(&(sfe_sensor_buffers[m].temp),samples);
	}
	init_buffer(&(forehead_buffer.temp),samples);
	for(uint8_t n=0;n<3;n++) {
		init_buffer(&(forehead_buffer.accel[n]),(samples*LSM330_ACCEL_RAW_SAMPLE_RATE)/100);
		init_buffer(&(forehead_buffer.gyro[n]),(samples*LSM330_GYRO_RAW_SAMPLE_RATE)/100);//Use extra samples here to accomodate the higher sampling rate
		SampleFilter_init_190(&LSM330_Gyro_Filter[n]);//Also initialise the downsampling filters for the Accel and gyro on the LSM330
		SampleFilter_init_1200(&LSM330_Accel_Filter[n]);
	}
}

void Configure_I2C_Driver(void) {
	//Setup the pointers to the read data
	I2C1_Setup_Job(0, (volatile uint8_t*)&(RawFifo[0]));
	for(uint8_t n=2;n<10;n++)
		I2C1_Setup_Job(n, (volatile uint8_t*)&(Rawdata[n-2][0]));//Each data buffer
}

//Loads the data from the raw i2c driver into the data buffers as 16 bit integers
void Fill_Sample_Buffers(uint8_t state) {	//State should be zero or two Init the two sparkfun sensor buffers
	for(uint8_t n=0;n<3;n++) {
		Add_To_Buffer(*(uint16_t*)&(Rawdata[1+4*state][2*n]),		&(sfe_sensor_buffers[state].accel[n]));
		Add_To_Buffer(Flipedbytes(*(uint16_t*)&(Rawdata[2+4*state][2*n])),&(sfe_sensor_buffers[state].magno[n]));
		Add_To_Buffer(Flipedbytes(*(uint16_t*)&(Rawdata[3+4*state][2*n+2])),&(sfe_sensor_buffers[state].gyro[n]));//+2 as we skip the temperature
	}
	Add_To_Buffer(Flipedbytes(*(uint16_t*)&(Rawdata[3+4*state][0])),&(sfe_sensor_buffers[state].temp));//the temperature from the itg3200
	//Add the LSM330 temperature sensor (this is only 8 bit output)	- other LSM330 sensor data is added directly from callbacks
	if(!state)
		Add_To_Buffer((uint16_t)Rawdata[4][0],&(forehead_buffer.temp));
}
