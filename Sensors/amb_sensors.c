#include "amb_sensors.h"
#include "../i2c_int.h"
#include "../main.h"

volatile uint8_t Rawdata[8][8];			//This is an array for 8 sensors, each one having up to 8 (4*16bit) bytes of data per sample

const uint8_t LSM_330_ACCEL_config[]=LSM_330_ACCEL_CONFIG;//Config arrays for the sensors
const uint8_t LSM_330_GYRO_config[]=LSM_330_GYRO_CONFIG;
const uint8_t ADXL_config[]=ADXL_CONFIG;
const uint8_t HMC_config[]=HMC_CONFIG;
const uint8_t ITG3200_config[]=ITG3200_CONFIG;

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
		init_buffer(&(forehead_buffer.accel[n]),samples);
		init_buffer(&(forehead_buffer.gyro[n]),samples);
	}
}

void Configure_I2C_Driver(void) {
	//Setup the pointers to the read data
	for(uint8_t n=0;n<8;n++)
		I2C1_Setup_Job(n, (volatile uint8_t*)&(Rawdata[n][0]));//Each data buffer
}

//Loads the data from the raw i2c driver into the data buffers as 16 bit integers
void Fill_Sample_Buffers(void) {
	for(uint8_t m=0;m<2;m++) {		//Init the two sparkfun sensor buffers
		for(uint8_t n=0;n<3;n++) {
			Add_To_Buffer(*(uint16_t*)&(Rawdata[2+3*m][2*n]),		&(sfe_sensor_buffers[m].accel[n]));
			Add_To_Buffer(Flipedbytes(*(uint16_t*)&(Rawdata[3+3*m][2*n])),	&(sfe_sensor_buffers[m].magno[n]));
			Add_To_Buffer(Flipedbytes(*(uint16_t*)&(Rawdata[4+3*m][2*n+2])),&(sfe_sensor_buffers[m].gyro[n]));//+2 as we skip the temperature
		}
		Add_To_Buffer(Flipedbytes(*(uint16_t*)&(Rawdata[4+3*m][0])),&(sfe_sensor_buffers[m].temp));//the temperature from the itg3200
	}
	//Add the lsm330 temperature sensor (this is only 8 bit output)	
	Add_To_Buffer((uint16_t)Rawdata[1][0],&(forehead_buffer.temp));
	for(uint8_t n=0;n<3;n++) {
		Add_To_Buffer(*(uint16_t*)&(Rawdata[0][2*n]),		&(forehead_buffer.accel[n]));
		Add_To_Buffer(*(uint16_t*)&(Rawdata[1][2*n+2]),		&(forehead_buffer.gyro[n]));//+2 as we skip the temperature
	}
}
