#include "stm32f10x.h"
#pragma once
#include "Util/fat_fs/inc/ff.h"
#include "Util/buffer.h"
#include "Util/delay.h"
#include "Sensors/amb_sensors.h"

//externs for all the globals
extern volatile uint8_t Pressure_control;
extern volatile float Pressure_Setpoint;

extern volatile uint32_t Millis;

extern volatile uint8_t System_state_Global;
extern volatile uint8_t Sensors;

#define PRE_SIZE 1000000ul	/*Preallocate size*/

#define SYSTEM_STATES 4		/*Number of different control states- atm just four status codes implimented*/

#define WATCHDOG_TIMEOUT 3000	/*4 second timeout - enough for the uSD card to block its max time and a bit*/

#define ITERATION_RATE 100	/*System timer*/

//Battery specific config goes here
#define BATTERY_STARTUP_LIMIT 3.7f /*Around 25% capacity remaining for lithium polymer at 25C slow discharge*/
#if BOARD<3
	#define MINIMUM_VOLTAGE 3.0	/* A single lithium polymer cell*/
#else
	#define MINIMUM_VOLTAGE 3.37	/* A single lithium polymer cell through LDO regulator - no smps on later boards*/
#endif

//function prototypes
void __fat_print_char(char c);
void __str_print_char(char c);
uint8_t detect_sensors(uint8_t noini);
void file_preallocation_control(FIL* file);
//buffer globals
extern volatile Sparkfun_9DOF_buff sfe_sensor_buffers[2];//Data from sparkfun sensors
extern volatile Forehead_sensor_buff forehead_buffer;	//Data from forehead sensors
//fatfs globals
extern volatile uint8_t file_opened;
extern FIL FATFS_logfile, FATFS_wavfile_accel, FATFS_wavfile_gyro;
