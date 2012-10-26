#include <string.h>
#include "stm32f10x.h"
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "interrupts.h"
#include "watchdog.h"
#include "Util/rprintf.h"
#include "Util/delay.h"
#include "Sensors/amb_sensors.h"
#include "usb_lib.h"
#include "Util/USB/hw_config.h"
#include "Util/USB/usb_pwr.h"
#include "Util/fat_fs/inc/diskio.h"
#include "Util/fat_fs/inc/ff.h"
#include "Util/fat_fs/inc/integer.h"
#include "Util/fat_fs/inc/rtc.h"

//Global variables - other files (e.g. hardware interface/drivers) may have their own globals
extern uint16_t MAL_Init (uint8_t lun);			//For the USB filesystem driver
volatile uint8_t file_opened=0;				//So we know to close any opened files before turning off
uint8_t print_string[256];				//For printf data
UINT a;							//File bytes counter
volatile uint32_t Millis;				//System uptime (rollover after 50 days)
volatile uint8_t System_state_Global;			//Stores the system state, controlled by the button, most significant bit is a flag
volatile uint8_t Sensors;				//Global holding a mask of the sensors found by automatic sensor discovery
uint8_t Sensor_Cable;					//ID for the attached cable
volatile float battery_voltage;				//Used to flush the adc
//Sensor buffers to pass data back to logger
volatile Sparkfun_9DOF_buff sfe_sensor_buffers[2];	//Data from sparkfun sensors
volatile Forehead_sensor_buff forehead_buffer;		//Data from forehead sensors
//FatFs filesystem globals go here
FRESULT f_err_code;
static FATFS FATFS_Obj;
FIL FATFS_logfile;
FILINFO FATFS_info;
//volatile int bar[3] __attribute__ ((section (".noinit"))) ;//= 0xaa

int main(void)
{
	uint32_t data_counter=0;			//used as data timestamp
	uint8_t deadly_flashes=0,system_state=0;
	int16_t sensor_data;				//used for handling data passed back from sensors
	RTC_t RTC_time;
	SystemInit();					//Sets up the clk
	setup_gpio();					//Initialised pins, and detects boot source
	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);	//Watchdog stopped during JTAG halt
	if(RCC->CSR&RCC_CSR_IWDGRSTF) {			//Watchdog reset, turn off
		RCC->CSR|=RCC_CSR_RMVF;			//Reset the reset flags
		shutdown();
	}
	SysTick_Configuration();			//Start up system timer at 100Hz for uSD card functionality
	Watchdog_Config(WATCHDOG_TIMEOUT);		//Set the watchdog
	Watchdog_Reset();				//Reset watchdog as soon as possible incase it is still running at power on
	rtc_init();					//Real time clock initialise - (keeps time unchanged if set)
	Usarts_Init();
	ISR_Config();
	rprintfInit(__usart_send_char);			//Printf over the bluetooth
	if(USB_SOURCE==bootsource) {
		Set_System();				//This actually just inits the storage layer
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
		uint32_t nojack=0x000FFFFF;		//Countdown timer - a few hundered ms of 0v on jack detect forces a shutdown
		while (bDeviceState != CONFIGURED) {	//Wait for USB config - timeout causes shutdown
			if(Millis>10000 || !nojack)	//No USB cable - shutdown (Charger pin will be set to open drain, cant be disabled without usb)
				shutdown();
			if(GET_CHRG_STATE)		//Jack detect resets the countdown
				nojack=0x0FFFFF;
			nojack--;
			Watchdog_Reset();		//Reset watchdog here, if we are stalled here the Millis timeout should catch us
		}
		USB_Configured_LED();
		EXTI_ONOFF_EN();			//Enable the off interrupt - allow some time for debouncing
		while(1) {				//If running off USB (mounted as mass storage), stay in this loop - dont turn on anything
			if(Millis%1000>500)		//1Hz on/off flashing
				switch_leds_on();	//Flash the LED(s)
			else
				switch_leds_off();
			Watchdog_Reset();
		}
	}
	else {
		if(!GET_PWR_STATE)			//Check here to make sure the power button is still pressed, if not, sleep
			shutdown();			//This means a glitch on the supply line, or a power glitch results in sleep
		if((f_err_code = f_mount(0, &FATFS_Obj)))Usart_Send_Str((char*)"FatFs mount error\r\n");//This should only error if internal error
		else {					//FATFS initialised ok, try init the card, this also sets up the SPI1
			if(!f_open(&FATFS_logfile,"time.txt",FA_OPEN_EXISTING | FA_READ | FA_WRITE)) {//Try and open a time file to get the system time
				if(!f_stat((const TCHAR *)"time.txt",&FATFS_info)) {//Get file info
					if(!FATFS_info.fsize) {//Empty file
						RTC_time.year=(FATFS_info.fdate>>9)+1980;//populate the time struct (FAT start==1980, RTC.year==0)
						RTC_time.month=(FATFS_info.fdate>>5)&0x000F;
						RTC_time.mday=FATFS_info.fdate&0x001F;
						RTC_time.hour=(FATFS_info.ftime>>11)&0x001F;
						RTC_time.min=(FATFS_info.ftime>>5)&0x003F;
						RTC_time.sec=(FATFS_info.ftime<<1)&0x003E;
						rtc_settime(&RTC_time);
						rprintfInit(__fat_print_char);//printf to the open file
						printf("RTC set to %d/%d/%d %d:%d:%d\n",RTC_time.mday,RTC_time.month,RTC_time.year,\
						RTC_time.hour,RTC_time.min,RTC_time.sec);
					}				
				}
				f_close(&FATFS_logfile);//Close the time.txt file
			}
			rtc_gettime(&RTC_time);				//Get the RTC time and put a timestamp on the start of the file
			rprintfInit(__str_print_char);			//Print to the string
			printf("%d-%d-%dT%d-%d-%d.txt",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);//timestamp name
			rprintfInit(__usart_send_char);			//Printf over the bluetooth
			if((f_err_code=f_open(&FATFS_logfile,print_string,FA_CREATE_ALWAYS | FA_WRITE))) {//Present
				printf("FatFs drive error %d\r\n",f_err_code);
				if(f_err_code==FR_DISK_ERR || f_err_code==FR_NOT_READY)
					Usart_Send_Str((char*)"No uSD card inserted?\r\n");
			}
			else {				//We have a mounted card
				print_string[0]=0x00;	//Wipe the string
				f_err_code=f_lseek(&FATFS_logfile, PRE_SIZE);// Pre-allocate clusters
				if (f_err_code || f_tell(&FATFS_logfile) != PRE_SIZE)// Check if the file size has been increased correctly
					Usart_Send_Str((char*)"Pre-Allocation error\r\n");
				else {
					if((f_err_code=f_lseek(&FATFS_logfile, 0)))//Seek back to start of file to start writing
						Usart_Send_Str((char*)"Seek error\r\n");
					else
						rprintfInit(__str_print_char);//Printf to the logfile
				}
				if(f_err_code)
					f_close(&FATFS_logfile);//Close the already opened file on error
				else
					file_opened=1;	//So we know to close the file properly on shutdown
			}
		}
		ADC_Configuration();			//At present this is purely here to detect low battery
		do {
			battery_voltage=GET_BATTERY_VOLTAGE;//Have to flush adc for some reason
			Delay(10000);
		} while(abs(GET_BATTERY_VOLTAGE-battery_voltage)>0.01);
		EXTI_ONOFF_EN();			//Enable the off interrupt - allow some time for debouncing
		I2C_Config();				//Setup the I2C bus
		Sensors=detect_sensors();		//Search for connected sensors
		if(battery_voltage<BATTERY_STARTUP_LIMIT)
			deadly_flashes=1;
		if(!(Sensors&(1<<FOREHEAD_ACCEL)))	//Check for any missing sensors
			deadly_flashes=2;
		if(!(Sensors&(1<<FOREHEAD_GYRO)))
			deadly_flashes=3;
		if(!(Sensors&(1<<SFE_1_ACCEL)))
			deadly_flashes=4;
		if(!(Sensors&(1<<SFE_1_MAGNO)))
			deadly_flashes=5;
		if(!(Sensors&(1<<SFE_1_GYRO)))
			deadly_flashes=6;
		if(!(Sensors&(1<<SFE_2_ACCEL)))
			deadly_flashes=7;
		if(!(Sensors&(1<<SFE_2_MAGNO)))
			deadly_flashes=8;
		if(!(Sensors&(1<<SFE_2_GYRO)))
			deadly_flashes=9;		
		//We die, but flash out a number of flashes first
		if(f_err_code || deadly_flashes) {	//There was an init error
			for(;deadly_flashes;deadly_flashes--) {
				RED_LED_ON;
				Delay(200000);
				RED_LED_OFF;
				Delay(200000);
				Watchdog_Reset();
			}
			RED_LED_ON;
			Delay(400000);
			shutdown();			//Abort after a (further )single red flash
		}
	}
	rtc_gettime(&RTC_time);				//Get the RTC time and put a timestamp on the start of the file
	printf("%d-%d-%dT%d:%d:%d\n",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);//ISO 8601 timestamp header
	printf("Battery: %3fV\n",GET_BATTERY_VOLTAGE);	//Get the battery voltage using blocking regular conversion and print
	printf("Sensor Cable ID is:%d\n",Sensor_Cable);	//Print the sensor cable ID in the header
	if(file_opened) {
		f_puts(print_string,&FATFS_logfile);
		print_string[0]=0x00;			//Set string length to 0
	}
	Millis=0;					//Reset system uptime, we have 50 days before overflow
	while (1) {
		Watchdog_Reset();			//Reset the watchdog each main loop iteration
		while(!bytes_in_buff(&(forehead_buffer.temp)));	//Wait for some data - as all the sensor reads are aligned, can just use this one sensor
		printf("%2f,",(float)data_counter/ITERATION_RATE);//the timestamp
		for(uint8_t n=0;n<3;n++) {
			Get_From_Buffer((uint16_t*)&sensor_data,&(forehead_buffer.accel[n]));//Retrive one sample of data
			printf("%d,",sensor_data);	//print the retreived data
		}
		for(uint8_t n=0;n<3;n++) {
			Get_From_Buffer((uint16_t*)&sensor_data,&(forehead_buffer.gyro[n]));//Retrive one sample of data
			printf("%d,",sensor_data);	//print the retreived data
		}
		Get_From_Buffer((uint16_t*)&sensor_data,&(forehead_buffer.temp));//Retrive one sample of data
		printf("%d,",*(int8_t*)&sensor_data);	//LSM sensor outputs a signed 8 bit temperature in degrees C
		for(uint8_t m=0;m<2;m++) {
			for(uint8_t n=0;n<3;n++) {
				Get_From_Buffer((uint16_t*)&sensor_data,&(sfe_sensor_buffers[m].accel[n]));//Retrive one sample of data
				printf("%d,",sensor_data);	//print the retreived data
			}
			for(uint8_t n=0;n<3;n++) {
				Get_From_Buffer((uint16_t*)&sensor_data,&(sfe_sensor_buffers[m].gyro[n]));//Retrive one sample of data
				printf("%d,",sensor_data);	//print the retreived data
			}
			for(uint8_t n=0;n<3;n++) {
				Get_From_Buffer((uint16_t*)&sensor_data,&(sfe_sensor_buffers[m].magno[n]));//Retrive one sample of data
				printf("%d,",sensor_data);	//print the retreived data
			}
			Get_From_Buffer((uint16_t*)&sensor_data,&(sfe_sensor_buffers[m].temp));//Retrive one sample of data
			printf("%d,",sensor_data);
		}
		// ^ data order is
		//time, forehead acc, forehead gyro, forehead temp, sfe1 acc, sfe1 gyro, sfe1 magno, sfe1 temp, sfe2 acc, sfe2 gyro, sfe2 magno, sfe2 temp
		//Other sensors etc can go here
		//Button multipress status
		if(System_state_Global&0x80) {		//A "control" button press
			system_state=System_state_Global&~0x80;//Copy to local variable
			//Any button press implimented functionality can go here
			System_state_Global&=~0x80;	//Wipe the flag bit to show this has been processed
		}
		printf("%d",system_state);		//Terminating newline
		system_state=0;				//Reset this
		printf("\n");				//Terminating newline
		if(file_opened) {
			f_puts(print_string,&FATFS_logfile);
			print_string[0]=0x00;		//Set string length to 0
		}
		data_counter++;				//Counts up for use as a timestamp
		//Deal with file size - may need to preallocate some more
		if(f_size(&FATFS_logfile)-f_tell(&FATFS_logfile)<(PRE_SIZE/2)) {//More than half way through the pre-allocated area
			DWORD size=f_tell(&FATFS_logfile);
			f_lseek(&FATFS_logfile, f_size(&FATFS_logfile)+PRE_SIZE);//preallocate another PRE_SIZE
			f_lseek(&FATFS_logfile, size);	//Seek back to where we were before
		}
		if(Millis%1000>500)			//1Hz on/off flashing
			switch_leds_on();		//Flash the LED(s)
		else
			switch_leds_off();
		if(System_state_Global&0x80) {		//A "control" button press
			System_state_Global&=~0x80;	//Wipe the flag bit to show this has been processed
			//at moment nothing implimented for button press, but system state is printed for use as an event marker
		}
	}
}

/**
  * @brief  Writes a char to logfile
  * @param  Character to write
  * @retval None
  */
void __fat_print_char(char c) {
	f_write(&FATFS_logfile,&c,1,&a);
}

/**
  * @brief  Writes a char to string - use for better logfile performance
  * @param  Character to write
  * @retval None
  */
void __str_print_char(char c) {
	uint8_t a=strlen(print_string)%255;		//Make sure we cant overwrite ram
	print_string[a]=c;				//Append string
	print_string[a+1]=0x00;				//Null terminate
}

/**
  * @brief  Detects which sensors are plugged in, inits buffers for attached peripheral sensors
  * @param  None
  * @retval Bitmask of detected sensors
  */
uint8_t detect_sensors(void) {
	uint32_t millis = Millis;			//Store the time on entry
	uint32_t jobs_completed = 0;
	uint8_t sensors = 0;
	Jobs=SCHEDULE_CONFIG_FIRST_BUS;			//Run the I2C devices config on first bus
	I2C1_Request_Job(LSM330_ACCEL_CONFIG_JOB);
	while(Jobs);//while((I2C1->CR2)&(I2C_IT_EVT));	//Wait for the i2c driver to complete (if we jam here, watchdog should catch us)
	jobs_completed = Completed_Jobs;		//Copy this so it is not overwritten
	if(I2C1error.job == LSM330_ACCEL_CONFIG_JOB) {	//The accel address is different on differing versions of the sensor cable
		I2C_jobs[FOREHEAD_ACCEL].address = LSM_330_ACCEL_ADDR|0x02;
		I2C_jobs[LSM330_ACCEL_CONFIG_JOB].address = LSM_330_ACCEL_ADDR|0x02;//Change the addresses on the forehead accel
		I2C1_Request_Job(LSM330_ACCEL_CONFIG_JOB);
		while(Jobs);				//Wait for completion
		jobs_completed |= Completed_Jobs;
		Sensor_Cable=0x01;			//This cable id functionality could be extended if future adding i2c EEPROM for example
	}
	Remap();					//Remap i2c to bus2
	Jobs=SCHEDULE_CONFIG_SECOND_BUS;		//Run the I2C devices config on second bus
	I2C1_Request_Job(ADXL_CONFIG_JOB);		//Restart i2c by calling this
	while((I2C1->CR2)&(I2C_IT_EVT));		//Wait for the i2c driver to complete (if we jam here, watchdog should catch us), auto unremapped
	sensors=(jobs_completed>>LSM330_ACCEL_CONFIG_JOB)&0x1F;//shift this to set the sensor detected bits
	sensors|=((Completed_Jobs)>>(ADXL_CONFIG_JOB-SFE_2_ACCEL))&0xE0;
	if(sensors==0xFF) {				//All sensors found ok
		Allocate_Sensor_Buffers(50);		//Calls sensor function to allocate buffers - enough for 0.5s of data
		Configure_I2C_Driver();
	}
	return sensors;
}
