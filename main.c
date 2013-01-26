#include <string.h>
#include <math.h>
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
#include "Util/wave.h"
#include "Util/filter_1200.h"
#include "Util/filter_190.h"
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
volatile uint8_t Sensor_Cable;				//ID for the attached cable
volatile float battery_voltage;				//Used to flush the adc
volatile uint8_t Shutdown_System;			//Used to order a system shutdown to sleep mode
//Sensor buffers to pass data back to logger
volatile Sparkfun_9DOF_buff sfe_sensor_buffers[2];	//Data from sparkfun sensors
volatile Forehead_sensor_buff forehead_buffer;		//Data from forehead sensors
//FatFs filesystem globals go here
FRESULT f_err_code;
static FATFS FATFS_Obj;
FIL FATFS_logfile, FATFS_wavfile_accel, FATFS_wavfile_gyro;
FILINFO FATFS_info;
//volatile int bar[3] __attribute__ ((section (".noinit"))) ;//= 0xaa

int main(void)
{
	uint32_t data_counter=0;			//used as data timestamp
	uint8_t deadly_flashes=0,system_state=0,repetition_counter=0;
	int16_t sensor_data, sensor_raw_data[3]={};	//used for handling data passed back from sensors
	int16_t sfe_sensor_ref_buff[2][3],sfe_sensor_ref_buff_old[2][3];//used to detect and fix I2C bus lockup
	RTC_t RTC_time;
	wave_stuffer Gyro_wav_stuffer={0,0},Accel_wav_stuffer={0,0};//Used to controlling wav file bit packing
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
			__WFI();			//Sleep until something arrives
		}
	}
	else {
		if(!GET_PWR_STATE)			//Check here to make sure the power button is still pressed, if not, sleep
			shutdown();			//This means a glitch on the supply line, or a power glitch results in sleep
		EXTI_ONOFF_EN();			//Enable the off interrupt - allow some time for debouncing
		ADC_Configuration();			//At present this is purely here to detect low battery
		do {
			battery_voltage=Battery_Voltage;//Have to flush adc for some reason
			Delay(25000);
		} while(fabs(Battery_Voltage-battery_voltage)>0.01 || !battery_voltage);
		I2C_Config();				//Setup the I2C bus
		Sensors=detect_sensors(0);		//Search for connected sensors
		if(battery_voltage<BATTERY_STARTUP_LIMIT)
			deadly_flashes=1;
		if(!(Sensors&(1<<FOREHEAD_ACCEL)))	//Check for any missing sensors
			deadly_flashes=2;
		if(!(Sensors&(1<<(FOREHEAD_GYRO-1))))
			deadly_flashes=3;
		if(!(Sensors&(1<<(SFE_1_ACCEL-1))))
			deadly_flashes=4;
		if(!(Sensors&(1<<(SFE_1_MAGNO-1))))
			deadly_flashes=5;
		if(!(Sensors&(1<<(SFE_1_GYRO-1))))
			deadly_flashes=6;
		if(!(Sensors&(1<<(SFE_2_ACCEL-3))))
			deadly_flashes=7;
		if(!(Sensors&(1<<(SFE_2_MAGNO-3))))
			deadly_flashes=8;
		if(!(Sensors&(1<<(SFE_2_GYRO-3))))
			deadly_flashes=9;
		if((f_err_code = f_mount(0, &FATFS_Obj)))Usart_Send_Str((char*)"FatFs mount error\r\n");//This should only error if internal error
		else if(!deadly_flashes){		//FATFS and the I2C initialised ok, try init the card, this also sets up the SPI1
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
			rtc_gettime(&RTC_time);		//Get the RTC time and put a timestamp on the start of the file
			rprintfInit(__str_print_char);	//Print to the string
			//timestamp name
			printf("%d-%02d-%02dT%02d-%02d-%02d",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);
			rprintfInit(__usart_send_char);	//Printf over the bluetooth
			f_err_code = f_mkdir(print_string); //Try to make a directory where the logfiles will live
			if(f_err_code) {
				printf("FatFs drive error %d\r\n",f_err_code);
				if(f_err_code==FR_DISK_ERR || f_err_code==FR_NOT_READY)
					Usart_Send_Str((char*)"No uSD card inserted?\r\n");
				repetition_counter=1;
			}
			else
				f_err_code=f_chdir(print_string);//enter our new directory
			if(f_err_code) {
				if(!repetition_counter)
					printf("FatFs drive error entering direcotry %d\r\n",f_err_code);
				repetition_counter=1;
			}
			else {
				strcat(print_string,".csv");
				f_err_code=f_open(&FATFS_logfile,print_string,FA_CREATE_ALWAYS | FA_WRITE);//Try to open the main 100sps csv logfile
			}
			if(f_err_code) {
				if(!repetition_counter)
					printf("FatFs drive error creating logfile %d\r\n",f_err_code);
				repetition_counter=1;
			}
			else {	
				print_string[strlen(print_string)-4]=0x00;	//Wipe the .csv off the string
				strcat(print_string,"_accel.wav");
				f_err_code=f_open(&FATFS_wavfile_accel,print_string,FA_CREATE_ALWAYS | FA_WRITE);//Try to open the accel wav logfile
			}
			if(f_err_code) {
				if(!repetition_counter)
					printf("FatFs drive error creating accel wav file %d\r\n",f_err_code);
				repetition_counter=1;
			}
			else {	
				print_string[strlen(print_string)-9]=0x00;	//Wipe the accel.wav off the string
				strcat(print_string,"gyro.wav");
				f_err_code=f_open(&FATFS_wavfile_gyro,print_string,FA_CREATE_ALWAYS | FA_WRITE);//Try to open the gyro wav logfile
			}
			if(f_err_code) {
				if(!repetition_counter)
					printf("FatFs drive error creating gyro wav file %d\r\n",f_err_code);
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
					file_opened=0x01;//So we know to close the file properly on shutdown - bit mask for the files
				if(!f_err_code) {
					f_err_code=f_lseek(&FATFS_wavfile_accel, PRE_SIZE);// Pre-allocate clusters
					if (f_err_code || f_tell(&FATFS_wavfile_accel) != PRE_SIZE)// Check if the file size has been increased correctly
						Usart_Send_Str((char*)"Pre-Allocation error\r\n");
					else {
						if((f_err_code=f_lseek(&FATFS_wavfile_accel, 0)))//Seek back to start of file to start writing
							Usart_Send_Str((char*)"Seek error\r\n");
					}
					if(f_err_code)
						f_close(&FATFS_wavfile_accel);//Close the already opened file on error
					else
						file_opened|=0x02;//So we know to close the file properly on shutdown - bit mask for the files
				}
				if(!f_err_code) {
					f_err_code=f_lseek(&FATFS_wavfile_gyro, PRE_SIZE);// Pre-allocate clusters
					if (f_err_code || f_tell(&FATFS_wavfile_gyro) != PRE_SIZE)// Check if the file size has been increased correctly
						Usart_Send_Str((char*)"Pre-Allocation error\r\n");
					else {
						if((f_err_code=f_lseek(&FATFS_wavfile_gyro, 0)))//Seek back to start of file to start writing
							Usart_Send_Str((char*)"Seek error\r\n");
					}
					if(f_err_code)
						f_close(&FATFS_wavfile_gyro);//Close the already opened file on error
					else
						file_opened|=0x04;//So we know to close the file properly on shutdown - bit mask for the files
				}
			}
		}
		repetition_counter=0;			//Reset this here	
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
	//ISO8601 timestamp header
	printf("%d-%02d-%02dT%02d:%02d:%02d\n",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);
	printf("Battery: %3fV\n",Battery_Voltage);	//Get the battery voltage using blocking regular conversion and print
	printf("Sensor Cable ID is:%d\n",Sensor_Cable);	//Print the sensor cable ID in the header
	if(file_opened & 0x01) {
		f_puts(print_string,&FATFS_logfile);
		print_string[0]=0x00;			//Set string length to 0
	}
	if(file_opened&0x02)
		write_wave_header(&FATFS_wavfile_accel, 3, LSM330_ACCEL_RAW_SAMPLE_RATE, 16);//Have to use 16bit as only PCM is supported by matlab/octave
	if(file_opened&0x04)
		write_wave_header(&FATFS_wavfile_gyro, 3, LSM330_GYRO_RAW_SAMPLE_RATE, 16);
	Millis=0;					//Reset system uptime, we have 50 days before overflow
	I2C1error.error=0;
	for(uint8_t n=0;n<3;n++) {
		Empty_Buffer(&(forehead_buffer.accel[n]));//Wipe all the buffers so that data is aligned with the start of the recording
		Empty_Buffer(&(forehead_buffer.gyro[n]));
		for(uint8_t m=0;m<2;m++) {
			Empty_Buffer(&(sfe_sensor_buffers[m].accel[n]));
			Empty_Buffer(&(sfe_sensor_buffers[m].magno[n]));
			Empty_Buffer(&(sfe_sensor_buffers[m].gyro[n]));
		}
	}
	Empty_Buffer(&(forehead_buffer.temp));
	Empty_Buffer(&(sfe_sensor_buffers[0].temp));
	Empty_Buffer(&(sfe_sensor_buffers[1].temp));
	while (1) {
		if(!I2C1error.error && repetition_counter<4) {
			Watchdog_Reset();		//Reset the watchdog each main loop iteration if everything looks ok
			if(memcmp(sfe_sensor_ref_buff,sfe_sensor_ref_buff_old,sizeof(sfe_sensor_ref_buff)))//If data differs
				repetition_counter=0;	//New data differs
			else
				repetition_counter++;	//Incriment the lockup detector
			memcpy(sfe_sensor_ref_buff,sfe_sensor_ref_buff_old,sizeof(sfe_sensor_ref_buff_old));//Copy for reference
		}
		else {
			do {
				Sensors=0;		//Set this to zero to stop the systick firing off I2C1 writes
				I2C1error.error=0;	//Reset both of these
				repetition_counter=0;
				I2C_Config();		//Setup the I2C bus
				uint8_t sensors;
				sensors=detect_sensors(1);//Search for connected sensors - argument means the i2c data output buffers are not reinitialised
				Delay(100000);
				Sensors=sensors;
			} while(Sensors !=0xFF);
			Watchdog_Reset();
		}
		while(!bytes_in_buff(&(forehead_buffer.temp)))	//Wait for some data - as all the sensor reads are aligned, can just use this one sensor
			__WFI();			//Sleep until something arrives
		printf("%d.%02d,",data_counter/100,data_counter%100);//the timestamp
		while(bytes_in_buff(&(forehead_buffer.accel[0]))) {//need to loop here and try to grab all the data, as it is sampled faster than 100Hz
			for(uint8_t n=0;n<3;n++) {
				Get_From_Buffer((uint16_t*)&sensor_data,&(forehead_buffer.accel[n]));//Retrive one sample of data
				SampleFilter_put_1200(&LSM330_Accel_Filter[n],(float)sensor_data);//Dump into the low pass filter
				sensor_raw_data[n]=sensor_data;
			}
			write_wave_samples(&FATFS_wavfile_accel, 3, 16, &Accel_wav_stuffer,(uint16_t*)sensor_raw_data);//Put the raw data into the wav file
		}
		for(uint8_t n=0;n<3;n++) {		//Grab the 100Sps downsampled gyro data from the three individual axis filters
			sensor_data=(int16_t)SampleFilter_get_1200(&LSM330_Accel_Filter[n]);
			printf("%d,",sensor_data);	//print the retreived data
		}
		while(bytes_in_buff(&(forehead_buffer.gyro[0]))) {//need to loop here and try to grab all the data, as it is sampled faster than 100Hz
			for(uint8_t n=0;n<3;n++) {
				Get_From_Buffer((uint16_t*)&sensor_data,&(forehead_buffer.gyro[n]));//Retrive one sample of data
				SampleFilter_put_190(&LSM330_Gyro_Filter[n],(float)sensor_data);//Dump into the low pass filter
				sensor_raw_data[n]=sensor_data;
			}
			write_wave_samples(&FATFS_wavfile_gyro, 3, 16, &Gyro_wav_stuffer, (uint16_t*) sensor_raw_data);//Put the raw data into the wav file
		}
		for(uint8_t n=0;n<3;n++) {		//Grab the 100Sps downsampled gyro data from the three individual axis filters
			sensor_data=(int16_t)SampleFilter_get_190(&LSM330_Gyro_Filter[n]);
			printf("%d,",sensor_data);	//print the retreived data
		}
		Get_From_Buffer((uint16_t*)&sensor_data,&(forehead_buffer.temp));//Retrive one sample of data
		printf("%d,",*(int8_t*)&sensor_data);	//LSM sensor outputs a signed 8 bit temperature in degrees C
		for(uint8_t m=0;m<2;m++) {
			for(uint8_t n=0;n<3;n++) {
				Get_From_Buffer((uint16_t*)&sensor_data,&(sfe_sensor_buffers[m].accel[n]));//Retrive one sample of data
				printf("%d,",sensor_data);	//print the retreived data
				sfe_sensor_ref_buff[m][n]=sensor_data;//Reference for lockup detection
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
		if(file_opened & 0x01) {
			f_puts(print_string,&FATFS_logfile);
			print_string[0]=0x00;		//Set string length to 0
		}
		data_counter++;				//Counts up for use as a timestamp
		//Deal with file size - may need to preallocate some more
		file_preallocation_control(&FATFS_logfile);
		file_preallocation_control(&FATFS_wavfile_accel);
		file_preallocation_control(&FATFS_wavfile_gyro);
		if(Millis%1000>500)			//1Hz on/off flashing
			switch_leds_on();		//Flash the LED(s)
		else
			switch_leds_off();
		if(Shutdown_System) {			//A system shutdown has been requested
			if(file_opened)
				shutdown_filesystem(Shutdown_System, file_opened);
			if(Shutdown_System==USB_INSERTED)
				NVIC_SystemReset();	//Software reset of the system - USB inserted whilst running
			else {
				if(Shutdown_System==LOW_BATTERY)
					red_flash();	//Used to indicate an error condition before turnoff
				shutdown();		//Puts us into sleep mode
			}
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
  * @brief  Ensures that we have significant preallocation on a file, useful to avoid significant delays on write
  * @param  Pointer to the file
  * @retval None
  */
void file_preallocation_control(FIL* file) {
	if(f_size(file)-f_tell(file)<(PRE_SIZE/2)) {//More than half way through the pre-allocated area
		DWORD size=f_tell(file);
		f_lseek(file, f_size(file)+PRE_SIZE);//preallocate another PRE_SIZE
		f_lseek(file, size);	//Seek back to where we were before
	}
}

/**
  * @brief  Detects which sensors are plugged in, inits buffers for attached peripheral sensors
  * @param  None
  * @retval Bitmask of detected sensors
  */
uint8_t detect_sensors(uint8_t noini) {
	uint32_t millis = Millis;			//Store the time on entry
	uint32_t jobs_completed = 0;
	uint8_t sensors = 0;
	Jobs=SCHEDULE_CONFIG_FIRST_BUS;			//Run the I2C devices config on first bus
	I2C1_Request_Job(LSM330_ACCEL_CONFIG_JOB);
	while(Jobs) {//while((I2C1->CR2)&(I2C_IT_EVT));//Wait for th i2c driver to complete
		if(Millis>(millis+20))
			return 0;
	}
	jobs_completed = Completed_Jobs;		//Copy this so it is not overwritten
	if(I2C1error.job == LSM330_ACCEL_CONFIG_JOB || I2C1error.job == LSM330_ACCEL_FIFO_JOB) {//Accel address is different on differing sensor cables
		I2C_jobs[FOREHEAD_ACCEL].address = LSM_330_ACCEL_ADDR|0x02;
		I2C_jobs[LSM330_ACCEL_CONFIG_JOB].address = LSM_330_ACCEL_ADDR|0x02;//Change the addresses on the forehead accel
		I2C_jobs[LSM330_ACCEL_FIFO_JOB].address = LSM_330_ACCEL_ADDR|0x02;
		I2C_jobs[FOREHEAD_ACCEL_FIFO].address = LSM_330_ACCEL_ADDR|0x02;
		Sensor_Cable=0x01;			//This cable id functionality could be extended if future adding i2c EEPROM for example
		Jobs|=(1<<LSM330_ACCEL_FIFO_JOB);
		I2C1_Request_Job(LSM330_ACCEL_CONFIG_JOB);
		millis=Millis;
		while(Jobs) {				//Wait for completion
			if(Millis>(millis+20))
				return 0;
		}
		jobs_completed |= Completed_Jobs;
	}
	while(I2C1->CR1 & 0x0300);			//Wait for stop/start bits to clear
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
	asm volatile ("dmb" ::: "memory");
	Remap();					//Remap i2c to bus2 - now handled in the isr
	asm volatile ("dmb" ::: "memory");
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	asm volatile ("dmb" ::: "memory");
	Jobs=SCHEDULE_CONFIG_SECOND_BUS;		//Run the I2C devices config on second bus
	I2C1_Request_Job(ADXL_CONFIG_JOB);		//Restart i2c by calling this
	millis=Millis;
	while( (I2C1->CR2)&(I2C_IT_EVT) || (I2C1->CR1 & 0x0200) ) {//Wait for the i2c driver to complete, and send stop - auto unremapped
		if(Millis>(millis+20))
			return 0;
	}
	sensors=(jobs_completed>>(LSM330_GYRO_CONFIG_JOB-1))&0x1E;//shift this to set the sensor detected bits
	sensors|=(jobs_completed>>LSM330_ACCEL_CONFIG_JOB)&0x01;
	sensors|=((Completed_Jobs)>>(ADXL_CONFIG_JOB-SFE_2_ACCEL))&0xE0;
	if(sensors==0xFF && !noini) {			//All sensors found ok and we are allowed to initialise the buffers
		Allocate_Sensor_Buffers(50);		//Calls sensor function to allocate buffers - enough for 0.5s of data
		Configure_I2C_Driver();
	}
	Delay(20000);
	return sensors;
}
