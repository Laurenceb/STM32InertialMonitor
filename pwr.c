#include "pwr.h"

void setuppwr() {
	PWR_DeInit();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//clk to the pwr control
}

void shutdown() {
	PWR_WakeUpPinCmd(ENABLE);			//enable the pin
	PWR_EnterSTANDBYMode();				//only wakes on RTC signals or WKUP pin
}

void disable_pin() {
	PWR_WakeUpPinCmd(DISABLE);			//disable the pin
}

void shutdown_filesystem(uint8_t reason, uint8_t file_flags) {
	uint8_t c[25];
	if(reason)
		memcpy(c,"\r\nLogger turned off\r\n",21);
	else
		memcpy(c,"\r\nLow Battery\r\n",15);
	if(file_flags&0x01) {
		uint8_t a;
		f_write(&FATFS_logfile,c,sizeof(c),&a);	//Write the error to the file
		f_sync(&FATFS_logfile);			//Flush buffers
		f_truncate(&FATFS_logfile);		//Truncate the lenght - fix pre allocation
		f_close(&FATFS_logfile);		//Close any opened file
	}
	if(file_flags&0x02)
		wave_terminate(&FATFS_wavfile_accel);	//Close all opened files - terminate wav files correctly
	if(file_flags&0x04)
		wave_terminate(&FATFS_wavfile_gyro);	//Close all opened files - terminate wav files correctly
}
