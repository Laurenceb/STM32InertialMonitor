#include <stdlib.h>
#include "interrupts.h"
#include "Sensors/amb_sensors.h"

volatile uint8_t Button_hold_tim;				//Timer for On/Off/Control button functionality
volatile float Battery_Voltage;					//Battery voltage read from systick

/**
  * @brief  Configure all interrupts accept on/off pin
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void ISR_Config(void) {
	NVIC_InitTypeDef   NVIC_InitStructure;
	/* Set the Vector Table base location at 0x08000000 */    
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);      
	//First we configure the systick ISR
	/* Configure one bit for preemption priority */   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable and set SYSTICK Interrupt to the fifth priority */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;	//The 100hz timer triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;	//5th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//Now we configure the I2C Event ISR
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;	//The I2C1 triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;	//Second to highest group priority
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//Now we configure the I2C Error ISR
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;	//The I2C1 triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	//Highest group priority
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure on/off pin interrupt
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void EXTI_ONOFF_EN(void) {
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_DeInit();
	/* Connect EXTI0 Line to PA.0 pin - WKUP*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	if(USB_SOURCE==bootsource)				//If we booted from USB, disconnect gives -ive pulse
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	else
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	//The WKUP triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;	//low group priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/**
  * @brief  This function configures the systick timer to 100hz overflow
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void) {
	RCC_HCLKConfig(RCC_SYSCLK_Div1);			//CLK the periferal - configure the AHB clk
	SysTick_Config(90000);					//SYSTICK at 100Hz - this function also enables the interrupt
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);   //SYSTICK AHB1/8
}

/**
  * @brief  This function handles External line 0 interrupt request.- WKUP ISR
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(USB_SOURCE!=bootsource && GET_CHRG_STATE) {	//Interrupt due to USB insertion - reset to usb mode
			if(file_opened)
				shutdown_filesystem(1,file_opened);
			NVIC_SystemReset();			//Software reset of the system - USB inserted whilst running
		}
		if(USB_SOURCE==bootsource) {
			if(file_opened) 
				shutdown_filesystem(1,file_opened);
			red_flash();				//Flash red led - provides some debouncing on jack removal
			shutdown();				//Shuts down - only wakes up on power pin i.e. WKUP
		}
		/*Called Code goes here*/
		Button_hold_tim=BUTTON_TURNOFF_TIME;
		RED_LED_ON;					//Red LED is used to indicate successful button press to the user
	}
}


/**
  * @brief  This function handles ADC1-2 interrupt requests.- Should only be from the analogu watchdog
  * @param  None
  * @retval None
  */
void ADC1_2_IRQHandler(void) {
	if(ADC_GetITStatus(ADC2, ADC_IT_AWD)) {			//Analogue watchdog was triggered
		if(file_opened) {
			shutdown_filesystem(0,file_opened);
		}
		red_flash();					//Flash red led
		shutdown();					//Shutdown to save battery
	}
	ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
	ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);		//None of these should ever happen, but best to be safe
	ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);		//make sure flags are clear
}


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler - runs at 100hz.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTickHandler(void)
{
	static float I,old_pressure;
	static uint16_t Enabled_iterations;			//Note, this is going to break if we spend long periods with +ive pressure set
	static uint32_t Last_Button_Press;			//Holds the timestamp for the previous button press
	static uint8_t System_state_counter;			//Holds the system state counter
	static uint8_t tmpindex;				//Temp sensor decimator
	static uint8_t acc_samples=0, old_acc_samples=0, gyro_samples=0, old_gyro_samples=0;//Used to syncronise to the raw sensor sample rates with 10sps precison
	//FatFS timer function
	disk_timerproc();
	//Incr the system uptime
	Millis+=10;
	if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC)) {		//We have adc2 converted data from the injected channels
		ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);		//Clear the flag
		//Sets the Rohm motor controller to idle (low current shutdown) state
		//Check the die temperature - not possible on adc1 :-(
		//Device_Temperature=convert_die_temp(ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_3));//The on die temperature sensor
		//Could process some more sensor data here
		Battery_Voltage=(float)ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1)/(SAMPLING_FACTOR);
	}
	ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);		//Trigger the injected channel group
	//Read any I2C bus sensors here (100Hz)
	if(Sensors==0xFF) {
		//First read the sensor buffers into the data sample buffers
		Fill_Sample_Buffers();
		//Calculate the number of FIFO reads for this iteration
		acc_samples+=LSM330_ACCEL_RAW_SAMPLE_RATE/10;	//This is defined in the sensors header file
		LSM330_Accel_Reads=((uint8_t)(acc_samples-old_acc_samples))/10;//Number of consecutive reads
		old_acc_samples+=LSM330_Accel_Reads*10;
		gyro_samples+=LSM330_GYRO_RAW_SAMPLE_RATE/10;	//This is defined in the sensors header file
		LSM330_Gyro_Reads=((uint8_t)(gyro_samples-old_gyro_samples))/10;//Number of consecutive reads
		old_gyro_samples+=LSM330_Gyro_Reads*10;
		//Now set the jobs
		Jobs|=FIRST_BUS_READS;				//Request all first bus reads
		I2C1_Request_Job(FOREHEAD_ACCEL);		//This will automatically cycle through sensor busses
	}
	//Now process the control button functions
	if(Button_hold_tim ) {					//If a button press generated timer has been triggered
		if(GPIO_ReadInputDataBit(GPIOA,WKUP)) {		//Button hold turns off the device
			if(!--Button_hold_tim) {
				shutdown_filesystem(1,file_opened);
				shutdown();			//Turn off the logger after closing any open files
			}
		}
		else {						//Button released - this can only ever run once per press
			RED_LED_OFF;				//Turn off the red LED - used to indicate button press to user
			if(Button_hold_tim<BUTTON_DEBOUNCE) {	//The button has to be held down for longer than the debounce period
				Last_Button_Press=Millis;
				if(++System_state_counter>=SYSTEM_STATES)
					System_state_counter=0;//The system can only have a limited number of states
			}
			Button_hold_tim=0;			//Reset the timer here
		}
	}
	if(Last_Button_Press&&(Millis-Last_Button_Press>BUTTON_MULTIPRESS_TIMEOUT)&&!Button_hold_tim) {//Last press timed out and button is not pressed
		if(!(System_state_Global&0x80))			//The main code has unlocked the global using the bit flag - as it has processed
			System_state_Global=0x80|System_state_counter;//The previous state update
		System_state_counter=0;				//Reset state counter here
		Last_Button_Press=0;				//Reset the last button press timestamp, as the is no button press in play
	}
}

//Included interrupts from ST um0424 mass storage example
#ifndef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_HP_CAN1_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts requests
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
  CTR_HP();
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
  USB_Istr();
}
#endif /* STM32F10X_CL */

#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{ 
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
  
}
#endif /* STM32F10X_HD | STM32F10X_XL*/

#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : OTG_FS_IRQHandler
* Description    : This function handles USB-On-The-Go FS global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void OTG_FS_IRQHandler(void)
{
  STM32_PCD_OTG_ISR_Handler(); 
}
#endif /* STM32F10X_CL */

