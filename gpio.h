#include "stm32f10x.h"

#define RED		GPIO_Pin_12
#define GREEN		GPIO_Pin_11
#define TST		GPIO_Pin_8

#define WKUP		GPIO_Pin_0
#define PWREN		GPIO_Pin_15
#define CHARGER_EN	GPIO_Pin_2
#define SD_SEL_PIN	GPIO_Pin_14

#define USB_SOURCE	0x01

#define GREEN_LED_ON	GPIO_WriteBit(GPIOB,GREEN,Bit_SET)
#define GREEN_LED_OFF	GPIO_WriteBit(GPIOB,GREEN,Bit_RESET)
#define RED_LED_ON	GPIO_WriteBit(GPIOB,RED,Bit_SET)
#define RED_LED_OFF	GPIO_WriteBit(GPIOB,RED,Bit_RESET)

#define GET_CHRG_STATE  GPIO_ReadInputDataBit(GPIOB,CHARGER_EN)
#define CHRG_ON		GPIO_WriteBit(GPIOB,CHARGER_EN,Bit_SET)
#define CHRG_OFF	GPIO_WriteBit(GPIOB,CHARGER_EN,Bit_RESET)


#define GET_PWR_STATE	GPIO_ReadInputDataBit(GPIOA,WKUP)

//I2C1 on pins 6 and 7 or 8 and 9 - configured in i2c_int.h
#define I2C1_SCL	GPIO_Pin_6
#define I2C1_SDA	GPIO_Pin_7
#define I2C1_SCL_RE	GPIO_Pin_8
#define I2C1_SDA_RE	GPIO_Pin_9

extern uint8_t bootsource;

void setup_gpio(void);
void switch_leds_on(void);
void switch_leds_off(void);
void red_flash(void);
uint8_t get_wkup(void);
