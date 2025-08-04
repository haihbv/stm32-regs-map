#include "stm32system.h"
#include "peripherals.h"
#include "iic.h"
#include "ds3231.h"

static uint8_t hour, minute, second;
void Gpio_Config(void);
int main(void)
{
	RCC_Clock72MHz_HSE();
	SysTick_Init();
	I2C1_Init();
	DS3231_Set_Time(4, 1, 0);
	while (1)
	{
		Gpio_Config();
		DS3231_Get_Time(&hour, &minute, &second);
		if (second > 1 && second < 30)
		{
			GPIOC->ODR.REG ^= (1 << 13);
		}
		delay_ms(1000);
	}
}
void Gpio_Config(void)
{
	RCC_APB2ClockCmd(RCC_APB2_GPIOC, ENABLE);
	
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = GPIO_PIN_13;
	gpio.Speed = GPIO_SPEED_50MHZ;
	GPIO_Init(GPIOC, &gpio);
}

