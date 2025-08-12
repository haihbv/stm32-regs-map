#include "stm32system.h"
#include "peripherals.h"
#include "iic.h"
#include "ds3231.h"

static uint8_t h, m, s;
void gpio_config(void);
int main(void)
{
	RCC_Clock72MHz_HSE();
	SysTick_Init();
	I2C1_Init();
	ds3231_set_time(3, 43, 0);
	while (1)
	{
		gpio_config();
		ds3231_get_time(&h, &m, &s);
		if (s > 1 && s < 30)
		{
			GPIOC->ODR.REG ^= (1 << 13);
		}
		delay_ms(1000);
	}
}
void gpio_config(void)
{
	RCC_APB2ClockCmd(RCC_APB2_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = GPIO_PIN_13;
	gpio.Speed = GPIO_SPEED_50MHZ;
	GPIO_Init(GPIOC, &gpio);
}
