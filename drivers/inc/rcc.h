#ifndef RCC_H
#define RCC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32system.h"

// RCC Register map
#define RCC_APB2_AFIO 		((uint32_t)0x00000001)
#define RCC_APB2_GPIOA 		((uint32_t)0x00000004)
#define RCC_APB2_GPIOB 		((uint32_t)0x00000008)
#define RCC_APB2_GPIOC 		((uint32_t)0x00000010)
#define RCC_APB2_ADC1 		((uint32_t)0x00000200)
#define RCC_APB2_ADC2 		((uint32_t)0x00000400)
#define RCC_APB2_TIM1 		((uint32_t)0x00000800)
#define RCC_APB2_SPI1 		((uint32_t)0x00001000)
#define RCC_APB2_USART1 	((uint32_t)0x00004000)

#define RCC_APB1_TIM2 		((uint32_t)0x00000001)
#define RCC_APB1_TIM3 		((uint32_t)0x00000002)
#define RCC_APB1_TIM4 		((uint32_t)0x00000004)
#define RCC_APB1_WWDG 		((uint32_t)0x00000800)
#define RCC_APB1_SPI2 		((uint32_t)0x00004000)
#define RCC_APB1_USART2 	((uint32_t)0x00020000)
#define RCC_APB1_USART3 	((uint32_t)0x00040000)
#define RCC_APB1_I2C1 		((uint32_t)0x00200000)
#define RCC_APB1_I2C2 		((uint32_t)0x00400000)
#define RCC_APB1_USB 		((uint32_t)0x00800000)
#define RCC_APB1_CAN 		((uint32_t)0x02000000)
#define RCC_APB1_BKP 		((uint32_t)0x08000000)
#define RCC_APB1_PWR 		((uint32_t)0x10000000)


void RCC_Clock72MHz_HSE(void);
void RCC_APB2ClockCmd(uint32_t periph_mask, FunctionalState state);
void RCC_APB1ClockCmd(uint32_t periph_mask, FunctionalState state);

#ifdef __cplusplus
}
#endif

#endif /* RCC_H */
