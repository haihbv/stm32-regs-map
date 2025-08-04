#ifndef __STM32SYSTEM_H
#define __STM32SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32util.h" // dùng __IO

// Base addresses
#define GPIOA_BASE 				(0x40010800UL)
#define GPIOB_BASE 				(0x40010C00UL)
#define GPIOC_BASE 				(0x40011000UL)
#define RCC_BASE    			(0x40021000UL)
#define FLASH_BASE 				(0x40022000UL)
#define AFIO_BASE 				(0x40010000UL)
#define NVIC_Base 				(0xE000E100UL)
#define EXTI_Base 				(0x40010400UL)
#define TIM1_Base 				(0x40012C00UL)
#define TIM2_Base 				(0x40000000UL)
#define TIM3_Base 				(0x40000400UL)
#define TIM4_Base 				(0x40000800UL)
#define ADC1_Base 				(0x40012400UL)
#define ADC2_Base 				(0x40012800UL)
#define USART1_Base				(0x40013800UL)
#define USART2_Base				(0x40004400UL)
#define USART3_Base				(0x40004800UL)
#define SPI1_Base       		(0x40013000UL)
#define SPI2_Base       		(0x40003800UL)
#define I2C1_Base				(0x40005400UL)
#define I2C2_Base				(0x40005800UL)
#define USB_Base				(0x40005C00UL)

// ===== Peripheral Structs =====
// FLASH registers
typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t LATENCY 		: 3;
		uint32_t HLFCYA 		: 1;
		uint32_t PRFTBE 		: 1;
		uint32_t PRFTBS 		: 1;
		uint32_t RESERVED 		: 26;
	} BITS;
} FLASH_ACR_t;

typedef struct
{
	FLASH_ACR_t 	ACR;
} FLASH_TypeDef;

// RCC registers

typedef union
{
	__IO uint32_t REG;
	struct
	{
		__IO uint32_t HSION 	: 1;
		__IO uint32_t HSIRDY 	: 1;
		__IO uint32_t RESERVED0 : 1;
		__IO uint32_t HSITRIM 	: 5;
		__IO uint32_t HSICAL 	: 8;
		__IO uint32_t HSEON 	: 1;
		__IO uint32_t HSERDY 	: 1;
		__IO uint32_t HSEBYP 	: 1;
		__IO uint32_t CSSON 	: 1;
		__IO uint32_t RESERVED1 : 4;
		__IO uint32_t PLLON 	: 1;
		__IO uint32_t PLLRDY 	: 1;
		__IO uint32_t RESERVED2 : 6;
	} BITS;
} RCC_CR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t SW 			: 2;
		uint32_t SWS 			: 2;
		uint32_t HPRE 			: 4;
		uint32_t PPRE1 			: 3;
		uint32_t PPRE2 			: 3;
		uint32_t ADCPRE 		: 2;
		uint32_t PLLSRC 		: 1;
		uint32_t PLLXTPRE 		: 1;
		uint32_t PLLMUL 		: 4;
		uint32_t USBPRE 		: 1;
		uint32_t RESERVED0 		: 1;
		uint32_t MCO 			: 3;
		uint32_t RESERVED1 		: 5;
	} BITS;
} RCC_CFGR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t LSIRDYF 		: 1;
		uint32_t LSERDYF 		: 1;
		uint32_t HSIRDYF 		: 1;
		uint32_t HSERDYF 		: 1;
		uint32_t PLLRDYF 		: 1;
		uint32_t RESERVED0 		: 2;
		uint32_t CSSF 			: 1;
		uint32_t LSIRDYIE 		: 1;
		uint32_t LSERDYIE 		: 1;
		uint32_t HSIRDYIE 		: 1;
		uint32_t HSERDYIE 		: 1;
		uint32_t PLLRDYIE 		: 1;
		uint32_t RESERVED1 		: 3;
		uint32_t LSIRDYC 		: 1;
		uint32_t LSERDYC 		: 1;
		uint32_t HSIRDYC 		: 1;
		uint32_t HSERDYC 		: 1;
		uint32_t PLLRDYC 		: 1;
		uint32_t RESERVED2 		: 2;
		uint32_t CSSC 			: 1;
		uint32_t RESERVED3 		: 8;
	} BITS;
} RCC_CIR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t AFIORST 		: 1;
		uint32_t RESERVED0 		: 1;
		uint32_t IOPARST 		: 1;
		uint32_t IOPBRST 		: 1;
		uint32_t IOPCRST 		: 1;
		uint32_t IOPDRST 		: 1;
		uint32_t IOPERST 		: 1;
		uint32_t IOPFRST 		: 1;
		uint32_t IOPGRST 		: 1;
		uint32_t ADC1RST 		: 1;
		uint32_t ADC2RST 		: 1;
		uint32_t TIM1RST 		: 1;
		uint32_t SPI1RST 		: 1;
		uint32_t TIM8RST 		: 1;
		uint32_t USART1RST 		: 1;
		uint32_t ADC3RST 		: 1;
		uint32_t RESERVED1 		: 3;
		uint32_t TIM9RST 		: 1;
		uint32_t TIM10RST 		: 1;
		uint32_t TIM11RST 		: 1;
		uint32_t RESERVED2 		: 10;
	} BITS;
} RCC_APB2RSTR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t TIM2RST 		: 1;
		uint32_t TIM3RST 		: 1;
		uint32_t TIM4RST 		: 1;
		uint32_t TIM5RST 		: 1;
		uint32_t TIM6RST 		: 1;
		uint32_t TIM7RST 		: 1;
		uint32_t TIM12RST 		: 1;
		uint32_t TIM13RST 		: 1;
		uint32_t TIM14RST 		: 1;
		uint32_t RESERVED0 		: 2;
		uint32_t WWDGRST 		: 1;
		uint32_t RESERVED1 		: 2;
		uint32_t SPI2RST 		: 1;
		uint32_t SPI3RST 		: 1;
		uint32_t RESERVED2 		: 1;
		uint32_t USART2RST 		: 1;
		uint32_t USART3RST 		: 1;
		uint32_t UART4RST 		: 1;
		uint32_t UART5RST 		: 1;
		uint32_t I2C1RST 		: 1;
		uint32_t I2C2RST 		: 1;
		uint32_t USBRST 		: 1;
		uint32_t RESERVED3 		: 1;
		uint32_t CANRST 		: 1;
		uint32_t RESERVED4 		: 1;
		uint32_t BKPRST 		: 1;
		uint32_t PWRRST 		: 1;
		uint32_t DACRST 		: 1;
		uint32_t RESERVED5 		: 2;
	} BITS;
} RCC_APB1RSTR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t DMA1EN 		: 1;
		uint32_t DMA2EN 		: 1;
		uint32_t SRAMEN 		: 1;
		uint32_t RESERVED0 		: 1;
		uint32_t FLITFEN 		: 1;
		uint32_t RESERVED1 		: 1;
		uint32_t CRCEN 			: 1;
		uint32_t RESERVED2 		: 1;
		uint32_t FSMCEN 		: 1;
		uint32_t RESERVED3 		: 1;
		uint32_t SDIOEN 		: 1;
		uint32_t RESERVED4 		: 21;
	} BITS;
} RCC_AHBENR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t AFIOEN 		: 1;
		uint32_t RESERVED0 		: 1;
		uint32_t IOPAEN 		: 1;
		uint32_t IOPBEN 		: 1;
		uint32_t IOPCEN 		: 1;
		uint32_t IOPDEN 		: 1;
		uint32_t IOPEEN 		: 1;
		uint32_t IOPFEN 		: 1;
		uint32_t IOPGEN 		: 1;
		uint32_t ADC1EN 		: 1;
		uint32_t ADC2EN 		: 1;
		uint32_t TIM1EN 		: 1;
		uint32_t SPI1EN 		: 1;
		uint32_t TIM8EN 		: 1;
		uint32_t USART1EN 		: 1;
		uint32_t ADC3EN 		: 1;
		uint32_t RESERVED1 		: 3;
		uint32_t TIM9EN 		: 1;
		uint32_t TIM10EN 		: 1;
		uint32_t TIM11EN 		: 1;
		uint32_t RESERVED2 		: 10;
	} BITS;
} RCC_APB2ENR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t TIM2EN 		: 1;
		uint32_t TIM3EN 		: 1;
		uint32_t TIM4EN 		: 1;
		uint32_t TIM5EN 		: 1;
		uint32_t TIM6EN 		: 1;
		uint32_t TIM7EN 		: 1;
		uint32_t TIM12EN 		: 1;
		uint32_t TIM13EN 		: 1;
		uint32_t TIM14EN 		: 1;
		uint32_t RESERVED0 		: 2;
		uint32_t WWDGEN 		: 1;
		uint32_t RESERVED1 		: 2;
		uint32_t SPI2EN 		: 1;
		uint32_t SPI3EN 		: 1;
		uint32_t RESERVED2 		: 1;
		uint32_t USART2EN 		: 1;
		uint32_t USART3EN 		: 1;
		uint32_t UART4EN 		: 1;
		uint32_t UART5EN 		: 1;
		uint32_t I2C1EN 		: 1;
		uint32_t I2C2EN 		: 1;
		uint32_t USBEN 			: 1;
		uint32_t RESERVED3 		: 1;
		uint32_t CANEN 			: 1;
		uint32_t RESERVED4 		: 1;
		uint32_t BKPEN 			: 1;
		uint32_t PWREN 			: 1;
		uint32_t DACEN 			: 1;
		uint32_t RESERVED5 		: 2;
	} BITS;
} RCC_APB1ENR_t;

typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t LSEON 			: 1;
		uint32_t LSERDY 		: 1;
		uint32_t LSEBYP 		: 1;
		uint32_t RESERVED0 		: 5;
		uint32_t RTCSEL 		: 2;
		uint32_t RESERVED1 		: 5;
		uint32_t RTCEN 			: 1;
		uint32_t BDRST 			: 1;
		uint32_t RESERVED2 		: 15;
	} BITS;
} RCC_BDCR_t;
typedef union
{
	__IO uint32_t REG;
	struct
	{
		uint32_t LSION 			: 1;
		uint32_t LSIRDY 		: 1;
		uint32_t RESERVED0 		: 22;
		uint32_t RMVF 			: 1;
		uint32_t RESERVED1 		: 1;
		uint32_t PINRSTF 		: 1;
		uint32_t PORRSTF 		: 1;
		uint32_t SFTRSTF 		: 1;
		uint32_t IWDGRSTF 		: 1;
		uint32_t WWDGRSTF 		: 1;
		uint32_t LPWRRSTF 		: 1;
	} BITS;
} RCC_CSR_t;

typedef struct
{
	RCC_CR_t 		CR;
	RCC_CFGR_t 		CFGR;
	RCC_CIR_t 		CIR;
	RCC_APB2RSTR_t 	APB2RSTR;
	RCC_APB1RSTR_t 	APB1RSTR;
	RCC_AHBENR_t 	AHBENR;
	RCC_APB2ENR_t 	APB2ENR;
	RCC_APB1ENR_t 	APB1ENR;
	RCC_BDCR_t 		BDCR;
	RCC_CSR_t 		CSR;
} RCC_TypeDef;

// GPIO registers
typedef union
{
    __IO uint32_t REG;
    struct
    {
        __IO uint32_t MODE0     : 2; /*!< Bits 1:0 MODE0: Port x mode bits (y = 0) */
        __IO uint32_t CNF0      : 2;  /*!< Bits 3:2 CNF0: Port x configuration bits (y = 0) */
        __IO uint32_t MODE1     : 2; /*!< Bits 5:4 MODE1: Port x mode bits (y = 1) */
        __IO uint32_t CNF1      : 2;  /*!< Bits 7:6 CNF1: Port x configuration bits (y = 1) */
        __IO uint32_t MODE2     : 2; /*!< Bits 9:8 MODE2: Port x mode bits (y = 2) */
        __IO uint32_t CNF2      : 2;  /*!< Bits 11:10 CNF2: Port x configuration bits (y = 2) */
        __IO uint32_t MODE3     : 2; /*!< Bits 13:12 MODE3: Port x mode bits (y = 3) */
        __IO uint32_t CNF3      : 2;  /*!< Bits 15:14 CNF3: Port x configuration bits (y = 3) */
        __IO uint32_t MODE4     : 2; /*!< Bits 17:16 MODE4: Port x mode bits (y = 4) */
        __IO uint32_t CNF4      : 2;  /*!< Bits 19:18 CNF4: Port x configuration bits (y = 4) */
        __IO uint32_t MODE5     : 2; /*!< Bits 21:20 MODE5: Port x mode bits (y = 5) */
        __IO uint32_t CNF5      : 2;  /*!< Bits 23:22 CNF5: Port x configuration bits (y = 5) */
        __IO uint32_t MODE6     : 2; /*!< Bits 25:24 MODE6: Port x mode bits (y = 6) */
        __IO uint32_t CNF6      : 2;  /*!< Bits 27:26 CNF6: Port x configuration bits (y = 6) */
        __IO uint32_t MODE7     : 2; /*!< Bits 29:28 MODE7: Port x mode bits (y = 7) */
        __IO uint32_t CNF7      : 2;  /*!< Bits 31:30 CNF7: Port x configuration bits (y = 7) */
    } BITS;
} GPIO_CRL_t;
typedef union
{
    __IO uint32_t REG;
    struct
    {
        __IO uint32_t MODE8     : 2;  /*!< Bits 1:0 MODE8: Port x mode bits (y = 8) */
        __IO uint32_t CNF8      : 2;   /*!< Bits 3:2 CNF8: Port x configuration bits (y = 8) */
        __IO uint32_t MODE9     : 2;  /*!< Bits 5:4 MODE9: Port x mode bits (y = 9) */
        __IO uint32_t CNF9      : 2;   /*!< Bits 7:6 CNF9: Port x configuration bits (y = 9) */
        __IO uint32_t MODE10    : 2; /*!< Bits 9:8 MODE10: Port x mode bits (y = 10) */
        __IO uint32_t CNF10     : 2;  /*!< Bits 11:10 CNF10: Port x configuration bits (y = 10) */
        __IO uint32_t MODE11    : 2; /*!< Bits 13:12 MODE11: Port x mode bits (y = 11) */
        __IO uint32_t CNF11     : 2;  /*!< Bits 15:14 CNF11: Port x configuration bits (y = 11) */
        __IO uint32_t MODE12    : 2; /*!< Bits 17:16 MODE12: Port x mode bits (y = 12) */
        __IO uint32_t CNF12     : 2;  /*!< Bits 19:18 CNF12: Port x configuration bits (y = 12) */
        __IO uint32_t MODE13    : 2; /*!< Bits 21:20 MODE13: Port x mode bits (y = 13) */
        __IO uint32_t CNF13     : 2;  /*!< Bits 23:22 CNF13: Port x configuration bits (y = 13) */
        __IO uint32_t MODE14    : 2; /*!< Bits 25:24 MODE14: Port x mode bits (y = 14) */
        __IO uint32_t CNF14     : 2;  /*!< Bits 27:26 CNF14: Port x configuration bits (y = 14) */
        __IO uint32_t MODE15    : 2; /*!< Bits 29:28 MODE15: Port x mode bits (y = 15) */
        __IO uint32_t CNF15     : 2;  /*!< Bits 31:30 CNF15: Port x configuration bits (y = 15) */
    } BITS;
} GPIO_CRH_t;
typedef union
{
    __IO uint32_t REG;
    struct
    {
        __IO uint32_t IDR0      : 1;      /*!< Bit 0 IDR0: Port input data (y = 0) */
        __IO uint32_t IDR1      : 1;      /*!< Bit 1 IDR1: Port input data (y = 1) */
        __IO uint32_t IDR2      : 1;      /*!< Bit 2 IDR2: Port input data (y = 2) */
        __IO uint32_t IDR3      : 1;      /*!< Bit 3 IDR3: Port input data (y = 3) */
        __IO uint32_t IDR4      : 1;      /*!< Bit 4 IDR4: Port input data (y = 4) */
        __IO uint32_t IDR5      : 1;      /*!< Bit 5 IDR5: Port input data (y = 5) */
        __IO uint32_t IDR6      : 1;      /*!< Bit 6 IDR6: Port input data (y = 6) */
        __IO uint32_t IDR7      : 1;      /*!< Bit 7 IDR7: Port input data (y = 7) */
        __IO uint32_t IDR8      : 1;      /*!< Bit 8 IDR8: Port input data (y = 8) */
        __IO uint32_t IDR9      : 1;      /*!< Bit 9 IDR9: Port input data (y = 9) */
        __IO uint32_t IDR10     : 1;     /*!< Bit 10 IDR10: Port input data (y = 10) */
        __IO uint32_t IDR11     : 1;     /*!< Bit 11 IDR11: Port input data (y = 11) */
        __IO uint32_t IDR12     : 1;     /*!< Bit 12 IDR12: Port input data (y = 12) */
        __IO uint32_t IDR13     : 1;     /*!< Bit 13 IDR13: Port input data (y = 13) */
        __IO uint32_t IDR14     : 1;     /*!< Bit 14 IDR14: Port input data (y = 14) */
        __IO uint32_t IDR15     : 1;     /*!< Bit 15 IDR15: Port input data (y = 15) */
        __IO uint32_t RESERVED  : 16; /*!< Bits 31:16 Reserved, must be kept at reset value */
    } BITS;
} GPIO_IDR_t;
typedef union
{
    __IO uint32_t REG;
    struct
    {
        __IO uint32_t ODR0      : 1;      /*!< Bit 0 ODR0: Port output data (y = 0) */
        __IO uint32_t ODR1      : 1;      /*!< Bit 1 ODR1: Port output data (y = 1) */
        __IO uint32_t ODR2      : 1;      /*!< Bit 2 ODR2: Port output data (y = 2) */
        __IO uint32_t ODR3      : 1;      /*!< Bit 3 ODR3: Port output data (y = 3) */
        __IO uint32_t ODR4      : 1;      /*!< Bit 4 ODR4: Port output data (y = 4) */
        __IO uint32_t ODR5      : 1;      /*!< Bit 5 ODR5: Port output data (y = 5) */
        __IO uint32_t ODR6      : 1;      /*!< Bit 6 ODR6: Port output data (y = 6) */
        __IO uint32_t ODR7      : 1;      /*!< Bit 7 ODR7: Port output data (y = 7) */
        __IO uint32_t ODR8      : 1;      /*!< Bit 8 ODR8: Port output data (y = 8) */
        __IO uint32_t ODR9      : 1;      /*!< Bit 9 ODR9: Port output data (y = 9) */
        __IO uint32_t ODR10     : 1;     /*!< Bit 10 ODR10: Port output data (y = 10) */
        __IO uint32_t ODR11     : 1;     /*!< Bit 11 ODR11: Port output data (y = 11) */
        __IO uint32_t ODR12     : 1;     /*!< Bit 12 ODR12: Port output data (y = 12) */
        __IO uint32_t ODR13     : 1;     /*!< Bit 13 ODR13: Port output data (y = 13) */
        __IO uint32_t ODR14     : 1;     /*!< Bit 14 ODR14: Port output data (y = 14) */
        __IO uint32_t ODR15     : 1;     /*!< Bit 15 ODR15: Port output data (y = 15) */
        __IO uint32_t RESERVED  : 16; /*!< Bits 31:16 Reserved, must be kept at reset value */
    } BITS;
} GPIO_ODR_t;
typedef union
{
    __IO uint32_t REG;
    struct
    {
        __IO uint32_t BS0       : 1;  /*!< Bit 0 BS0: Port x Set bit 0 */
        __IO uint32_t BS1       : 1;  /*!< Bit 1 BS1: Port x Set bit 1 */
        __IO uint32_t BS2       : 1;  /*!< Bit 2 BS2: Port x Set bit 2 */
        __IO uint32_t BS3       : 1;  /*!< Bit 3 BS3: Port x Set bit 3 */
        __IO uint32_t BS4       : 1;  /*!< Bit 4 BS4: Port x Set bit 4 */
        __IO uint32_t BS5       : 1;  /*!< Bit 5 BS5: Port x Set bit 5 */
        __IO uint32_t BS6       : 1;  /*!< Bit 6 BS6: Port x Set bit 6 */
        __IO uint32_t BS7       : 1;  /*!< Bit 7 BS7: Port x Set bit 7 */
        __IO uint32_t BS8       : 1;  /*!< Bit 8 BS8: Port x Set bit 8 */
        __IO uint32_t BS9       : 1;  /*!< Bit 9 BS9: Port x Set bit 9 */
        __IO uint32_t BS10      : 1; /*!< Bit 10 BS10: Port x Set bit 10 */
        __IO uint32_t BS11      : 1; /*!< Bit 11 BS11: Port x Set bit 11 */
        __IO uint32_t BS12      : 1; /*!< Bit 12 BS12: Port x Set bit 12 */
        __IO uint32_t BS13      : 1; /*!< Bit 13 BS13: Port x Set bit 13 */
        __IO uint32_t BS14      : 1; /*!< Bit 14 BS14: Port x Set bit 14 */
        __IO uint32_t BS15      : 1; /*!< Bit 15 BS15: Port x Set bit 15 */
        __IO uint32_t BR0       : 1;  /*!< Bit 16 BR0: Port x Reset bit 0 */
        __IO uint32_t BR1       : 1;  /*!< Bit 17 BR1: Port x Reset bit 1 */
        __IO uint32_t BR2       : 1;  /*!< Bit 18 BR2: Port x Reset bit 2 */
        __IO uint32_t BR3       : 1;  /*!< Bit 19 BR3: Port x Reset bit 3 */
        __IO uint32_t BR4       : 1;  /*!< Bit 20 BR4: Port x Reset bit 4 */
        __IO uint32_t BR5       : 1;  /*!< Bit 21 BR5: Port x Reset bit 5 */
        __IO uint32_t BR6       : 1;  /*!< Bit 22 BR6: Port x Reset bit 6 */
        __IO uint32_t BR7       : 1;  /*!< Bit 23 BR7: Port x Reset bit 7 */
        __IO uint32_t BR8       : 1;  /*!< Bit 24 BR8: Port x Reset bit 8 */
        __IO uint32_t BR9       : 1;  /*!< Bit 25 BR9: Port x Reset bit 9 */
        __IO uint32_t BR10      : 1; /*!< Bit 26 BR10: Port x Reset bit 10 */
        __IO uint32_t BR11      : 1; /*!< Bit 27 BR11: Port x Reset bit 11 */
        __IO uint32_t BR12      : 1; /*!< Bit 28 BR12: Port x Reset bit 12 */
        __IO uint32_t BR13      : 1; /*!< Bit 29 BR13: Port x Reset bit 13 */
        __IO uint32_t BR14      : 1; /*!< Bit 30 BR14: Port x Reset bit 14 */
        __IO uint32_t BR15      : 1; /*!< Bit 31 BR15: Port x Reset bit 15 */
    } BITS;
} GPIO_BSRR_t;
typedef union
{
    __IO uint32_t REG;
    struct
    {
        __IO uint32_t BR0       : 1;       /*!< Bit 0 BR0: Port x Reset bit 0 */
        __IO uint32_t BR1       : 1;       /*!< Bit 1 BR1: Port x Reset bit 1 */
        __IO uint32_t BR2       : 1;       /*!< Bit 2 BR2: Port x Reset bit 2 */
        __IO uint32_t BR3       : 1;       /*!< Bit 3 BR3: Port x Reset bit 3 */
        __IO uint32_t BR4       : 1;       /*!< Bit 4 BR4: Port x Reset bit 4 */
        __IO uint32_t BR5       : 1;       /*!< Bit 5 BR5: Port x Reset bit 5 */
        __IO uint32_t BR6       : 1;       /*!< Bit 6 BR6: Port x Reset bit 6 */
        __IO uint32_t BR7       : 1;       /*!< Bit 7 BR7: Port x Reset bit 7 */
        __IO uint32_t BR8       : 1;       /*!< Bit 8 BR8: Port x Reset bit 8 */
        __IO uint32_t BR9       : 1;       /*!< Bit 9 BR9: Port x Reset bit 9 */
        __IO uint32_t BR10      : 1;      /*!< Bit 10 BR10: Port x Reset bit 10 */
        __IO uint32_t BR11      : 1;      /*!< Bit 11 BR11: Port x Reset bit 11 */
        __IO uint32_t BR12      : 1;      /*!< Bit 12 BR12: Port x Reset bit 12 */
        __IO uint32_t BR13      : 1;      /*!< Bit 13 BR13: Port x Reset bit 13 */
        __IO uint32_t BR14      : 1;      /*!< Bit 14 BR14: Port x Reset bit 14 */
        __IO uint32_t BR15      : 1;      /*!< Bit 15 BR15: Port x Reset bit 15 */
        __IO uint32_t RESERVED  : 16; /*!< Bits 31:16 Reserved, must be kept at reset value */
    } BITS;
} GPIO_BRR_t;
typedef union
{
    __IO uint32_t REG;
    struct
    {
    __IO uint32_t LCK0      : 1;      /*!< Bit 0 LCK0: Port x Lock bit 0 */
    __IO uint32_t LCK1      : 1;      /*!< Bit 1 LCK1: Port x Lock bit 1 */
    __IO uint32_t LCK2      : 1;      /*!< Bit 2 LCK2: Port x Lock bit 2 */
    __IO uint32_t LCK3      : 1;      /*!< Bit 3 LCK3: Port x Lock bit 3 */
    __IO uint32_t LCK4      : 1;      /*!< Bit 4 LCK4: Port x Lock bit 4 */
    __IO uint32_t LCK5      : 1;      /*!< Bit 5 LCK5: Port x Lock bit 5 */
    __IO uint32_t LCK6      : 1;      /*!< Bit 6 LCK6: Port x Lock bit 6 */
    __IO uint32_t LCK7      : 1;      /*!< Bit 7 LCK7: Port x Lock bit 7 */
    __IO uint32_t LCK8      : 1;      /*!< Bit 8 LCK8: Port x Lock bit 8 */
    __IO uint32_t LCK9      : 1;      /*!< Bit 9 LCK9: Port x Lock bit 9 */
    __IO uint32_t LCK10     : 1;     /*!< Bit 10 LCK10: Port x Lock bit 10 */
    __IO uint32_t LCK11     : 1;     /*!< Bit 11 LCK11: Port x Lock bit 11 */
    __IO uint32_t LCK12     : 1;     /*!< Bit 12 LCK12: Port x Lock bit 12 */
    __IO uint32_t LCK13     : 1;     /*!< Bit 13 LCK13: Port x Lock bit 13 */
    __IO uint32_t LCK14     : 1;     /*!< Bit 14 LCK14: Port x Lock bit 14 */
    __IO uint32_t LCK15     : 1;     /*!< Bit 15 LCK15: Port x Lock bit 15 */
    __IO uint32_t LCKK      : 1;      /*!< Bit 16 LCKK: Lock key bit */
    __IO uint32_t RESERVED  : 15; /*!< Bits 31:17 Reserved, must be kept at reset value */
    } BITS;
} GPIO_LCKR_t;

typedef struct
{
    GPIO_CRL_t CRL;   /*!< GPIO port configuration register low,     offset: 0x00 */
    GPIO_CRH_t CRH;   /*!< GPIO port configuration register high,    offset: 0x04 */
    GPIO_IDR_t IDR;   /*!< GPIO port input data register,            offset: 0x08 */
    GPIO_ODR_t ODR;   /*!< GPIO port output data register,           offset: 0x0C */
    GPIO_BSRR_t BSRR; /*!< GPIO port bit set/reset register,         offset: 0x10 */
    GPIO_BRR_t BRR;   /*!< GPIO port bit reset register,             offset: 0x14 */
    GPIO_LCKR_t LCK;  /*!< GPIO port configuration lock register,    offset: 0x18 */
} GPIO_TypeDef;

// NVIC registers
typedef struct
{
	__IO uint32_t ISER[8];  /*!< Interrupt Set Enable Registers,           Address offset: 0x00-0x1C */
	uint32_t      RESERVED0[24];
	__IO uint32_t ICER[8];  /*!< Interrupt Clear Enable Registers,         Address offset: 0x80-0x9C */
	uint32_t      RESERVED1[24];
	__IO uint32_t ISPR[8];  /*!< Interrupt Set Pending Registers,          Address offset: 0x100-0x11C */
	uint32_t      RESERVED2[24];
	__IO uint32_t ICPR[8];  /*!< Interrupt Clear Pending Registers,        Address offset: 0x180-0x19C */
	uint32_t      RESERVED3[24];
	__IO uint32_t IABR[8];  /*!< Interrupt Active bit Registers,           Address offset: 0x200-0x21C */
	uint32_t      RESERVED4[56];                      
	__IO uint8_t  IPR[240]; /*!< Interrupt Priority Registers,             Address offset: 0x300-0x3EF */
	uint32_t      RESERVED5[644];                        
	__O  uint32_t STIR;     /*!< Software Trigger Interrupt Register,      Address offset: 0xE00 */
} NVIC_TypeDef;

// EXTI registers
typedef union
  {
  __IO uint32_t ALL;
  struct
  {
    __IO uint32_t BIT0  : 1;
    __IO uint32_t BIT1  : 1;
    __IO uint32_t BIT2  : 1;
    __IO uint32_t BIT3  : 1;
    __IO uint32_t BIT4  : 1;
    __IO uint32_t BIT5  : 1;
    __IO uint32_t BIT6  : 1;
    __IO uint32_t BIT7  : 1;
    __IO uint32_t BIT8  : 1;
    __IO uint32_t BIT9  : 1;
    __IO uint32_t BIT10 : 1;
    __IO uint32_t BIT11 : 1;
    __IO uint32_t BIT12 : 1;
    __IO uint32_t BIT13 : 1;
    __IO uint32_t BIT14 : 1;
    __IO uint32_t BIT15 : 1;
    __IO uint32_t BIT16 : 1;
    __IO uint32_t BIT17 : 1;
    __IO uint32_t BIT18 : 1;
    __IO uint32_t BIT19 : 1;
    __IO uint32_t BIT20 : 1;
    __IO uint32_t BIT21 : 1;
    __IO uint32_t BIT22 : 1;
    __IO uint32_t BIT23 : 1;
    __IO uint32_t BIT24 : 1;
    __IO uint32_t BIT25 : 1;
    __IO uint32_t BIT26 : 1;
    __IO uint32_t BIT27 : 1;
    __IO uint32_t BIT28 : 1;
    __IO uint32_t BIT29 : 1;
    __IO uint32_t BIT30 : 1;
    __IO uint32_t BIT31 : 1;
  } BITS;
} __32bit;
typedef struct
{
    __32bit IMR;
    __32bit EMR;
    __32bit RTSR;
    __32bit FTSR;
    __32bit SWIER;
    __32bit PR;
} EXTI_TypeDef;

// TIM registers
typedef struct {
   __IO uint32_t CR1;    /*!< TIM control register 1,              Address offset: 0x00 */
   __IO uint32_t CR2;    /*!< TIM control register 2,              Address offset: 0x04 */
   __IO uint32_t SMCR;   /*!< TIM slave mode control register,     Address offset: 0x08 */
   __IO uint32_t DIER;   /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
   __IO uint32_t SR;     /*!< TIM status register,                 Address offset: 0x10 */
   __IO uint32_t EGR;    /*!< TIM event generation register,       Address offset: 0x14 */
   __IO uint32_t CCMR1;  /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
   __IO uint32_t CCMR2;  /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
   __IO uint32_t CCER;   /*!< TIM capture/compare enable register, Address offset: 0x20 */
   __IO uint32_t CNT;    /*!< TIM counter register,                Address offset: 0x24 */
   __IO uint32_t PSC;    /*!< TIM prescaler register,              Address offset: 0x28 */
   __IO uint32_t ARR;    /*!< TIM auto-reload register,            Address offset: 0x2C */
   __IO uint32_t RCR;    /*!< TIM repetition counter register,     Address offset: 0x30 */
   __IO uint32_t CCR1;   /*!< TIM capture/compare register 1,      Address offset: 0x34 */
   __IO uint32_t CCR2;   /*!< TIM capture/compare register 2,      Address offset: 0x38 */
   __IO uint32_t CCR3;   /*!< TIM capture/compare register 3,      Address offset: 0x3C */
   __IO uint32_t CCR4;   /*!< TIM capture/compare register 4,      Address offset: 0x40 */
   __IO uint32_t BDTR;   /*!< TIM break and dead-time register,    Address offset: 0x44 */
   __IO uint32_t DCR;    /*!< TIM DMA control register,            Address offset: 0x48 */
   __IO uint32_t DMAR;   /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
} TIM_TypeDef;

// ADC registers
typedef struct
{
    __IO uint32_t SR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMPR1;
    __IO uint32_t SMPR2;
    __IO uint32_t JOFR1;
    __IO uint32_t JOFR2;
    __IO uint32_t JOFR3;
    __IO uint32_t JOFR4;
    __IO uint32_t HTR;
    __IO uint32_t LTR;
    __IO uint32_t SQR1;
    __IO uint32_t SQR2;
    __IO uint32_t SQR3;
    __IO uint32_t JSQR;
    __IO uint32_t JDR1;
    __IO uint32_t JDR2;
    __IO uint32_t JDR3;
    __IO uint32_t JDR4;
    __IO uint32_t DR;
} ADC_TypeDef;

// UART registers
typedef struct
{
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t BRR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t GTPR;
} USART_TypeDef;

// SPI registers
typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t CRCPR;
    __IO uint32_t RXCRCR;
    __IO uint32_t TXCRCR;
    __IO uint32_t I2SCFGR;
    __IO uint32_t I2SPR;
} SPI_TypeDef;

// I2C registers
typedef struct
{
	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t OAR1;
	__IO uint32_t OAR2;
	__IO uint32_t DR;
	__IO uint32_t SR1;
	__IO uint32_t SR2;
	__IO uint32_t CCR;
	__IO uint32_t TRISE;
} I2C_TypeDef;

// USB registers
typedef struct
{
	__IO uint32_t EP0R;
	__IO uint32_t EP1R;
	__IO uint32_t EP2R;
	__IO uint32_t EP3R;
	__IO uint32_t EP4R;
	__IO uint32_t EP5R;
	__IO uint32_t EP6R;
	__IO uint32_t EP7R;
	__IO uint32_t RESERVED[3];
	__IO uint32_t CNTR;
	__IO uint32_t ISTR;
	__IO uint32_t FNR;
	__IO uint32_t DADDR;
	__IO uint32_t BTABLE;
} USB_TypeDef;

// ===== Peripheral instance =====
#define GPIOA 			((__IO GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB 			((__IO GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC 			((__IO GPIO_TypeDef *)GPIOC_BASE)
#define RCC 			((__IO RCC_TypeDef *)RCC_BASE)
#define FLASH 			((__IO FLASH_TypeDef *)FLASH_BASE)
#define AFIO 			((__IO AFIO_TypeDef *)AFIO_BASE)
#define NVIC 			((__IO NVIC_TypeDef *)NVIC_Base)
#define EXTI 			((__IO EXTI_TypeDef *)EXTI_Base)
#define TIM1 			((__IO TIM_TypeDef *)TIM1_Base)
#define TIM2 			((__IO TIM_TypeDef *)TIM2_Base)
#define TIM3 			((__IO TIM_TypeDef *)TIM3_Base)
#define TIM4 			((__IO TIM_TypeDef *)TIM4_Base)
#define ADC1 	 		((__IO ADC_TypeDef *)ADC1_Base)
#define ADC2 	 		((__IO ADC_TypeDef *)ADC2_Base)
#define USART1          ((__IO USART_TypeDef *)USART1_Base)
#define USART2          ((__IO USART_TypeDef *)USART2_Base)
#define USART3          ((__IO USART_TypeDef *)USART3_Base)
#define SPI1            ((__IO SPI_TypeDef *)SPI1_Base)
#define SPI2            ((__IO SPI_TypeDef *)SPI2_Base)
#define I2C1			((__IO I2C_TypeDef *)I2C1_Base)
#define I2C2			((__IO I2C_TypeDef *)I2C2_Base)
#define USB				((__IO USB_TypeDef *)USB_Base)

#ifdef __cplusplus
}
#endif

#endif /* __STM32SYSTEM_H */

