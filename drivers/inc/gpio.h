#ifndef GPIO_H
#define GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32system.h"

typedef enum
{
    GPIO_MODE_INPUT = 0x00, /*!< Input floating mode */
    GPIO_MODE_OUTPUT_PP,    /*!< General purpose output push-pull */
    GPIO_MODE_OUTPUT_OD,    /*!< General purpose output open-drain */
    GPIO_MODE_AF_PP,        /*!< Alternate function output push-pull */
    GPIO_MODE_AF_OD,        /*!< Alternate function output open-drain */
    GPIO_MODE_ANALOG,       /*!< Analog mode */
    GPIO_MODE_INPUT_PU,     /*!< Input with pull-up */
    GPIO_MODE_INPUT_PD      /*!< Input with pull-down */
} GPIO_Mode_t;

typedef enum
{
    GPIO_SPEED_NONE = 0x00,  /*!< No speed specified (input mode) */
    GPIO_SPEED_10MHZ = 0x01, /*!< Low speed: max 10 MHz output */
    GPIO_SPEED_2MHZ = 0x02,  /*!< Lowest speed: max 2 MHz output */
    GPIO_SPEED_50MHZ = 0x03  /*!< High speed: max 50 MHz output */
} GPIO_Speed_t;


typedef struct
{
    uint16_t Pin;       /*!< Specifies the GPIO pins to be configured.
                            This parameter can be any value of @ref GPIO_pins_define */
    GPIO_Mode_t Mode;   /*!< Specifies the operating mode for the selected pins.
                                 This parameter can be a value of @ref GPIO_Mode_t */
    GPIO_Speed_t Speed; /*!< Specifies the speed for the selected pins.
                            This parameter can be a value of @ref GPIO_Speed_t */
} GPIO_InitTypeDef;

void GPIO_Init(__IO GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);
void GPIO_WritePin(__IO GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state);
void GPIO_TogglePin(__IO GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t GPIO_ReadPin(__IO GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_H */

