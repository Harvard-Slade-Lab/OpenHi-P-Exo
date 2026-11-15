/* Host-test stand-in for the STM32 HAL header: only the GPIO declarations that
 * Core/Src/control_logic.c uses (update_control_mode). A test that compiles control_logic.c
 * defines stubGPIOA, stubGPIOC, HAL_GPIO_ReadPin and HAL_GPIO_WritePin. */
#ifndef TESTS_STUB_STM32F4XX_HAL_H_
#define TESTS_STUB_STM32F4XX_HAL_H_

#include <stdint.h>

typedef struct
{
    uint32_t unused;
} GPIO_TypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

extern GPIO_TypeDef stubGPIOA, stubGPIOC;
#define GPIOA (&stubGPIOA)
#define GPIOC (&stubGPIOC)
#define GPIO_PIN_8 ((uint16_t)0x0100)
#define GPIO_PIN_9 ((uint16_t)0x0200)

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

#endif /* TESTS_STUB_STM32F4XX_HAL_H_ */
