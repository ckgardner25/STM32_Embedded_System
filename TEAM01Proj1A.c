/****************************************************************
 * TEAM 01: Christopher Gardner, Ryan Atkins
 * CPEG222 TEAM01Proj1A.c, 9/08/25
 * NucleoF466RE CMSIS STM32F4xx Sequence Pmod LEDs
 ****************************************************************/

#include "stm32f4xx.h"
#define LED_PIN 5
#define LED_PORT GPIOA
#define LED_PORT_B GPIOB


int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // rest pins
    LED_PORT->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (4 * 2)));
    LED_PORT->MODER |= ((1 << (0 * 2)) | (1 << (1 * 2)) | (1 << (4 * 2)));

    // set output
    LED_PORT_B->MODER &= ~(3 << (0 * 2));
    LED_PORT_B->MODER |= (1 << (0 * 2));

    while (1)
    {
        // LD3
        LED_PORT->ODR = (1 << 0);
        for (volatile int i = 0; i < 500000; i++)
            ;

        // LD2
        LED_PORT->ODR = (1 << 1);
        for (volatile int i = 0; i < 500000; i++)
            ;

        // LD1
        LED_PORT->ODR = (1 << 4);
        for (volatile int i = 0; i < 500000; i++)
            ;

        // LD0
        LED_PORT->ODR = 0;
        LED_PORT_B->ODR = (1 << 0);
        for (volatile int i = 0; i < 500000; i++)
            ;

        LED_PORT->ODR = 0;
        LED_PORT_B->ODR = 0;
    }

    return 0;
}
