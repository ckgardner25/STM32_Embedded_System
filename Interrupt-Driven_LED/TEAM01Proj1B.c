/****************************************************************
 * TEAM 01: Christopher Gardner, Ryan Atkins
 * CPEG222 TEAM01Proj1B.c, 9/12/25
 * NucleoF466RE CMSIS STM32F4xx Sequence Pmod LEDs
 ****************************************************************/
#include "stm32f4xx.h"

#define LED0 0
#define LED1 1
#define LED2 4
#define LED3 0

volatile int stop = 0;
volatile int direc = 1;
volatile int count = 0;
int leds = 0;

void Main_clock(void)
{
    count = 1;
}

int main(void)
{
    //activate clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIOA->MODER &= ~((3 << (LED0 * 2)) | (3 << (LED1 * 2)) | (3 << (LED2 * 2)));
    GPIOA->MODER |= ((1 << (LED0 * 2)) | (1 << (LED1 * 2)) | (1 << (LED2 * 2)));

    GPIOB->MODER &= ~(3 << (LED3 * 2));
    GPIOB->MODER |= (1 << (LED3 * 2));

    GPIOC->MODER &= ~(3 << (13 * 2));
    GPIOC->PUPDR |= (1 << (13 * 2));

    SysTick_Config(SystemCoreClock / 4);

    int but_oldstate = 1;
    while (1)
    {
    int btn_state = !(GPIOC->IDR & (1 << 13));
    if (btn_state && but_oldstate == 0)
        {
            if (!stop)
            {
                stop = 1;
            }
            else
            {
                stop = 0;
                direc = -direc;
            }
        }
    but_oldstate = btn_state;

        if (count && !stop)
        {
            count = 0;
            GPIOA->ODR &= ~((1 << LED0) | (1 << LED1) | (1 << LED2));
            GPIOB->ODR &= ~(1 << LED3);

            if (leds == 0)
            {
                GPIOA->ODR |= (1 << LED0);
            }
            else if (leds == 1)
            {
                GPIOA->ODR |= (1 << LED1);
            }
            else if (leds == 2)
            {
                GPIOA->ODR |= (1 << LED2);
            }
            else if (leds == 3)
            {
                GPIOB->ODR |= (1 << LED3);
            }
//wraps leds
            leds += direc;
            if (leds > 3)
                leds = 0;
            if (leds < 0)
                leds = 3;
        }
    }
}