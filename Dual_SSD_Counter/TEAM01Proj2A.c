/****************************************************************
 * TEAM 01: Christopher Gardner, Ryan Atkins
 * CPEG222 TEAM01Proj1B.c, 9/26/25
 * NucleoF466RE CMSIS STM32F4xx Sequence Pmod LEDs
 ****************************************************************/

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#define PIN_A 5
#define PIN_B 6
#define PIN_C 7
#define PIN_D 6
#define PIN_E 7
#define PIN_F 9
#define PIN_G 8
#define L_CAT 10
#define R_CAT 10

const unsigned char digitSegments[] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111, // 9
    0b0000000  // no display
};

volatile uint8_t left_num = 0;
volatile uint8_t right_num = 0;
volatile bool cat = 0;
volatile uint8_t count = 0;

void GPIO_Init(void);
void TIM2_Init(void);
void SSD_Write(uint8_t digit);

void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    GPIOA->MODER &= ~((3 << (PIN_A * 2)) | (3 << (PIN_B * 2)) | (3 << (PIN_C * 2)) | (3 << (PIN_F * 2)) | (3 << (PIN_G * 2)));

    // output setup
    GPIOA->MODER |= ((1 << (PIN_A * 2)) | (1 << (PIN_B * 2)) | (1 << (PIN_C * 2)) | (1 << (PIN_F * 2)) | (1 << (PIN_G * 2)));

    GPIOB->MODER &= ~((3 << (PIN_D * 2)) | (3 << (L_CAT * 2)) | (3 << (R_CAT * 2)));
    GPIOB->MODER |= ((1 << (PIN_D * 2)) | (1 << (L_CAT * 2)) | (1 << (R_CAT * 2)));
    GPIOC->MODER &= ~(3 << (PIN_E * 2));
    GPIOC->MODER |= (1 << (PIN_E * 2));
}

void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 15;
    TIM2->ARR = 500 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
}

void SSD_Write(uint8_t digit)
{
    if (digit >= 16)
    {
        GPIOA->ODR &= ~((1 << PIN_A) | (1 << PIN_B) | (1 << PIN_C) | (1 << PIN_F) | (1 << PIN_G));
        GPIOB->ODR &= ~(1 << PIN_D);
        GPIOC->ODR &= ~(1 << PIN_E);
        return;
    }

    uint8_t seg = digitSegments[digit & 0x0F];

    if (seg & 0x01)
        GPIOA->BSRR = (1 << PIN_A);
    else
        GPIOA->BSRR = (1 << (PIN_A + 16));
    if (seg & 0x02)
        GPIOA->BSRR = (1 << PIN_B);
    else
        GPIOA->BSRR = (1 << (PIN_B + 16));
    if (seg & 0x04)
        GPIOA->BSRR = (1 << PIN_C);
    else
        GPIOA->BSRR = (1 << (PIN_C + 16));
    if (seg & 0x08)
        GPIOB->BSRR = (1 << PIN_D);
    else
        GPIOB->BSRR = (1 << (PIN_D + 16));
    if (seg & 0x10)
        GPIOC->BSRR = (1 << PIN_E);
    else
        GPIOC->BSRR = (1 << (PIN_E + 16));
    if (seg & 0x20)
        GPIOA->BSRR = (1 << PIN_F);
    else
        GPIOA->BSRR = (1 << (PIN_F + 16));
    if (seg & 0x40)
        GPIOA->BSRR = (1 << PIN_G);
    else
        GPIOA->BSRR = (1 << (PIN_G + 16));
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        
        GPIOA->BSRR = (1 << (PIN_A + 16)) | (1 << (PIN_B + 16)) | (1 << (PIN_C + 16)) | (1 << (PIN_F + 16)) | (1 << (PIN_G + 16));
        GPIOB->BSRR = (1 << (PIN_D + 16));
        GPIOC->BSRR = (1 << (PIN_E + 16));

        if (cat)
        {
            GPIOB->ODR |= (1 << L_CAT);
            GPIOB->ODR &= ~(1 << R_CAT);
            SSD_Write(left_num);
        }
        else
        {
            GPIOB->ODR &= ~(1 << L_CAT);
            GPIOB->ODR |= (1 << R_CAT);
            SSD_Write(right_num);
        }
        cat = !cat;
    }
}

void SysTick_Handler(void)
{
    count++;
    if (count > 99)
        count = 0; // resets clock at 99

    if (count < 10)
    {
        right_num = 16;
        left_num = count;
    }
    else
    {
        right_num = count / 10;
        left_num = count % 10;
    }
}

int main(void)
{
    GPIO_Init();
    TIM2_Init();
    SysTick_Config(SystemCoreClock);
    while (1) {
        __WFI();
    }
}