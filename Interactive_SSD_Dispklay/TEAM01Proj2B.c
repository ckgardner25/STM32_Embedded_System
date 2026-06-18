/****************************************************************
 * TEAM 01: Christopher Gardner, Ryan Atkins
 * CPEG222 TEAM01Proj2B.c, 9/26/25
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
    0b0000000  // blank
};

volatile uint8_t left_num = 0;
volatile uint8_t right_num = 0;
volatile uint8_t count = 0;
volatile bool cat = false;
volatile bool paused = false;

volatile uint32_t ms_ticks = 0;
volatile bool double_press_window = false;

//prototypes
void GPIO_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void SSD_Write(uint8_t digit);
void Button_Init(void);

void GPIO_Init(void)
{
    //GPIO setup for SSD
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    GPIOA->MODER |= (1 << (PIN_A * 2)) | (1 << (PIN_B * 2)) | (1 << (PIN_C * 2)) | (1 << (PIN_F * 2)) | (1 << (PIN_G * 2));
    GPIOB->MODER |= (1 << (PIN_D * 2)) | (1 << (L_CAT * 2)) | (1 << (R_CAT * 2));
    GPIOC->MODER |= (1 << (PIN_E * 2));
}

void TIM2_Init(void)
{
    //used for swapping between digits (multiplexing)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15;
    TIM2->ARR = 500 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);

    //only used for display purposes
}

void TIM3_Init(void)
{
    //used for double press which resets the counter
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 16000 - 1;
    TIM3->ARR = 1000;
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 2);
    //no watchdog used, only gen purpose timer (though TIM3 acts as guard)
}

void SSD_Write(uint8_t digit)
{
    //sets the pin values and resets them by referring to the loopup table
    uint8_t seg = digitSegments[digit];
    (seg & 0x01) ? (GPIOA->BSRR = (1 << PIN_A)) : (GPIOA->BSRR = (1 << (PIN_A + 16)));
    (seg & 0x02) ? (GPIOA->BSRR = (1 << PIN_B)) : (GPIOA->BSRR = (1 << (PIN_B + 16)));
    (seg & 0x04) ? (GPIOA->BSRR = (1 << PIN_C)) : (GPIOA->BSRR = (1 << (PIN_C + 16)));
    (seg & 0x08) ? (GPIOB->BSRR = (1 << PIN_D)) : (GPIOB->BSRR = (1 << (PIN_D + 16)));
    (seg & 0x10) ? (GPIOC->BSRR = (1 << PIN_E)) : (GPIOC->BSRR = (1 << (PIN_E + 16)));
    (seg & 0x20) ? (GPIOA->BSRR = (1 << PIN_F)) : (GPIOA->BSRR = (1 << (PIN_F + 16)));
    (seg & 0x40) ? (GPIOA->BSRR = (1 << PIN_G)) : (GPIOA->BSRR = (1 << (PIN_G + 16)));
}

void TIM2_IRQHandler(void)
{
    //this interrupt handles the actual multiplexing of the SSD
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        GPIOA->ODR &= ~((1 << PIN_A) | (1 << PIN_B) | (1 << PIN_C) | (1 << PIN_F) | (1 << PIN_G));
        GPIOB->ODR &= ~(1 << PIN_D);
        GPIOC->ODR &= ~(1 << PIN_E);

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

void TIM3_IRQHandler(void)
{
    //handles timing for double press
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        double_press_window = false;
    }
}

void SysTick_Handler(void)
{
    //used for time keeping as a counter every 1ms
    ms_ticks++;

    static uint16_t sec_div = 0;
    if (!paused)
    {
        sec_div++;
        if (sec_div >= 1000)
        {
            sec_div = 0;
            count++;
            if (count > 99)
                count = 0;

            if (count < 10)
            {
                right_num = 10;
                left_num = count;
            }
            else
            {
                right_num = count / 10;
                left_num = count % 10;
            }
        }
    }
}

void Button_Init(void)
{
    //sets up the button and links it to the EXTI interrupt
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3 << (13 * 2));

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    EXTI->IMR |= EXTI_IMR_IM13;
    EXTI->FTSR |= EXTI_FTSR_TR13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
    //interrupt for button presses checks for high and low signals
    //this interrupt is set up to avoid bouncing signals with the TIM3 as a guard
    if (EXTI->PR & EXTI_PR_PR13)
    {
        EXTI->PR = EXTI_PR_PR13;

        if (double_press_window)
        {
            //double press clearing the display
            count = 0; //count reset
            right_num = 10; //tens digit blank
            left_num = 0; //ones digit 0
            paused = false;
            TIM3->CR1 &= ~TIM_CR1_CEN;
            double_press_window = false;
        }
        else
        {
            // First press and pauses the counter
            paused = !paused;
            double_press_window = true;
            TIM3->CNT = 0;
            TIM3->CR1 |= TIM_CR1_CEN;
        }
    }
}

int main(void)
{
    GPIO_Init();
    TIM2_Init();
    TIM3_Init();
    Button_Init();

    SysTick_Config(SystemCoreClock / 1000);


    while (1)
    {
        __WFI(); //simply waits for a interrupt
    }
}
