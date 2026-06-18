/****************************************************************
 * TEAM 01: Christopher Gardner, Ryan Atkins
 * CPEG222 TEAM01Proj3B.c, 10/22/25
 * Nucleo-F446RE  UD CPEG222 F25 Shield HC-SR04
 * Full Ultrasonic Ranging with Servo Scanning
 * Measures distance using HC-SR04 ultrasonic sensor mounted on a servo
 ****************************************************************/

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>
#include <stdbool.h>

#define TRIG_PIN 4 // PA4 trigger pin
#define ECHO_PIN 0 // PB0 echo pin
#define BTN_PIN 13 // PC13 user button
#define USART_TX 2 // PA2 UART TX pin
#define FREQUENCY 16000000UL // 16 MHz clock
#define SERVO3_PIN 6 // PC6 servo PWM output
#define SERVO3_PORT GPIOC // servo port

volatile uint32_t start_tim = 0;
volatile uint32_t pulse_width = 0;
volatile bool echo_high = false;
volatile bool useInches = false;
volatile bool newReadingReady = false;
volatile float distance = 0.0f;
volatile int ssd_val = 0;
volatile int digit_sel = 0;
volatile bool triggerReady = false;

void GPIO_Init(void);
void TIM5_Init(void);
void TIM2_Init(void);
void TIM3_PWM_Init(void);
void USART2_Init(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void TIM2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void USART2_SendChar(char c);
void USART2_SendString(char *s);
void servo_angle_set(int angle);

void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; // enable GPIOA/B/C clocks

    // TRIG (PA4) → output
    GPIOA->MODER &= ~(3 << (TRIG_PIN * 2)); // 3=00000011, shift left 8=0000110000, invert so 1111001111 clears bits to 00
    GPIOA->MODER |= (1 << (TRIG_PIN * 2));  // 1=00000001, shift left 8=0000010000 sets bits to 01 (output)
    GPIOA->ODR &= ~(1 << TRIG_PIN); // 4=00010000, invert→11101111 forces output low

    // ECHO (PB0) → input pulldown
    GPIOB->MODER &= ~(3 << (ECHO_PIN * 2)); // 3=00000011, shift left 0=00000011, invert so 11111100 clears bits to 00
    GPIOB->PUPDR &= ~(3 << (ECHO_PIN * 2)); // clear previous PUPD config
    GPIOB->PUPDR |= (2 << (ECHO_PIN * 2));  // 2=00000010 sets bits to 10 (pulldown)

    // BUTTON (PC13) → input
    GPIOC->MODER &= ~(3 << (BTN_PIN * 2)); // 3=00000011, shift left 26, invert clears bits to 00

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable SYSCFG for EXTI

    // EXTI0 from PB0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // clear EXTI0
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB; // map PB0 to EXTI0
    EXTI->IMR |= EXTI_IMR_IM0; // unmask line 0
    EXTI->RTSR |= EXTI_RTSR_TR0; // rising edge enable
    EXTI->FTSR |= EXTI_FTSR_TR0; // falling edge enable
    NVIC_EnableIRQ(EXTI0_IRQn); // enable NVIC
    NVIC_SetPriority(EXTI0_IRQn, 1);

    // EXTI13 from PC13
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13; // clear EXTI13
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // map PC13 to EXTI13
    EXTI->IMR |= EXTI_IMR_IM13; // unmask line 13
    EXTI->FTSR |= EXTI_FTSR_TR13; // falling edge enable
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 2);
}

void TIM5_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // enable TIM5 clock
    TIM5->PSC = 1; // prescale to 1 MHz
    TIM5->ARR = 0xFFFFFFFF; // full 32-bit counter
    TIM5->CR1 |= TIM_CR1_CEN; // enable timer
}

void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable TIM2
    TIM2->PSC = 8000 - 1; // prescale 8000
    TIM2->ARR = 1; // auto reload
    TIM2->DIER |= TIM_DIER_UIE; // update interrupt enable
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 3);
    TIM2->CR1 |= TIM_CR1_CEN; // start
}

void TIM3_PWM_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // enable port C
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3

    GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2)); // clear mode bits
    GPIOC->MODER |= (0x2 << (SERVO3_PIN * 2)); // set alt function (10)
    GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4)); // clear AFR
    GPIOC->AFR[0] |= (0x2 << (SERVO3_PIN * 4)); // AF2 for TIM3_CH1

    TIM3->PSC = (FREQUENCY / 1000000) - 1; // 1 us tick
    TIM3->ARR = 19999; // 20 ms period
    TIM3->CCR1 = 1500; // neutral (1.5 ms)
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M); // clear mode bits
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // enable preload
    TIM3->CCER |= TIM_CCER_CC1E; // enable output
    TIM3->CR1 |= TIM_CR1_ARPE; // auto reload buffer
    TIM3->EGR = TIM_EGR_UG; // force update
    TIM3->CR1 |= TIM_CR1_CEN; // start timer
}

void USART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable USART2
    GPIOA->MODER &= ~(3 << (USART_TX * 2)); // clear mode bits
    GPIOA->MODER |= (2 << (USART_TX * 2)); // set alt function
    GPIOA->AFR[0] &= ~(0xF << (USART_TX * 4)); // clear AFR
    GPIOA->AFR[0] |= (7 << (USART_TX * 4)); // AF7 for USART2_TX
    USART2->BRR = (16000000 / 115200); // baud rate =115200
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE; // enable TX + USART
}

void USART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE))
        ;           // wait TX ready
    USART2->DR = c; // send char
}

void USART2_SendString(char *s)
{
    while (*s)
        USART2_SendChar(*s++); // send each byte
}

void SysTick_Handler(void)
{
    if (echo_high)
        return; // ignore if echo high
    uint32_t t0 = TIM5->CNT; // capture start
    GPIOA->ODR |= (1 << TRIG_PIN); // set TRIG high
    while ((TIM5->CNT - t0) < 10)
        ; // 10 us delay
    GPIOA->ODR &= ~(1 << TRIG_PIN); // set TRIG low
    triggerReady = true;
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR = EXTI_PR_PR0; // clear pending
        if (!echo_high)
        {
            echo_high = true; // rising edge
            start_tim = TIM5->CNT; // start count
        }
        else
        {
            echo_high = false; // falling edge
            uint32_t end_time = TIM5->CNT; // stop count
            pulse_width = (end_time >= start_tim)
                              ? (end_time - start_tim)
                              : ((0xFFFFFFFF - start_tim) + end_time + 1);

            distance = useInches ? pulse_width * 0.000675f : pulse_width * 0.001715f;
            if (distance > 99.99f)
                ssd_val = 9999;
            else
                ssd_val = (int)(distance * 100);
            newReadingReady = true;
        }
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF; // clear flag
        int display_val = ssd_val;
        if (distance < 10.0f)
            display_val %= 1000;
        SSD_update(digit_sel, display_val, 2);
        digit_sel = (digit_sel + 1) % 4; // next digit
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR13)
    {
        EXTI->PR = EXTI_PR_PR13; // clear flag
        useInches = !useInches;  // toggle unit mode
    }
}

void servo_angle_set(int angle)
{
    uint32_t pw = 1500 + (500 * (angle / 45.0)); // map +/-45°, +/-0.5 ms offset
    TIM3->CCR1 = pw;
}

int main(void)
{
    GPIO_Init();
    TIM5_Init();
    TIM2_Init();
    TIM3_PWM_Init();
    USART2_Init();
    SSD_init();

    for (volatile int i = 0; i < 1000000; i++)
        ;  // startup delay
    SysTick_Config(8000000); // 0.5 Hz trigger

    while (1)
    {
        for (int angle = -45; angle <= 45; angle += 5)
        {
            servo_angle_set(angle);
            for (volatile int i = 0; i < 2000000; i++)
                ;
            if (newReadingReady)
            {
                char msg[60];
                sprintf(msg, "angle(deg): %d, pulsewidth(us): %lu, range: %.2f %s\r\n",
                        angle, pulse_width, distance, useInches ? "inch" : "cm");
                USART2_SendString(msg);
                newReadingReady = false;
            }
        }
        for (int angle = 45; angle >= -45; angle -= 5)
        {
            servo_angle_set(angle);
            for (volatile int i = 0; i < 2000000; i++)
                ;
            if (newReadingReady)
            {
                char msg[60];
                sprintf(msg, "angle(deg): %d, pulsewidth(us): %lu, range: %.2f %s\r\n",
                        angle, pulse_width, distance, useInches ? "inch" : "cm");
                USART2_SendString(msg);
                newReadingReady = false;
            }
        }
    }
}
