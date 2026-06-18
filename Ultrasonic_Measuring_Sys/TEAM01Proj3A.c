/****************************************************************
 * TEAM 01: Christopher Gardner, Ryan Atkins
 * CPEG222 TEAM01Proj3A.c, 10/14/25
 * Nucleo-F446RE | UD CPEG222 F25 Shield | HC-SR04 Ultrasonic
 * Fully interrupt-driven: SysTick + EXTI + TIM2 + TIM5
 ****************************************************************/

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>
#include <stdbool.h>

#define TRIG_PIN   4      //PA4
#define ECHO_PIN   0      // PB0
#define BTN_PIN   13      // PC13 (button)
#define USART_PIN   2      // PA2

volatile uint32_t start_tim = 0;
volatile uint32_t pulse_width = 0;
volatile bool echo_high = false;
volatile bool useInches = false;
volatile bool newReadingReady = false;

volatile float distance = 0.0f;
volatile int ssd_val = 0;
volatile int digit_sel = 0;

void GPIO_Init(void);
void TIM5_Init(void);
void TIM2_Init(void);
void USART2_Init(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void TIM2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void USART2_SendChar(char c);
void USART2_SendString(char *s);

void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN; //enable clocks A-C

    //Trig pin
    GPIOA->MODER &= ~(3 << (TRIG_PIN * 2)); //3 = 00000000 00000011, shift left 8 = 00000011 00000000 and invert to get 00
    GPIOA->MODER |=  (1 << (TRIG_PIN * 2)); //1 = 00000000 00000001, shift left 8 = 00000001 00000000 to set to 01 (output)
    GPIOA->ODR &= ~(1 << TRIG_PIN); //4 = 0000 0000 0001 0000, inverted 1111 1111 1110 1111 mask = 0 

    // ECHO
    GPIOB->MODER &= ~(3 << (ECHO_PIN * 2)); //3 = 00000000 00000011, shift left 0 = 00000000 00000011 and invert to get 00
    GPIOB->PUPDR &= ~(3 << (ECHO_PIN * 2)); //3 = 00000000 00000011, shift left 0 = 00000000 00000011 and invert to get 00 no PUPD
    GPIOB->PUPDR |=  (2 << (ECHO_PIN * 2)); //2 = 00000000 00000010, shift left 0 = 00000000 00000010 to set to 10 so PD

    // Button PC13 input
    GPIOC->MODER &= ~(3 << (BTN_PIN * 2)); // 3=00000011, shift left 26 = 11000000 00000000 00000000 00000000 invert, clears to 00

    //EXTI0 rising and fal edge
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //turn on SYSCFG clock
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; //invert and clear bit 3:0
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PB; //set bit 3:0 to 0001 
    EXTI->IMR  |= EXTI_IMR_IM0; //0=00000001 sets interrupt
    EXTI->RTSR |= EXTI_RTSR_TR0; //0=00000001 sets rising-edge
    EXTI->FTSR |= EXTI_FTSR_TR0; //0=00000001 sets falling-edge
    NVIC_EnableIRQ(EXTI0_IRQn); //enable Interrupt request
    NVIC_SetPriority(EXTI0_IRQn, 1); //priority 1

    //EXTI13 falling edge
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13; //invert and clear 11:8
    SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC; //set 9:8=10
    EXTI->IMR  |= EXTI_IMR_IM13; //13= shift 1 enable mask
    EXTI->FTSR |= EXTI_FTSR_TR13; //set fall edge
    NVIC_EnableIRQ(EXTI15_10_IRQn); //share IRQ 
    NVIC_SetPriority(EXTI15_10_IRQn, 2); //priority 2
}

void TIM5_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //TIM5 clock enable
    TIM5->PSC = 1;           // 16 MHz / (15 + 1) = 1 MHz to 1 µs
    TIM5->ARR = 0xFFFFFFFF;  //free running counter
    TIM5->CR1 |= TIM_CR1_CEN; //start counter
}

void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable TIM2 clock
    TIM2->PSC = 8000 - 1;     // 16 MHz / 8000 = 2 kHz to 0.5 ms
    TIM2->ARR = 1; // overflow every 1 ms
    TIM2->DIER |= TIM_DIER_UIE; //interrupt on overflow
    NVIC_EnableIRQ(TIM2_IRQn); //NVIC for TIM2
    NVIC_SetPriority(TIM2_IRQn, 3); //pritority 3
    TIM2->CR1 |= TIM_CR1_CEN; //start TIM2
}

void USART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //enable USART2 clock
    GPIOA->MODER &= ~(3 << (USART_PIN * 2)); // 3 = 00000000 00000011, shift left 4 = 00000000 00001100 and invert to get 00
    GPIOA->MODER |=  (2 << (USART_PIN * 2));      // 2 = 00000000 00000010, shift left 4 = 00000000 00001000 to set to 10 (AF)
    GPIOA->AFR[0] &= ~(0xF << (USART_PIN * 4)); // 0xF = 0000 0000 0000 1111, shift left 8 = 0000 1111 0000 0000 and invert to get 1111 0000 1111 1111
    GPIOA->AFR[0] |=  (7 << (USART_PIN * 4));  // 7 = 0000 0000 0000 0111, shift left 8 = 0000 0111 0000 0000 to set to AF7 (USART2)
    USART2->BRR = (16000000 / 115200); // 16 MHz / 115200 baud
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE; //enable TX and USART
}

void USART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE)); // wait till bit 7  =1 for 10000000
    USART2->DR = c; //transmit ASCII
}

void USART2_SendString(char *s)
{
    while (*s) USART2_SendChar(*s++); //send string until null
}

void SysTick_Handler(void)
{
    if (echo_high) return;         //skip if echo still high
    uint32_t t0 = TIM5->CNT; //read 32 bit counter
    GPIOA->ODR |= (1 << TRIG_PIN); // 1<<4 = 00010000 sets bit4 high (TRIG=1)
    while ((TIM5->CNT - t0) < 10);  // 10 µs high wait
    GPIOA->ODR &= ~(1 << TRIG_PIN);  // 1<<4 = 00010000 invert, clears bit4 low (TRIG=0)
}

void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0) //check bit0 =1
    {
        EXTI->PR = EXTI_PR_PR0; //00000001 writes 1 to clear pending bit0

        if (!echo_high) // rising edge detected
        {
            echo_high = true; 
            start_tim = TIM5->CNT; // record start timestamp
        }
        else
        {
            echo_high = false;
            uint32_t end_time = TIM5->CNT;
            pulse_width = (end_time >= start_tim)
                        ? (end_time - start_tim) 
                        : ((0xFFFFFFFF - start_tim) + end_time + 1); 

            // Convert µs distance
            distance = useInches
                     ? pulse_width * 0.000675f //in
                     : pulse_width * 0.001715f; //cm

            if (distance > 99.99f) distance = 99.99f; //stop at 99.99
            ssd_val = (int)(distance * 100); //scale for SSD
            newReadingReady = true; //flag new reading
        }
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) //00000001 check update flag
    {
        TIM2->SR &= ~TIM_SR_UIF; //00000001 invert, clears UIF bit0

        // Blank first digit if < 10.00
        int display_val = ssd_val;
        if (distance < 10.0f) display_val %= 1000; //blank leading digit if <10.00

        SSD_update(digit_sel, display_val, 2); //update ssd
        digit_sel = (digit_sel + 1) % 4; //next digit

        if (digit_sel == 0 && newReadingReady) //new data per cycle
        {
            char msg[40];
            sprintf(msg, "%.2f %s\r\n",
                    distance, useInches ? "inch" : "cm");
            USART2_SendString(msg);
            newReadingReady = false;
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR13) //check bit13 =1
    {
        EXTI->PR = EXTI_PR_PR13; //0010 0000 0000 0000 writes 1 to clear bit13
        useInches = !useInches;  // toggle display units
    }
}

int main(void)
{
    GPIO_Init();
    TIM5_Init();
    TIM2_Init();
    USART2_Init();
    SSD_init();

    // Power-up delay
    for (volatile int i = 0; i < 1000000; i++);

    // 16 MHz × 0.5 = 0.5ms
    SysTick_Config(8000000);

    while (1)
        __WFI();  
}
