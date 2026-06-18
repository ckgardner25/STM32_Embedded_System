/****************************************************************
 * TEAM 01: Christopher Gardner, Ryan Atkins
 * CPEG222 TEAM01Proj4A.c  |  11/06/25
 * Nucleo-F446RE | UD CPEG222 F25 Shield | Parallax Feedback 360 Servo
 ****************************************************************/

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* --- System constants --- */
#define FREQ         16000000UL
#define SERVO_PIN    6
#define FEEDBACK_PIN 7
#define BTN_PIN      13
#define ADC_PIN      0
#define SERVO_STOP   1500
#define SERVO_MIN    1280
#define SERVO_MAX    1720

/* --- Global variables --- */
volatile uint32_t rising=0, falling=0, period=0, high=0;
volatile float duty=0.0f, angle=0.0f, prev_angle=0.0f;
volatile float rpm=0.0f, rpm_filt=0.0f;
volatile int adc_val=0, pwm_us=SERVO_STOP, dir_state=0;
volatile int ssd_val=0, digit=0;

/****************************************************************
 * Function Prototypes
 ****************************************************************/
void GPIO_Init(void);
void ADC1_Init(void);
uint16_t ADC1_Read(void);
void TIM3_PWM_IC_Init(void);
void TIM2_Init(void);
void USART2_Init(void);
void USART2_SendString(char *s);
void SysTick_Handler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

/****************************************************************
 * GPIO_Init
 * Configures the user button on PC13 with external interrupt
 * The button cycles through CW -> STOP -> CCW -> STOP states
 ****************************************************************/
void GPIO_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PC13 set as input mode
    GPIOC->MODER &= ~(3 << (BTN_PIN*2));

    // Connect PC13 to EXTI13 line
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC;

    // Enable falling edge interrupt on PC13
    EXTI->IMR  |= EXTI_IMR_IM13;
    EXTI->FTSR |= EXTI_FTSR_TR13;

    // NVIC configuration
    NVIC_SetPriority(EXTI15_10_IRQn,2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/****************************************************************
 * ADC1_Init / ADC1_Read
 * Configures ADC1 channel 0 on PA0 for analog input
 * Reads potentiometer voltage (0V = 0, 3.3V = 4095)
 ****************************************************************/
void ADC1_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // PA0 in analog mode
    GPIOA->MODER |= (3 << (ADC_PIN*2));

    ADC1->SQR3 = 0;           // Channel 0
    ADC1->SMPR2 = (7 << 0);   // Max sample time
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC1_Read(void){
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC));
    return (ADC1->DR & 0xFFF);
}

/****************************************************************
 * TIM3_PWM_IC_Init
 * Configures TIM3 for both PWM output and input capture
 * CH1 (PC6)  generates servo control pulses
 * CH2 (PC7)  reads servo feedback duty cycle to determine angle
 ****************************************************************/
void TIM3_PWM_IC_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // PC6  alternate function mode  TIM3 CH1
    GPIOC->MODER &= ~(3 << (SERVO_PIN*2));
    GPIOC->MODER |=  (2 << (SERVO_PIN*2));
    GPIOC->AFR[0] &= ~(0xF << (SERVO_PIN*4));
    GPIOC->AFR[0] |=  (0x2 << (SERVO_PIN*4));

    // PC7  alternate function mode  TIM3 CH2
    GPIOC->MODER &= ~(3 << (FEEDBACK_PIN*2));
    GPIOC->MODER |=  (2 << (FEEDBACK_PIN*2));
    GPIOC->AFR[0] &= ~(0xF << (FEEDBACK_PIN*4));
    GPIOC->AFR[0] |=  (0x2 << (FEEDBACK_PIN*4));

    // Timer base setup 1 MHz clock
    TIM3->PSC = 15;                 
    TIM3->ARR = 0xFFFF;

    // Channel 1 PWM output
    TIM3->CCR1  = SERVO_STOP;
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |=  (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM3->CCMR1 |=  TIM_CCMR1_OC1PE;
    TIM3->CCER  |=  TIM_CCER_CC1E;

    // Channel 2 input capture (feedback)
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S);
    TIM3->CCMR1 |=  TIM_CCMR1_CC2S_0;  // map TI2
    TIM3->CCER  &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);
    TIM3->CCER  |=  TIM_CCER_CC2E;
    TIM3->DIER  |=  TIM_DIER_CC2IE;

    NVIC_SetPriority(TIM3_IRQn,1);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

/****************************************************************
 * TIM2_Init
 * Configures TIM2 to multiplex the SSD display
 * Each interrupt occurs every 1.5 ms (about 67 Hz per digit)
 ****************************************************************/
void TIM2_Init(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 159;      // 16 MHz / 160 = 100 kHz
    TIM2->ARR = 150;      // 1.5 ms interval
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn,3);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
}

/****************************************************************
 * USART2_Init / USART2_SendString
 * Sets up USART2 TX on PA2 for serial output at 115200 baud
 * Used to print ADC, direction, PWM, and RPM to serial monitor
 ****************************************************************/
void USART2_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2 alternate function  AF7
    GPIOA->MODER &= ~(3 << (2*2));
    GPIOA->MODER |=  (2 << (2*2));
    GPIOA->AFR[0] &= ~(0xF << (2*4));
    GPIOA->AFR[0] |=  (7 << (2*4));

    USART2->BRR = FREQ / 115200;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void USART2_SendString(char *s){
    while(*s){
        while(!(USART2->SR & USART_SR_TXE));
        USART2->DR = *s++;
    }
}

/****************************************************************
 * SysTick_Handler
 * Runs every 2 ms
 * Calculates RPM from servo feedback and prints data every 1 s
 * Also updates SSD display value every 100 ms
 ****************************************************************/
void SysTick_Handler(void){
    static uint16_t ms_counter = 0;
    static uint8_t display_div = 0;

    float diff = angle - prev_angle;
    if(diff > 180.0f) diff -= 360.0f;
    else if(diff < -180.0f) diff += 360.0f;
    prev_angle = angle;

    // RPM calculation based on angle change over 2 ms
    rpm = (fabsf(diff) / 360.0f) * (60.0f / 0.002f);
    rpm_filt = 0.8f * rpm_filt + 0.2f * rpm;

    // Update SSD every 100 ms
    display_div++;
    if (display_div >= 50) {   // 50 * 2ms = 100ms
        display_div = 0;
        if (rpm_filt < 5.0f) {
            ssd_val = 0;
        } else {
            ssd_val = (int)(rpm_filt * 10.0f);   // one decimal place
            if (ssd_val > 9999) ssd_val = 9999;
        }
    }

    // Serial print once per second
    ms_counter += 2;
    if(ms_counter >= 1000){
        ms_counter = 0;
        char msg[100];
        sprintf(msg,"ADC:%d dir:%s servo:%d rpm:%.1f\r\n",
            adc_val,(dir_state==0?"CW":(dir_state==2?"CCW":"STOP")),
            pwm_us,rpm_filt);
        USART2_SendString(msg);
    }
}

/****************************************************************
 * TIM2_IRQHandler
 * SSD display refresh interrupt
 * Calls SSD_update() for next digit every 1.5 ms
 ****************************************************************/
void TIM2_IRQHandler(void){
    if(TIM2->SR & TIM_SR_UIF){
        TIM2->SR &= ~TIM_SR_UIF;
        SSD_update(digit, ssd_val, 3);   // decimal after 3rd digit
        digit = (digit+1)%4;
    }
}

/****************************************************************
 * TIM3_IRQHandler
 * Handles servo feedback signal on PC7 (CH2 input capture)
 * Measures pulse width and computes angle in degrees
 ****************************************************************/
void TIM3_IRQHandler(void){
    static uint32_t last_rise=0;
    static uint8_t falling_edge=0;
    static float duty_filt=0;

    if(TIM3->SR & TIM_SR_CC2IF){
        uint32_t now = TIM3->CCR2;

        if(!falling_edge){
            period = (now>=last_rise)?(now-last_rise):(0xFFFF-last_rise+now);
            last_rise = now;
            TIM3->CCER |= TIM_CCER_CC2P;     // switch to falling
            falling_edge = 1;
        } else {
            high = (now>=last_rise)?(now-last_rise):(0xFFFF-last_rise+now);
            TIM3->CCER &= ~TIM_CCER_CC2P;    // switch to rising
            falling_edge = 0;

            if(period>0){
                float raw = ((float)high/(float)period)*100.0f;
                if(raw<2.9f) raw=2.9f;
                if(raw>97.1f) raw=97.1f;
                duty_filt = 0.9f*duty_filt + 0.1f*raw;
                duty = duty_filt;
                angle = 360.0f - ((duty-2.9f)*360.0f)/(97.1f-2.9f);
            }
        }
        TIM3->SR &= ~TIM_SR_CC2IF;
    }
}

/****************************************************************
 * EXTI15_10_IRQHandler
 * Handles user button on PC13
 * Toggles servo direction CW / STOP / CCW / STOP
 ****************************************************************/
void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR13){
        EXTI->PR = EXTI_PR_PR13;
        dir_state = (dir_state+1)%4;
        if(dir_state==1 || dir_state==3) pwm_us = SERVO_STOP;
    }
}

/****************************************************************
 * main
 * Initializes all peripherals and runs main control loop
 * Adjusts PWM width based on ADC reading and direction
 ****************************************************************/
int main(void){
    GPIO_Init();
    ADC1_Init();
    TIM3_PWM_IC_Init();
    TIM2_Init();
    USART2_Init();
    SSD_init();

    // Short startup delay
    for(volatile int i=0;i<1000000;i++);

    // Configure SysTick for 2 ms interrupts
    SysTick_Config(FREQ/500);

    while(1){
        adc_val = ADC1_Read();

        // Direction control
        if(dir_state==0)      pwm_us = 1480 - (adc_val*200/4095);  // CW
        else if(dir_state==2) pwm_us = 1520 + (adc_val*200/4095);  // CCW
        else                  pwm_us = SERVO_STOP;                 // STOP

        if(pwm_us < SERVO_MIN) pwm_us = SERVO_MIN;
        if(pwm_us > SERVO_MAX) pwm_us = SERVO_MAX;
        TIM3->CCR1 = pwm_us;
    }
}
