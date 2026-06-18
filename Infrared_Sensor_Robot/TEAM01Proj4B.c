/****************************************************************
* TEAM01: Christopher Gardner | Ryan Atkins
* CPEG222 PROJ4B, 11/13/25
* NucleoF466RE CMSIS STM32F4xx
****************************************************************/
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdbool.h>
#include <stdio.h>
#include "UART2.h"
#include <stdlib.h>

#define BTN_PIN 13
#define SERVO_LEFT_PIN 8     /* PC8 TIM3_CH3 */
#define SERVO_RIGHT_PIN 9    /* PC9 TIM3_CH4 */
#define SERVO_PORT GPIOC

/* SSD state machine digit selector */
volatile int ssd_digit_index = 0;

/* mapping of digit index to SSD driver index */
const unsigned char ssd_digit_order[4] = {3, 2, 1, 0};

/* run control flags */
volatile bool robot_running = false;

/* end-line and lost-line tracking */
volatile int endline_tick_count = 0;
volatile int all_sensors_line_flag = 0;
volatile int lost_line_tick_count = 0;

/* steering mode state machine */
volatile int steering_state = 0;

/* lost-line threshold */
const int LOST_LINE_TIMEOUT_TICKS = 40; /* ~20 ms at 0.5 ms tick */

/* optional center lock feature */
volatile int center_lock_ticks = 0;
const int CENTER_LOCK_DURATION = 60;

/* persistence counters for hard edges */
volatile int right_edge_count = 0;
volatile int left_edge_count = 0;
const int EDGE_HYSTERESIS_TICKS = 6;

/* snap-back timers */
volatile int snap_timer_ticks = 0;
const int SNAP_DURATION_TICKS = 40;

/*  
   servo_set_both_us
   Sets both servo output pulse widths (clamped between 1000-2000 us)
     */
void servo_set_both_us(int left_us, int right_us)
{
    if (left_us < 1000) left_us = 1000;
    if (left_us > 2000) left_us = 2000;
    if (right_us < 1000) right_us = 1000;
    if (right_us > 2000) right_us = 2000;

    TIM3->CCR3 = (uint32_t)left_us;
    TIM3->CCR4 = (uint32_t)right_us;
}

/*  
   servo_both_stop
   Stops both servos with a 1500 us neutral pulse
     */
void servo_both_stop(void)
{
    servo_set_both_us(1500, 1500);
}

/*  
   servo_both_forward
   Drives both motors forward with tuned trim values
     */
void servo_both_forward(void)
{
    servo_set_both_us(1500 + 53, 1500 - 55);
}

/*  
   servo_turn_left
   Applies slight left steering correction
     */
void servo_turn_left(void)
{
    servo_set_both_us(1500 + 30, 1500 - 90);
}

/*  
   servo_turn_right
   Applies slight right steering correction
     */
void servo_turn_right(void)
{
    servo_set_both_us(1500 + 90, 1500 - 30);
}

/*  
   servo_pivot_right
   Performs a right spin pivot (left forward, right neutral)
     */
void servo_pivot_right(void)
{
    servo_set_both_us(1500 + 75, 1500);
}

/*  
   servo_pivot_left
   Performs a left spin pivot (right forward, left neutral)
     */
void servo_pivot_left(void)
{
    servo_set_both_us(1500, 1500 - 80);
}

/*  
   EXTI15_10_IRQHandler
   Handles the user button press to toggle robot run/stop state
     */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1 << BTN_PIN))
    {
        EXTI->PR |= (1 << BTN_PIN);

        if (robot_running)
        {
            robot_running = false;
            servo_both_stop();
        }
        else
        {
            endline_tick_count = 0;
            all_sensors_line_flag = 0;
            lost_line_tick_count = 0;
            steering_state = 0;
            center_lock_ticks = 0;
            right_edge_count = 0;
            left_edge_count = 0;
            snap_timer_ticks = 0;
            robot_running = true;
            servo_both_forward();
        }
    }
}

/*  
   TIM2_IRQHandler
   Main 0.5 ms ISR for SSD multiplexing and line-follow
     */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        /* read IR sensors PC0-PC3 (active low) */
        unsigned int port_val = GPIOC->IDR;
        int ir0 = ((port_val >> 0) & 1) ^ 1;
        int ir1 = ((port_val >> 1) & 1) ^ 1;
        int ir2 = ((port_val >> 2) & 1) ^ 1;
        int ir3 = ((port_val >> 3) & 1) ^ 1;

        /* update 4-digit SSD with IR pattern */
        int ir_display_value = ir3 * 1000 + ir2 * 100 + ir1 * 10 + ir0;
        SSD_update(ssd_digit_order[ssd_digit_index], ir_display_value, 4);
        ssd_digit_index = (ssd_digit_index + 1) % 4;

        if (robot_running)
        {
            /* pattern classification */
            int centered = (ir3 == 0 && ir2 == 1 && ir1 == 1 && ir0 == 0);

            int near_center =
                centered ||
                (ir3 == 0 && ir2 == 1 && ir1 == 0 && ir0 == 0) ||
                (ir3 == 0 && ir2 == 0 && ir1 == 1 && ir0 == 0);

            int slight_right =
                (ir3 == 1 && ir2 == 1 && ir1 == 0 && ir0 == 0) ||
                (ir3 == 0 && ir2 == 0 && ir1 == 0 && ir0 == 1);

            int slight_left =
                (ir3 == 0 && ir2 == 0 && ir1 == 1 && ir0 == 1) ||
                (ir3 == 1 && ir2 == 0 && ir1 == 0 && ir0 == 0);

            int hard_right = (ir3 == 1 && ir2 == 1 && ir1 == 1 && ir0 == 0);
            int hard_left  = (ir3 == 0 && ir2 == 1 && ir1 == 1 && ir0 == 1);

            /* persistence counters */
            if (hard_right) right_edge_count++;
            else right_edge_count = 0;

            if (hard_left) left_edge_count++;
            else left_edge_count = 0;

            /* latch/spin/snap state machine */
            if (steering_state == 1)
            {
                if (centered)
                {
                    steering_state = 3;
                    snap_timer_ticks = SNAP_DURATION_TICKS;
                    servo_pivot_left();
                }
                else
                {
                    servo_pivot_right();
                }
                lost_line_tick_count = 0;
            }
            else if (steering_state == 2)
            {
                if (centered)
                {
                    steering_state = 4;
                    snap_timer_ticks = SNAP_DURATION_TICKS;
                    servo_pivot_right();
                }
                else
                {
                    servo_pivot_left();
                }
                lost_line_tick_count = 0;
            }
            else if (steering_state == 3)
            {
                if (snap_timer_ticks-- > 0)
                {
                    servo_pivot_left();
                }
                else
                {
                    steering_state = 0;
                    servo_both_forward();
                }
                lost_line_tick_count = 0;
            }
            else if (steering_state == 4)
            {
                if (snap_timer_ticks-- > 0)
                {
                    servo_pivot_right();
                }
                else
                {
                    steering_state = 0;
                    servo_both_forward();
                }
                lost_line_tick_count = 0;
            }
            else
            {
                if (right_edge_count >= EDGE_HYSTERESIS_TICKS)
                {
                    steering_state = 1;
                    servo_pivot_right();
                    right_edge_count = 0;
                    lost_line_tick_count = 0;
                }
                else if (left_edge_count >= EDGE_HYSTERESIS_TICKS)
                {
                    steering_state = 2;
                    servo_pivot_left();
                    left_edge_count = 0;
                    lost_line_tick_count = 0;
                }
                else if (near_center)
                {
                    servo_both_forward();
                    lost_line_tick_count = 0;
                }
                else if (slight_right)
                {
                    servo_turn_right();
                    lost_line_tick_count = 0;
                }
                else if (slight_left)
                {
                    servo_turn_left();
                    lost_line_tick_count = 0;
                }
                else
                {
                    if (++lost_line_tick_count >= LOST_LINE_TIMEOUT_TICKS)
                    {
                        servo_both_stop();
                        lost_line_tick_count = LOST_LINE_TIMEOUT_TICKS;
                    }
                }
            }
        }
    }
}

/*  
   main
   Configures GPIO, TIM3 PWM, TIM2 ISR, EXTI button, SSDs, and runs idle
     */
int main(void)
{
    SSD_init();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PC13 button input with pull-up */
    GPIOC->MODER &= ~(3 << (13 * 2));
    GPIOC->PUPDR &= ~(3 << (13 * 2));
    GPIOC->PUPDR |=  (1 << (13 * 2));

    /* PC0-PC3 IR sensor inputs */
    GPIOC->MODER &= ~(3 << (0 * 2));
    GPIOC->PUPDR &= ~(3 << (0 * 2));
    GPIOC->PUPDR |=  (1 << (0 * 2));

    GPIOC->MODER &= ~(3 << (1 * 2));
    GPIOC->PUPDR &= ~(3 << (1 * 2));
    GPIOC->PUPDR |=  (1 << (1 * 2));

    GPIOC->MODER &= ~(3 << (2 * 2));
    GPIOC->PUPDR &= ~(3 << (2 * 2));
    GPIOC->PUPDR |=  (1 << (2 * 2));

    GPIOC->MODER &= ~(3 << (3 * 2));
    GPIOC->PUPDR &= ~(3 << (3 * 2));
    GPIOC->PUPDR |=  (1 << (3 * 2));

    /* PWM output pins PC8 and PC9 */
    GPIOC->MODER &= ~(3 << (SERVO_LEFT_PIN * 2));
    GPIOC->MODER |=  (2 << (SERVO_LEFT_PIN * 2));

    GPIOC->MODER &= ~(3 << (SERVO_RIGHT_PIN * 2));
    GPIOC->MODER |=  (2 << (SERVO_RIGHT_PIN * 2));

    GPIOC->AFR[1] &=
        ~((0xF << ((SERVO_LEFT_PIN - 8) * 4)) |
          (0xF << ((SERVO_RIGHT_PIN - 8) * 4)));

    GPIOC->AFR[1] |=
        (0x2 << ((SERVO_LEFT_PIN - 8) * 4)) |
        (0x2 << ((SERVO_RIGHT_PIN - 8) * 4));

    /* TIM3 PWM setup */
    TIM3->PSC = 15;
    TIM3->ARR = 19999;

    TIM3->CCMR2 =
        (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
        (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CCR3 = 1500;
    TIM3->CCR4 = 1500;
    TIM3->CR1 |= TIM_CR1_CEN;

    /* TIM2 0.5 ms interval */
    TIM2->PSC = 15;
    TIM2->ARR = 500 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    /* EXTI13 user button */
    EXTI->IMR  |= (1 << BTN_PIN);
    EXTI->FTSR |= (1 << BTN_PIN);
    SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4));
    SYSCFG->EXTICR[3] |=  (2 << (1 * 4));
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    servo_both_stop();

    while (1) {}
}
