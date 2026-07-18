#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

#define RCC_CLOCK_SETUP() rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ])

//Motor driver EN/PWM pin: PA1 = TIM2 CH2 (no remap), 20 kHz
#define PWM_MOTOR_TIMER    TIM2
#define PWM_MOTOR_OC       TIM_OC2
#define PWM_MOTOR_ARR      3599U  // 72 MHz / 3600 = 20 kHz
#define PWM_MOTOR_PSC      0U

//Address of parameter block in flash
#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1   //last block of 1k
#define CAN1_BLKNUM   2
#define CAN2_BLKNUM   4


#endif // HWDEFS_H_INCLUDED
