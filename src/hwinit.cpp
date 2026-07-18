/*
 * This file is part of the stm32-epb project.
 *
 * Copyright (C) 2026 Wim Boone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include "hwdefs.h"
#include "hwinit.h"
#include "stm32_loader.h"
#include "my_string.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
   RCC_CLOCK_SETUP();

   //The reset value for PRIGROUP (=0) is not actually a defined
   //value. Explicitly set 16 preemtion priorities
   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_USART1); //Terminal
   rcc_periph_clock_enable(RCC_TIM2);   //Motor PWM
   rcc_periph_clock_enable(RCC_TIM3);   //Scheduler
   rcc_periph_clock_enable(RCC_DMA1);   //ADC and UART receive
   rcc_periph_clock_enable(RCC_ADC1);
   rcc_periph_clock_enable(RCC_CRC);
   rcc_periph_clock_enable(RCC_AFIO);   //CAN
   rcc_periph_clock_enable(RCC_CAN1);   //CAN

   //Free PA15 (motor driver nSLEEP) from JTAG, keep SWD for debugging/flashing
   AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;
}

/* Some pins should never be left floating at any time
 * Since the bootloader delays firmware startup by a few 100ms
 * We need to tell it which pins we want to initialize right
 * after startup
 */
void write_bootloader_pininit()
{
   uint32_t flashSize = desig_get_flash_size();
   uint32_t pindefAddr = FLASH_BASE + flashSize * 1024 - PINDEF_BLKNUM * PINDEF_BLKSIZE;
   const struct pincommands* flashCommands = (struct pincommands*)pindefAddr;

   struct pincommands commands;

   memset32((int*)&commands, 0, PINDEF_NUMWORDS);

   //Keep the motor driver inactive while the bootloader runs:
   //PA1 (EN/PWM) and PA2 (PH) low. nSLEEP (PA15) is a JTAG pin during
   //boot and can't be driven here, but with EN low the driver won't move.
   commands.pindef[0].port = GPIOA;
   commands.pindef[0].pin = GPIO1 | GPIO2;
   commands.pindef[0].inout = PIN_OUT;
   commands.pindef[0].level = 0;
   //LED (PB12, active low) off during boot
   commands.pindef[1].port = GPIOB;
   commands.pindef[1].pin = GPIO12;
   commands.pindef[1].inout = PIN_OUT;
   commands.pindef[1].level = 1;

   crc_reset();
   uint32_t crc = crc_calculate_block(((uint32_t*)&commands), PINDEF_NUMWORDS);
   commands.crc = crc;

   if (commands.crc != flashCommands->crc)
   {
      flash_unlock();
      flash_erase_page(pindefAddr);

      //Write flash including crc, therefor <=
      for (uint32_t idx = 0; idx <= PINDEF_NUMWORDS; idx++)
      {
         uint32_t* pData = ((uint32_t*)&commands) + idx;
         flash_program_word(pindefAddr + idx * sizeof(uint32_t), *pData);
      }
      flash_lock();
   }
}

/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
   nvic_enable_irq(NVIC_TIM3_IRQ); //Scheduler
   nvic_set_priority(NVIC_TIM3_IRQ, 0xe << 4); //second lowest priority
}

void rtc_setup()
{
   //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
   //62.5kHz / (624 + 1) = 100Hz
   rtc_auto_awake(RCC_HSE, 624); //10ms tick
   rtc_set_counter_val(0);
}

/**
* Setup the motor PWM timer: TIM2 CH2 on PA1, 20 kHz edge aligned.
* Duty cycle is set at runtime in epb.cpp, starts at 0% (motor off).
*/
void tim_setup()
{
   timer_disable_counter(PWM_MOTOR_TIMER);
   timer_set_alignment(PWM_MOTOR_TIMER, TIM_CR1_CMS_EDGE);
   timer_set_prescaler(PWM_MOTOR_TIMER, PWM_MOTOR_PSC);
   timer_set_period(PWM_MOTOR_TIMER, PWM_MOTOR_ARR);
   timer_enable_preload(PWM_MOTOR_TIMER);

   timer_set_oc_mode(PWM_MOTOR_TIMER, PWM_MOTOR_OC, TIM_OCM_PWM1);
   timer_enable_oc_preload(PWM_MOTOR_TIMER, PWM_MOTOR_OC);
   timer_set_oc_polarity_high(PWM_MOTOR_TIMER, PWM_MOTOR_OC);
   timer_enable_oc_output(PWM_MOTOR_TIMER, PWM_MOTOR_OC);
   timer_set_oc_value(PWM_MOTOR_TIMER, PWM_MOTOR_OC, 0); //0% = motor off

   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO1);

   timer_generate_event(PWM_MOTOR_TIMER, TIM_EGR_UG);
   timer_enable_counter(PWM_MOTOR_TIMER);
}
