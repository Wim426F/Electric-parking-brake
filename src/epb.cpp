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

/* Parking brake state machine.
 *
 * The motor stalls against the brake mechanism at both ends of travel, so
 * clamp force (engage) and the release end-stop are detected by the rise in
 * motor current. Current readings are only trusted after a minimum run time,
 * which blanks the inrush spike and, on release, the fast ramp phase.
 *
 * Engaging while the car is rolling (EMERGENCY_CLAMPING) is like a normal
 * engage but ramps the PWM from 0 to clamp_duty over clamp_ramp ms instead
 * of applying the engage duty at once.
 */
#include "epb.h"
#include "digio.h"
#include "anain.h"
#include "hwdefs.h"
#include "my_math.h"
#include "errormessage.h"
#include <libopencm3/stm32/timer.h>

#define MOTOR_STOP       0
#define MOTOR_ENGAGE     1
#define MOTOR_DISENGAGE -1

//Current sense amplifier: fixed hardware characteristics
#define CURRENT_GAIN_MV_PER_A 132.0f
#define CURRENT_OFFSET_MV     1650.0f

int Epb::state = STATE_ENGAGED;
float Epb::filteredCurrent = 0;
uint32_t Epb::stateStartTime = 0;
uint32_t Epb::now = 0;

/** The real brake position is unknown at power-on. Assume engaged with park
 * requested so the brake is never released until the VCU explicitly
 * commands it. */
void Epb::Init()
{
   Param::SetInt(Param::opmode, state);
   Param::SetInt(Param::park_request, 1);
}

/** Read the current sense amplifier and return the unsigned motor current.
 * The amplifier output is bidirectional around the mid-rail offset, we only
 * care about the magnitude. */
float Epb::MeasureCurrent()
{
   float mV = AnaIn::current.Get() * (3300.0f / 4095.0f);
   float amps = (mV - CURRENT_OFFSET_MV) / CURRENT_GAIN_MV_PER_A;
   return ABS(amps);
}

/** mode: MOTOR_ENGAGE, MOTOR_DISENGAGE or MOTOR_STOP.
 * duty: PWM duty in percent (ignored for MOTOR_STOP). Each phase of travel
 * uses its own configurable duty parameter. */
void Epb::SetMotor(int mode, int duty)
{
   if (mode == MOTOR_STOP)
   {
      timer_set_oc_value(PWM_MOTOR_TIMER, PWM_MOTOR_OC, 0);
      DigIo::motor_sleep.Clear(); //put driver to sleep, saves quiescent current
      Param::SetInt(Param::motor_duty, 0);
   }
   else
   {
      DigIo::motor_sleep.Set(); //wake driver
      if (mode == MOTOR_ENGAGE)
         DigIo::motor_ph.Set();
      else
         DigIo::motor_ph.Clear();
      timer_set_oc_value(PWM_MOTOR_TIMER, PWM_MOTOR_OC, ((PWM_MOTOR_ARR + 1) * duty) / 100);
      Param::SetInt(Param::motor_duty, duty);
   }
}

void Epb::SetState(int newState)
{
   state = newState;
   stateStartTime = now;
   Param::SetInt(Param::opmode, newState);
}

void Epb::Run()
{
   now += 10;

   float current = MeasureCurrent();
   //EWMA smoothing against load noise
   filteredCurrent += Param::GetFloat(Param::current_filter) * (current - filteredCurrent);
   Param::SetFloat(Param::current, filteredCurrent);

   bool parkRequested = Param::GetBool(Param::park_request);
   float wheelSpeed = Param::GetFloat(Param::wheelspeed);
   //A park request at/above speed_moving triggers an emergency clamp (ramped
   //engage) instead of a static one. With no 0xCE received yet wheelspeed
   //stays 0 -> normal static engage.
   bool vehicleMoving = wheelSpeed >= Param::GetFloat(Param::speed_moving);

   uint32_t elapsed = now - stateStartTime;
   uint32_t engageTimeout = Param::GetInt(Param::engage_timeout);

   switch (state)
   {
   case STATE_DISENGAGED:
      if (parkRequested)
      {
         if (vehicleMoving)
         {
            SetState(STATE_EMERGENCY_CLAMPING);
            SetMotor(MOTOR_ENGAGE, 0); //ramp starts from 0
         }
         else
         {
            SetState(STATE_ENGAGING);
            SetMotor(MOTOR_ENGAGE, Param::GetInt(Param::engage_duty));
         }
      }
      break;

   case STATE_EMERGENCY_CLAMPING:
   {
      //Same as a normal engage, but ramp the PWM from 0 -> clamp_duty over
      //clamp_ramp ms to avoid shock-loading the mechanism at speed
      int clampDuty = Param::GetInt(Param::clamp_duty);
      uint32_t rampTime = Param::GetInt(Param::clamp_ramp);
      int rampDuty = (elapsed >= rampTime) ? clampDuty
                                           : (int)(clampDuty * elapsed / rampTime);
      SetMotor(MOTOR_ENGAGE, rampDuty);
      if (!parkRequested) //abort and release
      {
         SetState(STATE_DISENGAGING);
         SetMotor(MOTOR_DISENGAGE, Param::GetInt(Param::disengage_duty));
      }
      else if (elapsed >= (uint32_t)Param::GetInt(Param::min_engage_time) &&
               filteredCurrent >= Param::GetFloat(Param::engage_current))
      {
         SetMotor(MOTOR_STOP); //clamped to target force -> latch
         SetState(STATE_ENGAGED);
      }
      else if (elapsed >= engageTimeout)
      {
         SetMotor(MOTOR_STOP); //never reached target clamp force in time
         SetState(STATE_ENGAGE_FAILED);
         ErrorMessage::Post(ERR_ENGAGEFAILED);
      }
      break;
   }

   case STATE_ENGAGING:
      if (elapsed >= (uint32_t)Param::GetInt(Param::min_engage_time) &&
          filteredCurrent >= Param::GetFloat(Param::engage_current))
      {
         SetMotor(MOTOR_STOP);
         SetState(STATE_ENGAGED);
      }
      else if (elapsed >= engageTimeout)
      {
         SetMotor(MOTOR_STOP);
         SetState(STATE_ENGAGE_FAILED);
         ErrorMessage::Post(ERR_ENGAGEFAILED);
      }
      else if (!parkRequested) //abort and release
      {
         SetState(STATE_DISENGAGING);
         SetMotor(MOTOR_DISENGAGE, Param::GetInt(Param::disengage_duty));
      }
      break;

   case STATE_ENGAGED:
      if (!parkRequested)
      {
         SetState(STATE_DISENGAGING);
         SetMotor(MOTOR_DISENGAGE, Param::GetInt(Param::disengage_duty));
      }
      break;

   case STATE_DISENGAGING:
      //disengage_duty for release_ramp ms to get the clamp moving, then drop to
      //release_duty for a gentle approach to the end stop
      if (elapsed >= (uint32_t)Param::GetInt(Param::release_ramp))
      {
         SetMotor(MOTOR_DISENGAGE, Param::GetInt(Param::release_duty));
         //Only check current after the ramp and minimum run time to avoid
         //false trips on the inrush/ramp current
         if (elapsed >= (uint32_t)Param::GetInt(Param::min_release_time) &&
             filteredCurrent >= Param::GetFloat(Param::release_current))
         {
            SetMotor(MOTOR_STOP); //end stop reached
            SetState(STATE_DISENGAGED);
         }
         else if (elapsed >= (uint32_t)Param::GetInt(Param::release_timeout))
         {
            SetMotor(MOTOR_STOP); //hard stop: no current rise seen, assume released
            SetState(STATE_DISENGAGED);
         }
      }
      else
      {
         SetMotor(MOTOR_DISENGAGE, Param::GetInt(Param::disengage_duty));
      }
      break;

   case STATE_ENGAGE_FAILED:
      if (!parkRequested)
      {
         SetState(STATE_DISENGAGING);
         SetMotor(MOTOR_DISENGAGE, Param::GetInt(Param::disengage_duty));
      }
      break;
   }
}
