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
#include "interface.h"
#include "my_math.h"
#include <libopencm3/stm32/rtc.h>

bool Interface::everSeenVehicleOn = false;
uint32_t Interface::lastVehicleOnTick = 0;

void Interface::RegisterMessages(CanHardware* can)
{
   can->RegisterUserMessage(CAN_ID_COMMAND);
   can->RegisterUserMessage(CAN_ID_VEHICLE_STATE);
   can->RegisterUserMessage(CAN_ID_WHEELSPEED);
}

/* Called from the CAN receive interrupt */
bool Interface::CanReceive(uint32_t id, uint32_t data[2], uint8_t dlc)
{
   const uint8_t* bytes = (const uint8_t*)data;

   switch (id)
   {
   case CAN_ID_COMMAND:
      if (dlc >= 3) HandleCommand(bytes);
      break;
   case CAN_ID_VEHICLE_STATE:
      if (dlc >= 2) HandleVehicleState(bytes);
      break;
   case CAN_ID_WHEELSPEED:
      if (dlc >= 8) HandleWheelSpeed(bytes);
      break;
   }
   return false;
}

void Interface::HandleCommand(const uint8_t* bytes)
{
   Param::SetInt(Param::park_request, bytes[2] == PARK_DIRECTION);
}

void Interface::HandleVehicleState(const uint8_t* bytes)
{
   bool on = bytes[1] == VEHICLE_ON_BYTE;
   Param::SetInt(Param::vehicle_on, on);

   if (on)
   {
      everSeenVehicleOn = true;
      lastVehicleOnTick = rtc_get_counter_val();
   }
}

void Interface::HandleWheelSpeed(const uint8_t* bytes)
{
   //Track the fastest wheel: during hard braking a single wheel can lock and
   //read ~0 while the car still moves, so standstill must mean ALL wheels stopped.
   float maxSpeed = 0;

   for (int i = 0; i < 8; i += 2)
   {
      int16_t raw = (int16_t)(bytes[i] | (bytes[i + 1] << 8));
      float wheel = ABS(raw * 0.0625f);
      if (wheel > maxSpeed) maxSpeed = wheel;
   }
   Param::SetFloat(Param::wheelspeed, maxSpeed);
}

/** Consider the vehicle off when no vehicle-on message arrived within
 * vehicle_timeout, like the old firmware. Only armed once a vehicle-on
 * message has ever been seen. */
void Interface::CheckVehicleTimeout()
{
   if (everSeenVehicleOn && Param::GetBool(Param::vehicle_on))
   {
      uint32_t timeoutTicks = Param::GetInt(Param::vehicle_timeout) / 10; //RTC has 10ms ticks
      if ((rtc_get_counter_val() - lastVehicleOnTick) >= timeoutTicks)
         Param::SetInt(Param::vehicle_on, 0);
   }
}

/** Broadcast the status message: every 500 ms when idle, every 50 ms
 * while the motor runs for more accurate tracking. Called every 10 ms. */
void Interface::SendStatus(CanHardware* can)
{
   static int divider = 0;
   int state = Param::GetInt(Param::opmode);
   bool inMotion = state == STATE_ENGAGING || state == STATE_DISENGAGING ||
                   state == STATE_EMERGENCY_CLAMPING;

   divider++;
   if (divider < (inMotion ? 5 : 50)) return;
   divider = 0;

   uint8_t bytes[8];
   uint16_t engage = (uint16_t)(Param::GetFloat(Param::engage_current) * 10 + 0.5f);
   uint16_t release = (uint16_t)(Param::GetFloat(Param::release_current) * 10 + 0.5f);
   uint16_t current = (uint16_t)(Param::GetFloat(Param::current) * 10 + 0.5f);

   bytes[0] = state;
   bytes[1] = engage & 0xFF;
   bytes[2] = engage >> 8;
   bytes[3] = release & 0xFF;
   bytes[4] = release >> 8;
   bytes[5] = current & 0xFF;
   bytes[6] = current >> 8;
   bytes[7] = 0;

   can->Send(CAN_ID_STATUS, bytes, 7);
}
