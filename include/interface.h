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
#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdint.h>
#include "params.h"
#include "canhardware.h"

/* Custom CAN protocol, byte-compatible with the previous (Arduino) firmware.
 * Tuning parameters (currents, timings, etc.) are configured over CAN SDO /
 * the terminal, not through a dedicated message. */
#define CAN_ID_COMMAND       0x3FD //VCU lever position, byte 2 == 32 -> park
#define CAN_ID_STATUS        0x3FE //our status broadcast
#define CAN_ID_VEHICLE_STATE 0x480 //byte 1 == 0x32 -> vehicle on
#define CAN_ID_WHEELSPEED    0xCE  //DSC wheel speeds: 4x int16 LE, 0.0625 kph/bit

#define PARK_DIRECTION       32
#define VEHICLE_ON_BYTE      0x32

class Interface
{
public:
   static bool CanReceive(uint32_t id, uint32_t data[2], uint8_t dlc);
   static void RegisterMessages(CanHardware* can);
   static void CheckVehicleTimeout(); //call every 10 ms
   static void SendStatus(CanHardware* can); //call every 10 ms

private:
   static void HandleCommand(const uint8_t* bytes);
   static void HandleVehicleState(const uint8_t* bytes);
   static void HandleWheelSpeed(const uint8_t* bytes);

   static bool everSeenVehicleOn;
   static uint32_t lastVehicleOnTick;
};

#endif // INTERFACE_H
