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
#ifndef EPB_H
#define EPB_H

#include "params.h"

class Epb
{
public:
   static void Init(); //call once at startup
   static void Run(); //call every 10 ms

private:
   static void SetMotor(int mode, int duty = 0);
   static void SetState(int newState);
   static float MeasureCurrent();

   static int state;
   static float filteredCurrent;
   static uint32_t stateStartTime;
   static uint32_t now;
};

#endif // EPB_H
