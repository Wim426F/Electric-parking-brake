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

/* This file contains all parameters used in your project
 * See main.cpp on how to access them.
 * If a parameters unit is of format "0=Choice, 1=AnotherChoice" etc.
 * It will be displayed as a dropdown in the web interface
 * If it is a spot value, the decimal is translated to the name, i.e. 0 becomes "Choice"
 * If the enum values are powers of two, they will be displayed as flags, example
 * "0=None, 1=Flag1, 2=Flag2, 4=Flag3, 8=Flag4" and the value is 5.
 * It means that Flag1 and Flag3 are active -> Display "Flag1 | Flag3"
 *
 * Every parameter/value has a unique ID that must never change. This is used when loading parameters
 * from flash, so even across firmware versions saved parameters in flash can always be mapped
 * back to our list here. If a new value is added, it will receive its default value
 * because it will not be found in flash.
 * The unique ID is also used in the CAN module, to be able to recover the CAN map
 * no matter which firmware version saved it to flash.
 * Make sure to keep track of your ids and avoid duplicates. Also don't re-assign
 * IDs from deleted parameters because you will end up loading some random value
 * into your new parameter!
 * IDs are 16 bit, so 65535 is the maximum
 */

 //Define a version string of your firmware here
#define VER 2.00.R

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 21
//Retired param ids (do not reuse!): 11, 13, 14
//Next value Id: 2009
/*              category     name              unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(CAT_COMM,    canspeed,         CANSPEEDS, 0,      4,      2,      1   ) \
    PARAM_ENTRY(CAT_COMM,    nodeid,           "",        1,      63,     48,     2   ) \
    PARAM_ENTRY(CAT_BRAKE,   engage_current,   "A",       1,      20,     8,      3   ) \
    PARAM_ENTRY(CAT_BRAKE,   release_current,  "A",       0.1,    20,     0.5,    4   ) \
    PARAM_ENTRY(CAT_BRAKE,   engage_timeout,   "ms",      1000,   60000,  5000,   5   ) \
    PARAM_ENTRY(CAT_BRAKE,   release_timeout,  "ms",      100,    60000,  2000,   20  ) \
    PARAM_ENTRY(CAT_BRAKE,   min_engage_time,  "ms",      0,      20000,  1000,   6   ) \
    PARAM_ENTRY(CAT_BRAKE,   min_release_time, "ms",      0,      20000,  3000,   7   ) \
    PARAM_ENTRY(CAT_BRAKE,   engage_duty,      "%",       10,     100,    50,     16  ) \
    PARAM_ENTRY(CAT_BRAKE,   clamp_duty,       "%",       10,     100,    50,     18  ) \
    PARAM_ENTRY(CAT_BRAKE,   clamp_ramp,       "ms",      0,      5000,   500,    19  ) \
    PARAM_ENTRY(CAT_BRAKE,   disengage_duty,   "%",       10,     100,    50,     17  ) \
    PARAM_ENTRY(CAT_BRAKE,   release_ramp,     "ms",      0,      5000,   250,    8   ) \
    PARAM_ENTRY(CAT_BRAKE,   release_duty,     "%",       5,      100,    25,     9   ) \
    PARAM_ENTRY(CAT_VEHICLE, speed_moving,     "kph",     0.5,    30,     3,      10  ) \
    PARAM_ENTRY(CAT_VEHICLE, vehicle_timeout,  "ms",      100,    10000,  2000,   12  ) \
    PARAM_ENTRY(CAT_SENSE,   current_filter,   "",        0.03,   1,      0.5,    15  ) \
    VALUE_ENTRY(opmode,       EPBSTATES,       2000 ) \
    VALUE_ENTRY(version,      VERSTR,          2001 ) \
    VALUE_ENTRY(lasterr,      errorListString, 2002 ) \
    VALUE_ENTRY(cpuload,      "%",             2003 ) \
    VALUE_ENTRY(current,      "A",             2004 ) \
    VALUE_ENTRY(wheelspeed,   "kph",           2005 ) \
    VALUE_ENTRY(park_request, ONOFF,           2006 ) \
    VALUE_ENTRY(vehicle_on,   ONOFF,           2007 ) \
    VALUE_ENTRY(motor_duty,   "%",             2008 )


/***** Enum String definitions *****/
//Numeric values are sent as-is in byte 0 of the 0x3FE status message,
//so they must never be re-ordered (they match the old firmware's states)
#define EPBSTATES    "0=Disengaged, 1=Engaging, 2=Engaged, 3=Disengaging, 4=EngageFailed, 5=EmergencyClamping"
#define CANSPEEDS    "0=125k, 1=250k, 2=500k, 3=800k, 4=1M"
#define ONOFF        "0=Off, 1=On"
#define CAT_COMM     "Communication"
#define CAT_BRAKE    "Parking Brake"
#define CAT_VEHICLE  "Vehicle"
#define CAT_SENSE    "Current Sensing"

#define VERSTR STRINGIFY(4=VER-epb)

/***** enums ******/

enum _epbstates
{
   STATE_DISENGAGED = 0,
   STATE_ENGAGING = 1,
   STATE_ENGAGED = 2,
   STATE_DISENGAGING = 3,
   STATE_ENGAGE_FAILED = 4,
   STATE_EMERGENCY_CLAMPING = 5
};

//Generated enum-string for possible errors
extern const char* errorListString;
