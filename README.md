# stm32-epb

Electric parking brake controller for a BMW E90 EV conversion. A DC motor
clamps the brake through a self-locking spindle. Clamp force and the release
end stop are detected by motor current. The VCU commands it over CAN.

Built on [libopencm3](https://github.com/jsphuebner/libopencm3) +
[libopeninv](https://github.com/jsphuebner/libopeninv). All tuning happens
through the parameter system: serial terminal, CAN SDO (node id 48) or the
[esp8266 web interface](https://github.com/jsphuebner/esp8266-web-interface).

## Hardware

STM32F103C8

| Pin | Function |
|-----|----------|
| PA1 | Motor driver EN (20 kHz PWM) |
| PA2 | Motor driver PH (direction) |
| PA3 | Current sense (132 mV/A, 1.65 V mid-rail) |
| PA15 | Motor driver nSLEEP |
| PB12 | Status LED |
| PA11/PA12 | CAN |
| PA9/PA10 | Serial terminal |

## How it works

- **Engage:** run the motor until the current reaches `engage_current`, then
  stop. The spindle is self-locking, so the motor stays off.
- **Release:** `disengage_duty` for `release_ramp` ms, then a slow approach
  at `release_duty` until the end-stop current rise, or `release_timeout`
  as a hard stop.
- **Emergency clamp:** park requested while driving (≥ `speed_moving` kph).
  Same as engage, but the PWM ramps 0 → `clamp_duty` over `clamp_ramp` ms.
  Wheel speeds come from the DSC message, fastest wheel counts.
- Current readings are ignored for the first `min_engage_time` /
  `min_release_time` to blank the inrush spike.
- At power-on the brake assumes it's engaged and parked. It won't release
  until the VCU says so.

## CAN

Received:

| ID | Content |
|----|---------|
| 0x3FD | Lever position: byte 2 == 32 → park requested |
| 0x480 | Vehicle state: byte 1 == 0x32 → vehicle on |
| 0xCE  | DSC wheel speeds: 4× int16 LE, 0.0625 kph/bit |

Sent, 0x3FE status every 500 ms (50 ms while the motor runs):

| Byte | Content |
|------|---------|
| 0    | State: 0=Disengaged 1=Engaging 2=Engaged 3=Disengaging 4=EngageFailed 5=EmergencyClamping |
| 1-2  | Engage threshold ×0.1 A (LE) |
| 3-4  | Release threshold ×0.1 A (LE) |
| 5-6  | Motor current ×0.1 A (LE) |

## Parameters

| Parameter | Unit | Default | Description |
|-----------|------|---------|-------------|
| canspeed | - | 500k | CAN baud rate |
| nodeid | - | 48 | SDO node id |
| engage_current | A | 8 | Target clamp current |
| release_current | A | 0.5 | End-stop detection current |
| engage_timeout | ms | 5000 | Give up engaging → EngageFailed |
| release_timeout | ms | 2000 | Hard stop on release |
| min_engage_time | ms | 1000 | Blanking before the engage current check |
| min_release_time | ms | 3000 | Blanking before the release current check |
| engage_duty | % | 50 | Duty while engaging |
| clamp_duty | % | 50 | Peak duty of the emergency-clamp ramp |
| clamp_ramp | ms | 500 | Emergency-clamp ramp time |
| disengage_duty | % | 50 | Duty for the fast release phase |
| release_ramp | ms | 250 | Fast release phase duration |
| release_duty | % | 25 | Duty for the slow end-stop approach |
| speed_moving | kph | 3 | Above this a park request becomes an emergency clamp |
| vehicle_timeout | ms | 2000 | No 0x480 for this long → vehicle off |
| current_filter | - | 0.5 | Current EWMA coefficient (100 Hz) |

## Build

```
make get-deps   # once: submodules + libopencm3
make
```

Needs `arm-none-eabi-gcc`, `make` and Python. On Windows, build from Git
Bash.

## Flash

The first 4 KB are reserved for the
[CAN bootloader](https://github.com/jsphuebner/stm32-CANBootloader) — flash
that once, then:

- `make flash` (ST-Link/OpenOCD), or
- upload `stm32_epb.bin` over CAN via the web interface
