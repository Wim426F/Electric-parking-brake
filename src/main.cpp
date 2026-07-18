#include <Arduino.h>
#include <STM32_CAN.h>
#include <EEPROM.h>
#include <math.h>  // For isnan() and fabs()
#include <STM32LowPower.h>

#define LED_PIN PB12
#define MOTOR_EN PA1   // EN pin for PWM (IN1)
#define MOTOR_PH PA2   // PH pin for direction (IN2)
#define MOTOR_CURRENT PA3
#define DRIVER_SLEEP PA15

#define CAN_ID_COMMAND 0x3FD
#define CAN_ID_CONFIG 0x3FF
#define CAN_ID_STATUS 0x3FE
#define CAN_ID_VEHICLE_STATE 0x480
#define CAN_ID_SPEED 0xCE // 206 DSC WheelSpeeds: 4x int16 LE, 0.0625 kph/bit
#define PARK_DIRECTION 32
#define VEHICLE_ON_BYTE 0x32

STM32_CAN Can(CAN1, DEF);
static CAN_message_t rxMsg, txMsg;

HardwareTimer *MyTim = NULL;

float ENGAGE_CURRENT_THRESHOLD = 8;
float DISENGAGE_THRESHOLD = 0.5;

bool ledState = false;
unsigned long lastBlink = 0;
unsigned long lastStatus = 0;
unsigned long engageStart = 0;
unsigned long disengageStart = 0;
unsigned long lastVehicleOnMsg = 0;
unsigned long sleepDelayStart = 0;

const unsigned long BLINK_INTERVAL = 200;
const unsigned long STATUS_INTERVAL = 500;
const unsigned long ENGAGE_TIMEOUT = 5000;
const unsigned long MIN_ENGAGE_RUN_TIME = 1000;
const unsigned long MIN_DISENGAGE_RUN_TIME = 3000; // disengage is 3x slower then enage.
const unsigned long DISENGAGE_RAMP_TIME = 250; // ms at 100% before lower speed releasing
const unsigned long DISENGAGE_TIMEOUT = 2000; // Hard stop: if no current rise is seen, give up and latch DISENGAGED
const unsigned long EMERGENCY_RAMP_TIME = 500; // Emergency clamp: ramp EN 0 -> 90% over this time
const unsigned long VEHICLE_ON_TIMEOUT = 2000; // CAN-bus timing, not motor-dependent: unchanged
const unsigned long SLEEP_DELAY = 600000; 

// Wheel-speed thresholds (kph). Hysteresis: a park request at/above MOVING_SPEED
// triggers an emergency clamp; the vehicle is only "stopped" below STANDSTILL_SPEED.
const float MOVING_SPEED = 3.0;
const float STANDSTILL_SPEED = 2.0;

enum BrakeState {
  DISENGAGED = 0,
  ENGAGING = 1,
  ENGAGED = 2,
  DISENGAGING = 3,
  ENGAGE_FAILED = 4,
  EMERGENCY_CLAMPING = 5
};
BrakeState currentState = ENGAGED;

enum SleepState {
  AWAKE = 0,
  WAITING_FOR_BRAKE_FINISH = 1,
  WAITING_FOR_CAN_QUIET = 2
};
SleepState sleepState = AWAKE;

bool vehicleOn = false;
float maxWheelSpeed = 0.0f; // Highest wheel-speed magnitude (kph) from 0xCE

float getCurrent(int pin) {
  int adc = analogRead(pin);
  float voltage = (adc / 1024.0) * 3.3;
  float signedCurrent = (voltage - 1.65) / 0.132; // Compute signed value
  return fabs(signedCurrent); // Return absolute value (positive magnitude)
}

void setMotor(int mode, bool lowSpeed = false, int customDuty = -1) {
  // mode: 1 = engage (forward), -1 = disengage (reverse), 0 = stop
  // lowSpeed: true for gentle PWM on EN during disengage ramp-down.
  // customDuty: if >= 0, overrides duty (used for the emergency-clamp ramp-up).
  int duty = (customDuty >= 0) ? customDuty : (lowSpeed ? 25 : 50); // 20 and 60 worked fine. TEST pls

  digitalWrite(DRIVER_SLEEP, HIGH); // wakeup driver

  if (mode == 1) { // Engage: PH HIGH, EN PWM
    digitalWrite(MOTOR_PH, HIGH);
    MyTim->setCaptureCompare(2, duty, PERCENT_COMPARE_FORMAT);
  } else if (mode == -1) { // Disengage: PH LOW, EN PWM
    digitalWrite(MOTOR_PH, LOW);
    MyTim->setCaptureCompare(2, duty, PERCENT_COMPARE_FORMAT);
  } else { // Stop: EN 0%
    MyTim->setCaptureCompare(2, 0, PERCENT_COMPARE_FORMAT);
  }

  digitalWrite(DRIVER_SLEEP, HIGH); // shutdown driver

}

void enterSleepMode() {
  setMotor(0);
  digitalWrite(LED_PIN, HIGH); // Off when HIGH

  if (MyTim != NULL) {
    MyTim->pause();
  }

  // CAN RX line goes low on the start-of-frame bit; use it as the wake source.
  LowPower.attachInterruptWakeup(PA11, NULL, FALLING);

  LowPower.deepSleep();

  // Stop mode preserves RAM but leaves peripherals (CAN especially) in an
  // unusable state. Force a full reset so setup() re-runs and every peripheral
  // — current and future — is reinitialized from scratch.
  NVIC_SystemReset();
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_PH, OUTPUT);
  pinMode(DRIVER_SLEEP, OUTPUT);
  pinMode(MOTOR_CURRENT, INPUT_ANALOG);
  
  MyTim = new HardwareTimer(TIM2);
  MyTim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_EN); // Channel 2 for EN (PA1) PWM
  MyTim->setOverflow(20000, HERTZ_FORMAT); // 20 kHz frequency
  MyTim->resume();
  
  setMotor(0); // Ensure stopped
  
  float savedEngageThreshold;
  EEPROM.get(0, savedEngageThreshold);
  if (!isnan(savedEngageThreshold) && savedEngageThreshold >= 1 && savedEngageThreshold <= 20) {
    ENGAGE_CURRENT_THRESHOLD = savedEngageThreshold;
  }
  float savedDisengageThreshold;
  EEPROM.get(4, savedDisengageThreshold);
  if (!isnan(savedDisengageThreshold) && savedDisengageThreshold >= 1 && savedDisengageThreshold <= 20) {
    DISENGAGE_THRESHOLD = savedDisengageThreshold;
  }
  
  Can.begin();
  Can.setBaudRate(500000);
  txMsg.id = CAN_ID_STATUS;
  txMsg.len = 7; // State (1) + engage threshold (2) + disengage threshold (2) + measured current (2)
  txMsg.flags.extended = 0;
  
  currentState = ENGAGED; // Initial state
  sleepState = AWAKE;
  vehicleOn = false;
  digitalWrite(DRIVER_SLEEP, HIGH); // shutdown driver
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Blink LED
  if (currentMillis - lastBlink >= BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH); // On when LOW
    lastBlink = currentMillis;
  }
  
  // Gather Inputs
  static bool parkRequested = true;
  float brakeCurrent = getCurrent(MOTOR_CURRENT);

  // Smooth with EWMA for stability
  static float prev_brakeCurrent = 0.0f;
  brakeCurrent = 0.5f * brakeCurrent + 0.5f * prev_brakeCurrent;
  prev_brakeCurrent = brakeCurrent;
  
  // Read CAN messages
  while (Can.read(rxMsg)) 
  {
    if (rxMsg.id == CAN_ID_VEHICLE_STATE && rxMsg.len >= 2) // Vehicle state message
    {
      vehicleOn = (rxMsg.buf[1] == VEHICLE_ON_BYTE);
      if (vehicleOn) {
        lastVehicleOnMsg = currentMillis;
        // If we were waiting to sleep, cancel it
        if (sleepState != AWAKE) {
          sleepState = AWAKE;
        }
      }
    }
    else if (rxMsg.id == CAN_ID_COMMAND && rxMsg.len >= 3) // Message from VCU, lever position
    {
      parkRequested = (rxMsg.buf[2] == PARK_DIRECTION);
    }
    else if (rxMsg.id == CAN_ID_SPEED && rxMsg.len >= 8) // DSC individual wheel speeds
    {
      // Track the fastest wheel: during hard braking a single wheel can lock and
      // read ~0 while the car still moves, so standstill must mean ALL wheels stopped.
      float maxSpeed = 0.0f;
      for (int i = 0; i < 8; i += 2) {
        int16_t raw = (int16_t)(rxMsg.buf[i] | (rxMsg.buf[i + 1] << 8));
        float wheel = fabs(raw * 0.0625f);
        if (wheel > maxSpeed) maxSpeed = wheel;
      }
      maxWheelSpeed = maxSpeed;
    }
    else if (rxMsg.id == CAN_ID_CONFIG && rxMsg.len >= 4) // Configuration message
    {
      uint16_t rawEngageCurrent = (rxMsg.buf[1] << 8) | rxMsg.buf[0];
      ENGAGE_CURRENT_THRESHOLD = rawEngageCurrent / 10.0;
      ENGAGE_CURRENT_THRESHOLD = constrain(ENGAGE_CURRENT_THRESHOLD, 1.0, 20.0);
      EEPROM.put(0, ENGAGE_CURRENT_THRESHOLD);

      uint16_t rawDisengageCurrent = (rxMsg.buf[3] << 8) | rxMsg.buf[2];
      DISENGAGE_THRESHOLD = rawDisengageCurrent / 10.0;
      DISENGAGE_THRESHOLD = constrain(DISENGAGE_THRESHOLD, 1.0, 20.0);
      EEPROM.put(4, DISENGAGE_THRESHOLD);
    }
  }
  
  // Check if vehicle has turned off (no 0x480 message for VEHICLE_ON_TIMEOUT)
  if (lastVehicleOnMsg > 0 && (currentMillis - lastVehicleOnMsg >= VEHICLE_ON_TIMEOUT)) {
    vehicleOn = false;
  }

  // Derived wheel-speed flags. If no 0xCE has been seen, maxWheelSpeed stays 0,
  // so vehicleMoving is false and we fall back to the normal (static) engage.
  bool vehicleMoving = (maxWheelSpeed >= MOVING_SPEED);

  // Process State Machine
  switch (currentState) {
    case DISENGAGED:
      if (parkRequested) {
        engageStart = currentMillis;
        // Park requested while rolling -> emergency clamp (keep driving until we
        // stop, then latch). Otherwise a normal static engage.
        currentState = vehicleMoving ? EMERGENCY_CLAMPING : ENGAGING;
        setMotor(1); // Engage at 100%
      }
      break;
    case EMERGENCY_CLAMPING: {
      // Same as a normal engage, but ramp the PWM from 0 -> 90% over EMERGENCY_RAMP_TIME.
      unsigned long elapsed = currentMillis - engageStart;
      int rampDuty = (elapsed >= EMERGENCY_RAMP_TIME)
                       ? 90
                       : (int)(90UL * elapsed / EMERGENCY_RAMP_TIME);
      setMotor(1, false, rampDuty);
      if (!parkRequested) { // Abort and disengage
        currentState = DISENGAGING;
        disengageStart = currentMillis;
        setMotor(-1); // Disengage at 100%
      } else if (elapsed >= MIN_ENGAGE_RUN_TIME && brakeCurrent >= ENGAGE_CURRENT_THRESHOLD) {
        setMotor(0); // Clamped to target force -> latch
        currentState = ENGAGED;
      } else if (elapsed >= ENGAGE_TIMEOUT) {
        setMotor(0); // Never reached target clamp force in time
        currentState = ENGAGE_FAILED;
      }
      break;
    }
    case ENGAGING:
      if (currentMillis - engageStart >= MIN_ENGAGE_RUN_TIME && brakeCurrent >= ENGAGE_CURRENT_THRESHOLD) {
        setMotor(0); // Stop
        currentState = ENGAGED;
      } else if (currentMillis - engageStart >= ENGAGE_TIMEOUT) {
        setMotor(0); // Stop
        currentState = ENGAGE_FAILED;
      } else if (!parkRequested) { // Abort and disengage
        currentState = DISENGAGING;
        disengageStart = currentMillis;
        setMotor(-1); // Disengage at 100%
      }
      break;
    case ENGAGED:
      if (!parkRequested) {
        currentState = DISENGAGING;
        disengageStart = currentMillis;
        setMotor(-1); // Disengage at 100%
      }
      break;
    case DISENGAGING:
      // PWM ramp: 100% for DISENGAGE_RAMP_TIME, then drop to low-speed duty
      if (currentMillis - disengageStart >= DISENGAGE_RAMP_TIME) {
        setMotor(-1, true); // Low speed to avoid ramming end stop and getting stuck
      } else {
        setMotor(-1); // 100% initially
      }
      // Only check current after ramp (at 50% PWM) and min runtime to avoid low-duty false negatives
      if (currentMillis - disengageStart >= DISENGAGE_RAMP_TIME) {
        if (currentMillis - disengageStart >= MIN_DISENGAGE_RUN_TIME && brakeCurrent >= DISENGAGE_THRESHOLD) {
          setMotor(0); // Stop on current rise
          currentState = DISENGAGED;
        } else if (currentMillis - disengageStart >= DISENGAGE_TIMEOUT) {
          setMotor(0); // Hard stop: current never rose (e.g. input-side sensor reads ~0)
          currentState = DISENGAGED;
        }
      }
  break;
    case ENGAGE_FAILED:
      if (!parkRequested) {
        currentState = DISENGAGING;
        disengageStart = currentMillis;
        setMotor(-1); // Disengage at 100%
      }
      break;
  }
  
  // Process Sleep State Machine
  switch (sleepState) {
    case AWAKE:
      // Check if vehicle has turned off
      if (!vehicleOn && lastVehicleOnMsg > 0) {
        // Check if brake operation is in progress
        if (currentState == ENGAGING || currentState == DISENGAGING || currentState == EMERGENCY_CLAMPING) {
          sleepState = WAITING_FOR_BRAKE_FINISH;
        } else {
          // Brake is idle, start waiting for CAN to go quiet
          sleepState = WAITING_FOR_CAN_QUIET;
          sleepDelayStart = currentMillis;
        }
      }
      break;
      
    case WAITING_FOR_BRAKE_FINISH:
      // Wait until brake operation completes
      if (currentState == DISENGAGED || currentState == ENGAGED || currentState == ENGAGE_FAILED) {
        sleepState = WAITING_FOR_CAN_QUIET;
        sleepDelayStart = currentMillis;
      }
      // If vehicle turns back on, return to awake
      if (vehicleOn) {
        sleepState = AWAKE;
      }
      break;
      
    case WAITING_FOR_CAN_QUIET:
      // Wait for few minutes of no vehicle activity
      if (currentMillis - sleepDelayStart >= SLEEP_DELAY) {
        // Time to sleep
        enterSleepMode();
        // After waking up, we'll be back in AWAKE state
      }
      // If vehicle turns back on, return to awake
      if (vehicleOn) {
        sleepState = AWAKE;
      }
      break;
  }
  
  // Send Status
  unsigned long status_interval = STATUS_INTERVAL;
  if (currentState == ENGAGING || currentState == DISENGAGING || currentState == EMERGENCY_CLAMPING)
  {
    status_interval = 50; // burst status during movement for more accurate tracking.
  }

  if (currentMillis - lastStatus >= status_interval) {
    lastStatus = currentMillis;
    txMsg.buf[0] = currentState;
    uint16_t rawEngageThreshold = ENGAGE_CURRENT_THRESHOLD * 10;
    txMsg.buf[1] = rawEngageThreshold & 0xFF;
    txMsg.buf[2] = (rawEngageThreshold >> 8) & 0xFF;
    uint16_t rawDisengageThreshold = DISENGAGE_THRESHOLD * 10;
    txMsg.buf[3] = rawDisengageThreshold & 0xFF;
    txMsg.buf[4] = (rawDisengageThreshold >> 8) & 0xFF;
    uint16_t rawMeasuredCurrent = brakeCurrent * 10;
    txMsg.buf[5] = rawMeasuredCurrent & 0xFF;
    txMsg.buf[6] = (rawMeasuredCurrent >> 8) & 0xFF;
    Can.write(txMsg);
  }
}