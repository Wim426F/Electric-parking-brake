#include <Arduino.h>
#include <STM32_CAN.h>
#include <EEPROM.h>
#include <math.h>  // For isnan() and fabs()

#define LED_PIN PB12
#define MOTOR_EN PA1   // EN pin for PWM (IN1)
#define MOTOR_PH PA2   // PH pin for direction (IN2)
#define MOTOR_CURRENT PA3
#define DRIVER_SLEEP PA15

#define CAN_ID_COMMAND 0x3FD
#define CAN_ID_CONFIG 0x3FF
#define CAN_ID_STATUS 0x3FE
#define PARK_DIRECTION 32

STM32_CAN Can(CAN1, DEF);
static CAN_message_t rxMsg, txMsg;

HardwareTimer *MyTim = NULL;

float ENGAGE_CURRENT_THRESHOLD = 5.5; // Default 6A
float DISENGAGE_THRESHOLD = 3; // Default 4A

bool ledState = false;
unsigned long lastBlink = 0;
unsigned long lastStatus = 0;
unsigned long engageStart = 0;
unsigned long disengageStart = 0;
const unsigned long BLINK_INTERVAL = 200;
const unsigned long STATUS_INTERVAL = 500;
const unsigned long ENGAGE_TIMEOUT = 15000; // 15 seconds (shared for engage/disengage)
const unsigned long MIN_ENGAGE_RUN_TIME = 3000; // 4 seconds minimum before checking current
const unsigned long MIN_DISENGAGE_RUN_TIME = 3000; // 4 seconds minimum before checking current
const unsigned long DISENGAGE_RAMP_TIME = 500; // 0.5 seconds at 100% before dropping to 50% PWM

enum BrakeState {
  DISENGAGED = 0,
  ENGAGING = 1,
  ENGAGED = 2,
  DISENGAGING = 3,
  ENGAGE_FAILED = 4
};
BrakeState currentState = DISENGAGED;

float getCurrent(int pin) {
  int adc = analogRead(pin);
  float voltage = (adc / 1024.0) * 3.3;
  float signedCurrent = (voltage - 1.65) / 0.132; // Compute signed value
  return fabs(signedCurrent); // Return absolute value (positive magnitude)
}

void setMotor(int mode, bool lowSpeed = false) {
  // mode: 1 = engage (forward), -1 = disengage (reverse), 0 = stop
  // lowSpeed: true for 30% PWM on EN during disengage ramp-down
  int duty = lowSpeed ? 25 : 100;

  digitalWrite(DRIVER_SLEEP, HIGH); // wakeup driver

  if (mode == 1) { // Engage: PH HIGH, EN PWM
    digitalWrite(MOTOR_PH, HIGH);
    MyTim->setCaptureCompare(2, duty, PERCENT_COMPARE_FORMAT); // EN 100% or 50% (unused for engage)
  } else if (mode == -1) { // Disengage: PH LOW, EN PWM
    digitalWrite(MOTOR_PH, LOW);
    MyTim->setCaptureCompare(2, duty, PERCENT_COMPARE_FORMAT); // EN 100% or 50%
  } else { // Stop: EN 0%
    MyTim->setCaptureCompare(2, 0, PERCENT_COMPARE_FORMAT);
  }

  digitalWrite(DRIVER_SLEEP, HIGH); // shutdown driver

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
  if (!isnan(savedEngageThreshold) && savedEngageThreshold >= 1.0 && savedEngageThreshold <= 10.0) {
    ENGAGE_CURRENT_THRESHOLD = savedEngageThreshold;
  }
  float savedDisengageThreshold;
  EEPROM.get(4, savedDisengageThreshold);
  if (!isnan(savedDisengageThreshold) && savedDisengageThreshold >= 1.0 && savedDisengageThreshold <= 10.0) {
    DISENGAGE_THRESHOLD = savedDisengageThreshold;
  }
  
  Can.begin();
  Can.setBaudRate(500000);
  txMsg.id = CAN_ID_STATUS;
  txMsg.len = 7; // State (1) + engage threshold (2) + disengage threshold (2) + measured current (2)
  txMsg.flags.extended = 0;
  
  currentState = DISENGAGED; // Initial state
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
  static bool parkRequested = false;
  float brakeCurrent = getCurrent(MOTOR_CURRENT);

  // Smooth with EWMA for stability (reduces oscillations from load noise)
  static float prev_brakeCurrent = 0.0f;  // Persistent across calls
  brakeCurrent = 0.05f * brakeCurrent + 0.95f * prev_brakeCurrent;
  prev_brakeCurrent = brakeCurrent;
  
  // Read CAN messages
  while (Can.read(rxMsg)) 
  {
    if (rxMsg.id == CAN_ID_COMMAND && rxMsg.len >= 3) // Message from VCU, lever position
    {
      parkRequested = (rxMsg.buf[2] == PARK_DIRECTION);
    } 
    else if (rxMsg.id == CAN_ID_CONFIG && rxMsg.len >= 4) // Configuration message
    {
      uint16_t rawEngageCurrent = (rxMsg.buf[1] << 8) | rxMsg.buf[0];
      ENGAGE_CURRENT_THRESHOLD = rawEngageCurrent / 10.0;
      ENGAGE_CURRENT_THRESHOLD = constrain(ENGAGE_CURRENT_THRESHOLD, 1.0, 10.0);
      EEPROM.put(0, ENGAGE_CURRENT_THRESHOLD);

      uint16_t rawDisengageCurrent = (rxMsg.buf[3] << 8) | rxMsg.buf[2];
      DISENGAGE_THRESHOLD = rawDisengageCurrent / 10.0;
      DISENGAGE_THRESHOLD = constrain(DISENGAGE_THRESHOLD, 1.0, 10.0);
      EEPROM.put(4, DISENGAGE_THRESHOLD);
    }
  }
  
  // Process State Machine
  switch (currentState) {
    case DISENGAGED:
      if (parkRequested) {
        currentState = ENGAGING;
        engageStart = currentMillis;
        setMotor(1); // Engage at 100%
      }
      break;
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
      // PWM ramp: 100% for first 2s, then 50%
      if (currentMillis - disengageStart >= DISENGAGE_RAMP_TIME) {
        setMotor(-1, true); // Switch to 50% after 2s to avoid ramming end stop and getting stuck
      } else {
        setMotor(-1); // 100% initially
      }
      // Only check current after ramp (at 50% PWM) and min runtime to avoid low-duty false negatives
      if (currentMillis - disengageStart >= DISENGAGE_RAMP_TIME) {
        if (currentMillis - disengageStart >= MIN_DISENGAGE_RUN_TIME && brakeCurrent >= DISENGAGE_THRESHOLD) {
          setMotor(0); // Stop on current rise
          currentState = DISENGAGED;
        } else if (currentMillis - disengageStart >= ENGAGE_TIMEOUT) {
          setMotor(0); // Stop on timeout
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
  
  // Send Status
  if (currentMillis - lastStatus >= STATUS_INTERVAL) {
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