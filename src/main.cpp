#include <Arduino.h>
#include <STM32_CAN.h>

#define LED_PIN PB12
#define RIGHT_IN1 PA1
#define RIGHT_IN2 PA2
#define LEFT_IN1 PA6
#define LEFT_IN2 PA7
#define RIGHT_CURRENT PA3
#define LEFT_CURRENT PB0

STM32_CAN Can(CAN1, DEF);
static CAN_message_t rxMsg, txMsg;

const float STOP_CURRENT = 5.0; // 5A threshold
const unsigned long BLINK_INTERVAL = 200;
const unsigned long STATUS_INTERVAL = 500;
const unsigned long DISENGAGE_TIME = 1000;

bool ledState = false;
bool leftEngaged = false;
bool rightEngaged = false;
bool disengaging = false;
unsigned long lastBlink = 0;
unsigned long lastStatus = 0;
unsigned long disengageStart = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  
  Can.begin();
  Can.setBaudRate(500000);
  txMsg.id = 0x3FE;
  txMsg.len = 2;
  txMsg.flags.extended = 0;
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Simple LED blink
  if (currentMillis - lastBlink >= BLINK_INTERVAL) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH); // On when LOW
    lastBlink = currentMillis;
  }
  
  // Handle CAN
  if (Can.read(rxMsg) && rxMsg.id == 0x3FD && rxMsg.len >= 3) {
    int direction = rxMsg.buf[2];
    if (direction == 32 && !leftEngaged && !rightEngaged) { // Park
      digitalWrite(RIGHT_IN1, HIGH);
      digitalWrite(RIGHT_IN2, LOW);
      digitalWrite(LEFT_IN1, HIGH);
      digitalWrite(LEFT_IN2, LOW);
    } else if ((leftEngaged || rightEngaged) && direction != 32) { // Not Park
      disengaging = true;
      disengageStart = currentMillis;
      digitalWrite(RIGHT_IN1, LOW);
      digitalWrite(RIGHT_IN2, HIGH);
      digitalWrite(LEFT_IN1, LOW);
      digitalWrite(LEFT_IN2, HIGH);
    }
  }
  
  // Manage brake
  float rightCurrent = getCurrent(RIGHT_CURRENT);
  float leftCurrent = getCurrent(LEFT_CURRENT);
  
  if (!rightEngaged && rightCurrent >= STOP_CURRENT) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    rightEngaged = true;
  }
  if (!leftEngaged && leftCurrent >= STOP_CURRENT) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    leftEngaged = true;
  }
  if (disengaging && currentMillis - disengageStart >= DISENGAGE_TIME) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    leftEngaged = false;
    rightEngaged = false;
    disengaging = false;
  }
  
  // Send status
  if (currentMillis - lastStatus >= STATUS_INTERVAL) {
    lastStatus = currentMillis;
    txMsg.buf[0] = leftEngaged ? 1 : 0;  // Byte 0 for left
    txMsg.buf[1] = rightEngaged ? 1 : 0; // Byte 1 for right
    Can.write(txMsg);
  }
}

float getCurrent(int pin) {
  int adc = analogRead(pin);
  float voltage = (adc / 4095.0) * 3.3;
  return (voltage - 1.65) / 0.132; // Sensitivity 0.132V/A
}