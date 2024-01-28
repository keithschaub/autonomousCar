#include "TimerOne.h"

const byte MOTOR1 = 2; // Motor 1 interrupt pin
const byte MOTOR2 = 3; // Motor 2 interrupt pin

volatile unsigned int counter1 = 0;
volatile unsigned int counter2 = 0;

float slots = 20.00;

void ISR_count1() {
  counter1++;
}

void ISR_count2() {
  counter2++;
}

// TimerOne ISR
void ISR_timerone() {
  // Calculate RPM for Motor 1
  float rpm1 = (counter1 / slots) * 60.00;
  Serial.print("Motor 1 Speed: ");
  Serial.print(rpm1);
  Serial.println(" RPM");

  // Calculate RPM for Motor 2
  float rpm2 = (counter2 / slots) * 60.00;
  Serial.print("Motor 2 Speed: ");
  Serial.print(rpm2);
  Serial.println(" RPM");

  // Reset counters
  counter1 = 0;
  counter2 = 0;
}

void setup() {
  Serial.begin(9600);

  // Initialize Timer1 for 1-second interval
  Timer1.initialize(1000000);

  // Attach the ISR for TimerOne
  Timer1.attachInterrupt(ISR_timerone);

  // Attach the interrupt for Motor encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR1), ISR_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2), ISR_count2, RISING);
}

void loop() {
  // Nothing needed here for now
}
