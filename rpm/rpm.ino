#include "TimerOne.h"

const byte MOTOR1 = 2; // Motor 1 interrupt pin
const byte MOTOR2 = 3; // Motor 2 interrupt pin

unsigned int counter1 = 0;
unsigned int counter2 = 0;

float slots = 20.00;

void ISR_count1()
{
  counter1++;
}
void ISR_count2()
{
  counter2++;
}


// TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt(); // stop timer
  Serial.print("Motor 1 Speed: ");
  float rpm1 = (counter1 / slots) * 60.00; // calc RPM motor 1
  Serial.print(rpm1);
  Serial.println(" RPM - ");
  counter1 = 0; // reset counter

  Serial.print("Motor 2 Speed: ");
  float rpm2 = (counter2 / slots) * 60.00; // calc RPM motor 2
  Serial.print(rpm2);
  Serial.println(" RPM - ");
  counter2 = 0; // reset counter

  
  Timer1.attachInterrupt( ISR_timerone ); // enable timer
}

void setup() {
  Serial.begin(9600);

  Timer1.initialize(1000000); // set at 1 sec
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING); // increase counter by 1 when sensor goes HIGH
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING); // increase counter by 1 when sensor goes HIGH
  Timer1.attachInterrupt( ISR_timerone ); // enable timer
}

void loop() {
 
}
