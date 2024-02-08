/******************************************************************************
MotorAutoRun.ino

https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>
#include <SPI.h>
#include <WiFiS3.h>
#include <SparkFun_TB6612.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 7   //2
#define BIN1 8   // 7
#define AIN2 9   // 4 no idea (9 works, but maybe this isn't used - check and delete later)
#define BIN2 10  // 8 no idea (10 works, but  maybe this isn't used - check and delete later)
#define PWMA 5
#define PWMB 6  // PWM values controls balance
#define STBY 3  // 9

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
//Use of the drive function which takes as arguements the speed
//and optional duration.  A negative speed will cause it to go
//backwards.  Speed can be from -255 to 255.  Also use of the
//brake function which takes no arguements.

//   motor1.drive(255,1000);
//   motor1.drive(-255,1000);
//   motor1.brake();
//   delay(1000);
void forward(Motor motor1, int speed1, Motor motor2, int speed2)  //adjusts the PWM duty cycle of each motor independently for straightline calibration.
{
  motor1.drive(speed1);
  motor2.drive(speed2);
}

// Radar/Sonar setup
Servo myservo;
#define SERVO_PIN 10                   // sets the servo control to pin 10
const int trigPin = 13;                // Trigger Pin connected to A13
const int echoPin = 12;                // Echo Pin connected to A12
const unsigned int maxDistance = 300;  // Maximum distance to measure
const int boresightOffset = 5;
NewPing sonar(trigPin, echoPin, maxDistance);  // NewPing setup of pins and maximum distance.

unsigned int sweepAndMeasure(bool sweepMode, WiFiClient& client, int& direction) {
  unsigned long measuredMaxDistance = 999999;
  direction = 90;
  if (sweepMode) {
    for (int angle = 30; angle <= 150; angle += 40) {
      myservo.write(angle + boresightOffset);   // radar to angle
      delay(20);                                // Short delay to allow the servo to reach the position
      unsigned long distance = sonar.ping_median();  // Measure distance in cm
      if (distance < measuredMaxDistance) {
        measuredMaxDistance = distance;
        direction = angle;
      }
      //client.print(direction);
      //client.print(", ");
      //client.println(distance);
    }
  } else {
    myservo.write(90 + boresightOffset);
    delay(20);
    measuredMaxDistance = sonar.ping_median();
    //Serial.println(measuredMaxDistance);
  }
  return measuredMaxDistance;
}

//Wifi - Replace with your network credentials
const char* ssid = "WLAN-lumster";
const char* password = "401Cordoba";
WiFiServer server(80);
WiFiClient client;  // Declare WiFiClient object globally

// Keyboard interrupt
int keyInt = 0;

const long threshold = 2000;  // detection threshold (mm?)
bool ackReceived = false;  // Flag to indicate whether acknowledgment is received
int direction = 90;

void handleClientInput() {
  if (client.available()) {
    char c = client.read();
    if (c == 'X') {
      keyInt = 86;
      ackReceived = true;  // Set the acknowledgment flag
    } else if (c == '8' || c == '2' || c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
      keyInt += (c == '8') ? 50 : (c == '2') ? -50
                                : (c == 'F') ? 888
                                : (c == 'B') ? 222
                                : (c == 'L') ? 444
                                : (c == 'S') ? 555
                                             : 666;
      ackReceived = true;  // Set the acknowledgment flag
    }
    //Serial.print("from PC: ");
    //Serial.println(keyInt);
  }
}

void waitForAck() {
  while (!ackReceived) {
    handleClientInput();  // Check for acknowledgment
    delay(10);
  }
  ackReceived = false;  // Reset acknowledgment flag for the next command
}

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  //Serial.println(ssid);
  /*WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.print("Connected to ");
  server.begin();
  delay(500);
  //Serial.print(WiFi.localIP());
  Serial.println("...Start python script.");*/
}

void loop() {

  //client = server.available();  // Move this line to the beginning of the loop
  client = 0;
  //Serial.println("Client connected");
  //while (client.connected()) {
  keyInt = 888;
  //handleClientInput();
  //if (keyInt == 86) break;  //Terminate from ESC key

  while (keyInt == 888) {  // Move forward continuously until Distance <= 35

    unsigned int Distance = 999999; // Initialize Distance to a large value
    while (Distance > threshold) {  // compare against detection threshold
      Distance = sweepAndMeasure(false, client, direction);
      forward(motor1, 140, motor2, 150);
      delay(50);  // Adjust this delay as needed to control the poll frequency
    }
    brake(motor1, motor2);
    unsigned int distance = sweepAndMeasure(true, client, direction);  //rescan
    delay(50);
    switch (direction) {
      case 150:  //turn left based on direction
        left(motor1, motor2, 200);
        delay(850);
        break;
      case 110:
        left(motor1, motor2, 200);
        delay(425);
        break;
      case 70:  //turn right based on direction
        right(motor1, motor2, 200);
        delay(425);
        break;
      case 30:
        right(motor1, motor2, 200);
        delay(850);
        break;
    }
    brake(motor1, motor2);
     //Serial.print(direction);
    //Serial.print(", ");
    //Serial.println(distance);
    //waitForAck();  // Uncomment if acknowledgment is required after each forward movement
  }
  //brake(motor1, motor2); //remove for continuous run of last command
  //keyInt = 0; //reset keyInt for next client command
  //}
  unsigned long distance = sweepAndMeasure(false, client, direction);  //re-center sonar
  client.stop();
}