/******************************************************************************
TestRun.ino
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
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
#define PWMB 6
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

// Radar/Sonar setup
Servo myservo;
#define SERVO_PIN 10  // sets the servo control to pin 10
const int trigPin = 13;                // Trigger Pin connected to A13
const int echoPin = 12;                // Echo Pin connected to A12
const unsigned int maxDistance = 200;  // Maximum distance to measure
int angle = 0;
NewPing sonar(trigPin, echoPin, maxDistance);  // NewPing setup of pins and maximum distance.
unsigned int sweepAndMeasure(int angle) {
  myservo.write(angle);                     // radar to angle
  delay(60);                                // Short delay to allow the servo to reach the position
  unsigned int distance = sonar.ping_cm();  // Measure distance in mm
  //client.print(angle);
  //client.print(", ");
  //client.println(measuredDistance);
  return distance;
}

//Wifi - Replace with your network credentials
const char* ssid = "WLAN-lumster";
const char* password = "401Cordoba";
WiFiServer server(80);
WiFiClient client;  // Declare WiFiClient object globally

//keyboard interrupt
int keyInt = 0;
void handleClientInput() {
  if (client.available()) {
    char c = client.read();
    if (c == 'A') keyInt = 86;
    if (c == '8') keyInt += 50;
    if (c == '2') keyInt -= 50;
    if (keyInt < 0) keyInt = 0;
    if (c == 'F') keyInt = 888;
    if (c == 'B') keyInt = 222;
    if (c == 'L') keyInt = 444;
    if (c == 'R') keyInt = 666;
    //Serial.print("from PC: ");
    //Serial.println(keyInt);
  }
}

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  Serial.print("====================\n");
  //Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.print("Connected to ");
  server.begin();
  delay(1000);
  Serial.print(WiFi.localIP());
  Serial.println("...Start python script.");
}

void loop() {
  //Use of the drive function which takes as arguements the speed
  //and optional duration.  A negative speed will cause it to go
  //backwards.  Speed can be from -255 to 255.  Also use of the
  //brake function which takes no arguements.

  //   motor1.drive(255,1000);
  //   motor1.drive(-255,1000);
  //   motor1.brake();
  //   delay(1000);

  client = server.available();  // Move this line to the beginning of the loop
  //Serial.println("Client connected");
  while (client.connected()) {
    handleClientInput();
    if (keyInt == 86) break;

    if (keyInt == 888) {
      unsigned int measuredMaxDistance = 999;
      for (angle = 75; angle <= 105; angle += 15) {
        unsigned int measuredDistance = sweepAndMeasure(angle);
        if (measuredDistance < measuredMaxDistance) measuredMaxDistance = measuredDistance;
      }
      //unsigned int measuredDistance = sweepAndMeasure(angle);
      if (measuredMaxDistance <= 30) brake(motor1, motor2);
      else forward(motor1, motor2, 200);
      delay(400);
    }

    //Use of the back/forward function, which takes as arguments two motors
    //and optionally a speed.  Either a positive number or a negative
    //number for speed will cause it to go backwards
    if (keyInt == 222) {
      back(motor1, motor2, -200);
      delay(1000);
    }

    //Use of the left and right functions which take as arguements two
    //motors and a speed.  This function turns both motors to move in
    //the appropriate direction.  For turning a single motor use drive.
    if (keyInt == 666) {
      left(motor1, motor2, 200);
      delay(800);
    }

    if (keyInt == 444) {
      right(motor1, motor2, -200);
      delay(800);
    }

    //Use of the brake function which takes as arguments two motors.
    //Note that functions do not stop motors on their own.
    brake(motor1, motor2);
    delay(500);
    keyInt = 0;  //remove for continuous run
  }
  client.stop();
}