//#include <MechaQMC5883.h>

//MechaQMC5883 qmc;  // piece of shit, also, if you uncomment this, it uses A4 and A5 which prevents communication. 

// ************* DEFINE SERVO and SERVO PINS ********************************
# include <Servo.h>
Servo myservo;

// for later, for optical encoder for speed control - remember ONLY pins 2 and 3 are interrupt driven capable. 
// get the code from rpm/car3.ino

#define SERVO_PIN 3
// #define SERVO_PIN 13  // hack if you decide to use optical rotary encoders

#include <SoftwareSerial.h>
const int BT_RX = 0; // Connect to TX of Bluetooth Module
const int BT_TX = 1; // Connect to RX of Bluetooth Module

// Constants
SoftwareSerial BTSerial(BT_RX, BT_TX); // RX, TX




// ************* DEFINE ULTRA SOUND PINS ********************************
#include <NewPing.h>
const int trigPin = A5; // Trigger Pin connected to A5
const int echoPin = A4; // Echo Pin connected to A4
const unsigned int maxDistance = 200; // Maximum distance to measure

NewPing sonar(trigPin, echoPin, maxDistance); // NewPing setup of pins and maximum distance.

// ************* DEFINE CAR MOTOR PINS ********************************
//    The direction of the car's movement
//  ENA   ENB   IN1   IN2   IN3   IN4   Description
//  HIGH  HIGH  HIGH  LOW   LOW   HIGH  Car is runing forward
//  HIGH  HIGH  LOW   HIGH  HIGH  LOW   Car is runing back
//  HIGH  HIGH  LOW   HIGH  LOW   HIGH  Car is turning left
//  HIGH  HIGH  HIGH  LOW   HIGH  LOW   Car is turning right
//  HIGH  HIGH  LOW   LOW   LOW   LOW   Car is stoped
//  HIGH  HIGH  HIGH  HIGH  HIGH  HIGH  Car is stoped
//  LOW   LOW   N/A   N/A   N/A   N/A   Car is stoped


//define L298n module IO Pin
#define ENB 5
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define ENA 6

void sweepAndMeasure(int angle) {
  myservo.write(angle); // radar to angle
  delay(20); // Short delay to allow the servo to reach the position
  unsigned int distance = sonar.ping_cm(); // Measure distance
  BTSerial.print(angle);
  BTSerial.print(",");
  BTSerial.println(distance);
  BT_Ack();
}

void BT_Ack() {
  while (true) {
    if (BTSerial.available() )  {
      char ack = BTSerial.read();
      if (ack == 'A') {
        break;  // Continue if acknowledgement is received
      } else if (ack == 'F') {
        forward_new();
      } else if (ack == 'B') {
        back();
        delay(500); // Move backward for 500 ms
        stopped();  // Stop the car
        forward();  // immediately stop and go opposite direction for 10ms to stay straight
        delay(10);
        stopped();
      }
      else if (ack == 'L') {
        left();
        delay(515); // Rotate left for 1060 ms  950mS =~ 180degrees, so 20 degrees should be 106 ms delay
        stopped();  // Stop the car
      }
      // Add other commands if necessary
    }
  }
}

void setSpeed(int leftWheel, int rightWheel) {
  // speed values should be between 0-255. 255 is Max speed.
  analogWrite(ENA, leftWheel);
  analogWrite(ENB, rightWheel);
}

void forward_new() {
      int enb = 250;
      // forward
      analogWrite(ENA, 220);
      analogWrite(ENB, enb);
      digitalWrite(IN1, HIGH);    //set IN1 hight level
      digitalWrite(IN2, LOW);     //set IN2 low level
      digitalWrite(IN3, LOW);     //set IN3 low level
      digitalWrite(IN4, HIGH);    //set IN4 hight level
      delay(500); // for 1/2 sec

      stopped();   // stop
      delay(250);

      //analogWrite(ENA, 220);     // two right wheels
      analogWrite(ENB, enb);   // rotate two left wheels to bring it back straight
      digitalWrite(IN1, HIGH);    //set IN1 hight level
      digitalWrite(IN2, LOW);     //set IN2 low level
      digitalWrite(IN3, LOW);     //set IN3 low level
      digitalWrite(IN4, HIGH);    //set IN4 hight level
      delay(30);                  // kick the car slightly to the right to keep it straight (somewhere between 30-37ms, also the stopped delay is important)
      stopped();
}  
  
void forward(){
  // looks like ~ideal LEFT = 220, RIGHT = 245 (maybe 245, 246, 247)
  // Dec 19, enb = 249
  //digitalWrite(ENA, HIGH);    //enable L298n A channel
  //digitalWrite(ENB, HIGH);    //enable L298n B channel

  analogWrite(ENA, 220);
  analogWrite(ENB, 250);
  
  digitalWrite(IN1, HIGH);    //set IN1 hight level
  digitalWrite(IN2, LOW);     //set IN2 low level
  digitalWrite(IN3, LOW);     //set IN3 low level
  digitalWrite(IN4, HIGH);    //set IN4 hight level
  //Serial.println("Forward");  //send message to serial monitor
}

void back(){
  //digitalWrite(ENA, HIGH);
  //digitalWrite(ENB, HIGH);

  analogWrite(ENA, 220);
  analogWrite(ENB, 250);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //Serial.println("Back");
}

void left(){
  //digitalWrite(ENA, HIGH);
  //digitalWrite(ENB, HIGH);
  
  analogWrite(ENA, 220);
  analogWrite(ENB, 250); 

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  //Serial.println("Left");
}

void right(){
//  digitalWrite(ENA, HIGH);
//  digitalWrite(ENB, HIGH);

  analogWrite(ENA, 220);
  analogWrite(ENB, 250); 

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  //Serial.println("Right");
}

void stopped() {
  digitalWrite(ENB, LOW);
  digitalWrite(ENA, LOW);
}

//before execute loop() function, 
//setup() function will execute first and only execute once

// *******************************************************************
// VARIABLES

int turn_wheels_flag = 0;
int ultra_sound_flag = 0;
int sweep_ultra_sound_flag = 0;
int verify_bluetooth_serial_flag = 0;
int sweep_ultra_sound_with_blueooth_flag = 1;
int check_straight_flag = 0;
int sweep_servo_only_flag = 0;
int check_car_angle_movement_flag = 0;
int check_compass_flag = 0;

// *******************************************************************

void setup() {
  myservo.attach(SERVO_PIN);
  myservo.write(90);        // initialize to 90 deg
  BTSerial.begin(9600); // Bluetooth module baud rate
  //BTSerial.begin(115200); // Bluetooth module baud rate
  //Serial.begin(9600);     //open serial and set the baudrate
  //qmc.init();
  pinMode(IN1, OUTPUT);   //before useing io pin, pin mode must be set first 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

}

//Repeat execution
void loop() {
  if ( sweep_servo_only_flag ) {
    for (int angle = 30; angle <= 150; angle += 20) {
      myservo.write(angle); 
      delay(500);
    }

  }

  // ********** Check car angle movement flag ************************************
  if ( check_car_angle_movement_flag ) {
      for (int turn_time = 440; turn_time <=480; turn_time +=10) {
        int turn = 515;                   // 515 is the best I could come up with
        BTSerial.print(-1);
        BTSerial.print(",");
        BTSerial.println(turn);
        //BT_Ack();
        left();
        delay(turn);
        stopped();
        delay(7000);
      } 
  }


// ********** Check Compass ******************************
    if (check_compass_flag ) {
      int x, y, z;
      int azimuth;
      int delta, temp1, temp2;
      //qmc.read(&x, &y, &z, &azimuth);
      BTSerial.print(-1);
      BTSerial.print(",");
      BTSerial.println(azimuth);
      //BT_Ack();
      delay(1000);
    }

// ********** Check straight flag ************************************
  if (check_straight_flag ) {
    // start from 1m distance
    int angle = 0; // 90
    int avgCount = 10;
    int distance;
    int avgDistance;

    //int x, y, z;
    //int azimuth;
    //int delta, temp1, temp2;


    for (int enb = 245; enb < 255; enb++) {
      enb = 250;
      //qmc.read(&x, &y, &z, &azimuth);
      BTSerial.print(-1);
      BTSerial.print(",");
      BTSerial.println(enb);
//      BT_Ack();
      // forward
      analogWrite(ENA, 220);
      analogWrite(ENB, enb);
      digitalWrite(IN1, HIGH);    //set IN1 hight level
      digitalWrite(IN2, LOW);     //set IN2 low level
      digitalWrite(IN3, LOW);     //set IN3 low level
      digitalWrite(IN4, HIGH);    //set IN4 hight level
      delay(500); // for 1/2 sec

      stopped();   // stop
      //delay(3000);
      delay(250);

      //analogWrite(ENA, 220);     // two right wheels
      analogWrite(ENB, enb);   // two left wheels
      digitalWrite(IN1, HIGH);    //set IN1 hight level
      digitalWrite(IN2, LOW);     //set IN2 low level
      digitalWrite(IN3, LOW);     //set IN3 low level
      digitalWrite(IN4, HIGH);    //set IN4 hight level
      delay(30);                  // kick the car slightly to the right to keep it straight
      stopped();


      // Back
      //analogWrite(ENB, enb);
      //analogWrite(ENA, 220);
      //digitalWrite(IN1, LOW);
      //digitalWrite(IN2, HIGH);
      //digitalWrite(IN3, HIGH);
      //digitalWrite(IN4, LOW);
      //back();
      delay(500);

      stopped();
      delay(5000);
    }

  }


  // ********** SWEEP ULTRA SOUND WITH BLUETOOTH SERIAL ************************************
  if ( sweep_ultra_sound_with_blueooth_flag ) {
    // Sweep from 30 to 150 degrees and back to 30 and send angle and distance over Bluetooth serial connection and print to Serial Monitor
    for (int angle = 0; angle <= 180; angle += 5) {
      sweepAndMeasure(angle);
    }
    for (int angle = 180; angle >= 0; angle -= 5) {
      sweepAndMeasure(angle);
    }
  }

  // ********** VERIFY BLUETOOTH SERIAL ************************************
  if ( verify_bluetooth_serial_flag ) {
    int angle = 30;
    myservo.write(angle); // initialize to the front
    delay(30);
    unsigned int distance = sonar.ping_cm();

    BTSerial.print(angle);
    BTSerial.print(",");
    BTSerial.println(distance);
    BT_Ack();

  }


  // ********** SWEEP SERVO ************************************
  if ( sweep_ultra_sound_flag ) {
    myservo.write(90);
    unsigned int distance = sonar.ping_cm(); // Send ping, get distance in cm

    delay(2000);

    myservo.write(30);
    distance = sonar.ping_cm(); // Send ping, get distance in cm

    delay(2000);
    myservo.write(150);
    distance = sonar.ping_cm(); // Send ping, get distance in cm

    delay(2000);
    
  }



// ********** ULTRA SOUND ************************************
//delay(50); // Short delay between measurements
  if ( ultra_sound_flag ) {
    unsigned int distance = sonar.ping_cm(); // Send ping, get distance in cm
  }

 // ********** TURN THE WHEELS ************************************
  if ( turn_wheels_flag ) {
    forward();    //go forward
    delay(1000);  //delay 1000 ms
  
    stopped();
    delay(1000);
  
    back();       //go back
    delay(1000);
  
    stopped();
    delay(1000);
  
    left();       //turning left
    delay(1000);
  
    stopped();
    delay(1000);
  
    right();      //turning right
    delay(1000);
    
    stopped();
    delay(1000);
  }

}
