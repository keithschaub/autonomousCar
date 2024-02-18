#include <WiFiS3.h>

// Network credentials
const char* ssid = "thisistheway";
const char* password = "eatpie69";

WiFiServer server(80);
WiFiClient client;

//*********************** SETUP FOR MPU6050 *****************************
// Demo for getting individual unified sensor data from the MPU6050 (from Adafruit examples UnifiedSensors)
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_gyro;

sensors_event_t gyro;

float meanGyroZ = 0.0;
float GyroZ_calibrated = 0;
float heading_rad = 1.5708;
float heading_deg = 0.0;
float heading_start, heading_final = 0.0;

static unsigned long heading_lastTime = 0;
unsigned long heading_currentTime = 0;
float heading_deltaTime = 0.0;

float posX, posY;

unsigned long currentTime, lastTime, beginTime=0;
float deltaTime = 0.0;

// Define the maximum number of data points
const int maxDataPoints = 200;

// Declare arrays to store data
int dataIndex = 0;

double gyro_X[maxDataPoints];
int speed_array[maxDataPoints];
double timeStamps[maxDataPoints];

void updateHeading(); // Function prototype




// ***************** SETUP FOR THE MOTORS***************************
#include <SparkFun_TB6612.h>
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 7 //2
#define BIN1 8 // 7
#define AIN2 9 // 4 no idea (9 works, but maybe this isn't used - check and delete later)
#define BIN2 10 // 8 no idea (10 works, but  maybe this isn't used - check and delete later)
#define PWMA 5
#define PWMB 6
#define STBY 3 // 9

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Servo
#include <Servo.h>
Servo myservo;
#define SERVO_PIN 10

// Ultrasonic Sensor
#include <NewPing.h>
const int trigPin = 13;
const int echoPin = 12;
const unsigned int maxDistance = 200;
NewPing sonar(trigPin, echoPin, maxDistance);

// Setup for RPM
const byte MOTOR1 = 2; // Motor 1 interrupt pin
const byte MOTOR2 = 11; // Motor 2 interrupt pin

volatile unsigned int counter1 = 0;
volatile unsigned int counter2 = 0;

float rpm1, rpm2=0.0;
float distanceTraveled = 0.0;

const long interval = 2000;  // Interval at which to measure (2000 mS)

float slots = 20.00;  // Number of slots in the encoder

void ISR_count1() {
  counter1++;
}

void ISR_count2() {
  counter2++;
}


int acknowledge = 1; // flag used to control synch Transmit data with plot

void setup(void) {
  Serial.begin(115200);

    // Attach the interrupt for Motor encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR1), ISR_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2), ISR_count2, RISING);

  myservo.attach(SERVO_PIN);
  myservo.write(90);
  while (!Serial)
    delay(10); // will pause the program until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();
  calibrateGyroSensor();
  Serial.print("Gyro calibration completed ");

  Serial.println("pausing 1 second");
  delay(1000);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  server.begin();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Waiting for client to connect...");
}

void loop() {
  client = server.available();
  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      // MPU6050
      mpu_gyro->getEvent(&gyro);
      //slowForward();
      //updateHeading();
      //printData();
      //delay(100);

      if (client.available()) {
        char c = client.read();
        Serial.print("received : ");
        Serial.println(c);
        handleCommand(c);
      }
      if (acknowledge == 1) {
        String data_packet = prepareDataPacket();
        client.print(data_packet);
        client.flush();
        acknowledge = 0; // disable next transmit until PC acknowledges
      }
    }
    client.stop();
    Serial.println("Client disconnected");

  } else {
    myservo.write(90);
  }
}


void printData_Wifi() {
  if (client.connected()) {
    Serial.println("writing file");
    for (int i = 0; i < dataIndex; i++) {
      String dataString = "Hello\n";
      
                          //String(i) + "," + String(speed_array[i]) + "," +
                          //String(accel_X[i]) + "," + String(accel_Y[i]) + "," +
                          //String(accel_Z[i]) + "," + String(gyro_X[i]) + "," +
                          //String(gyro_Y[i]) + "," + String(gyro_Z[i]) + "," +
                          //String(timeStamps[i]) + "\n";
      //Serial.println(dataString);
      client.print(dataString);
      delay(10); // Small delay to ensure data is sent and processed
    }
    // Consider moving or removing client.stop(); to keep the connection open
  }
}

void updateHeading() {
    mpu_gyro->getEvent(&gyro);
    heading_currentTime = millis();
    heading_deltaTime = ((float)heading_currentTime - (float)heading_lastTime) / 1000.0;
    heading_lastTime = heading_currentTime;

    // Integrate angular velocity to get the angle

    GyroZ_calibrated = gyro.gyro.z - meanGyroZ;  // correct GyroZ using calibration mean (instantaneous heading)
 
    if (abs(GyroZ_calibrated) < 0.01) {  // if BELOW Threshold (0.01), ignore the gyro value
      GyroZ_calibrated = 0;
    }
    
    heading_rad += (GyroZ_calibrated) * heading_deltaTime;

    // Convert to degrees
    heading_deg = heading_rad*57.2958; // 1 radian = 57.2958
 
    // Normalize to 0-360 degrees
    heading_final = fmod(heading_deg, 360);
    if (heading_final < 0) heading_final +=360;

    if (0) {
      Serial.print("gyroZ (raw): " );
      Serial.print(gyro.gyro.z);
      Serial.print(", Active GyroZ: ");
      Serial.print("0 BELOW threshold");
      Serial.print(GyroZ_calibrated);
      Serial.print(", heading_rad+: ");
      Serial.print(heading_rad);
      Serial.print(", heading_deg: ");
      Serial.print(heading_deg);
      Serial.print(", heading_final = ");
      Serial.println(heading_final);
    }

    String heading_location_packet = "Heading " + String(heading_final) + "," + " deg " + "\n";
    client.print(heading_location_packet);
}

void slowForward() {
  int speed = 100;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading
  while (speed <= 200 ) {
    updateHeading();
    forward(motor1, motor2, speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed +=10;
    } 
  }
  while (speed > 0 ) {
    updateHeading();
    forward(motor1, motor2, speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed -=20;
    } 
  }
  brake(motor1, motor2);
 // Calculate RPM for Motors and distance traveled 
  calculate_RPM(counter1, counter2, rpm1, rpm2, distanceTraveled);
  updatePosition(); // determines new posX and posY
  printData();
}


void slowBackward() {
  int speed = 100;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading
  while (speed <= 200 ) {
    updateHeading();
    back(motor1, motor2, -speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed +=10;
    } 
  }
  while (speed > 0 ) {
    updateHeading();
    back(motor1, motor2, -speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed -=20;
    } 
  }
  brake(motor1, motor2);
  // Calculate RPM for Motors and distance traveled 
  calculate_RPM(counter1, counter2, rpm1, rpm2, distanceTraveled);
  updatePosition(); // determines new posX and posY
  printData();
}

void slowLeft() {
  int speed = 100;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading
  Serial.println("MOVING LEFT");
  while (speed <= 200 ) {
    updateHeading();
    right(motor1, motor2, speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed +=10;
    } 
  }
  while (speed > 0 ) {
    updateHeading();
    right(motor1, motor2, speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed -=20;
    } 
  }

  brake(motor1, motor2);
  // Calculate RPM for Motors and distance traveled 
  calculate_RPM(counter1, counter2, rpm1, rpm2, distanceTraveled);
  updatePosition(); // determines new posX and posY
  printData();
}

void slowRight() {
  Serial.println("MOVING RIGHT");
  int speed = 100;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading
  while (speed <= 200 ) {
    updateHeading();
    left(motor1, motor2, -speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed +=10;
    } 
  }
  while (speed > 0 ) {
    updateHeading();
    left(motor1, motor2, -speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      //delay(10);
      speed -=20;
    } 
  }

  brake(motor1, motor2);
  // Calculate RPM for Motors and distance traveled 
  calculate_RPM(counter1, counter2, rpm1, rpm2, distanceTraveled);
  updatePosition(); // determines new posX and posY
  printData();
}

void calculate_RPM(int counter1, int counter2, float &rpm1, float &rpm2, float &distanceTraveled) {
  rpm1 = ((float)counter1 / (float)slots) * (60000.0 / (float)interval);
  rpm2 = ((float)counter2 / (float)slots) * (60000.0 / (float)interval);

  // calculate distance traveled
  float wheelCircumference = 20.5; // measured in cm (could be slightly off)

  distanceTraveled = ( (float)wheelCircumference / (float)slots ) * ( ((float)counter1+(float)counter2)/2); // average both counters together
  Serial.print(wheelCircumference);
  Serial.print(" ");
  Serial.print(slots);
  Serial.print(" ");
  Serial.print(counter1);
  Serial.print(" ");
  Serial.println(counter2);
}

void calibrateGyroSensor() {
  int numReadings = 20;
  float sumGyroZ = 0.0;
  float gyroZValues[numReadings];

  for (int i = 0; i < numReadings; i++) {
    mpu_gyro->getEvent(&gyro);

    Serial.print("Calibrating gyro sensor, iteration #: ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(gyro.gyro.z);

    sumGyroZ += gyro.gyro.z;
    gyroZValues[i] = gyro.gyro.z;

    delay(10);
  }
  // Calculate mean
  meanGyroZ = sumGyroZ / numReadings;

  Serial.print("Mean GyroZ = ");
  Serial.println(meanGyroZ);

}

void handleCommand(char c) {
  switch(c) {
    case 'F':
      Serial.println("Moving Forward");
      slowForward();
      break;
    case 'B':
      slowBackward();
      break;
    case 'L':
      slowLeft();
      break;
    case 'R':
      slowRight();
      break;
    case 'C':
      calibrate_RPM();
      break;
    case 'X':
      acknowledge = 1; // enable next packet to transmit
      break;
  }
}

String prepareDataPacket() {
  String data_packet = "";
  for (int angle = 30; angle <= 150; angle += 5) {
    if (!client.connected()) {
      break; // Break the loop if client disconnects
    }
    unsigned int distance = sweepAndMeasure(angle); 
    data_packet += String(angle) + ", " + String(distance) + "\n";
  }
  for (int angle = 150; angle >= 30; angle -= 5) {
    if (!client.connected()) {
      break; // Break the loop if client disconnects
    }
    unsigned int distance = sweepAndMeasure(angle); 
    data_packet += String(angle) + ", " + String(distance) + "\n";
  }
  return data_packet;
}

unsigned int sweepAndMeasure(int angle) {
  myservo.write(angle);
  delay(20);
  return sonar.ping_cm();
}

void calibrate_RPM() {
  Serial.println("Calibrating wheels");
   // Reset counters
  counter1 = 0;
  counter2 = 0;

  int speed = 100;
  while (speed <= 200 ) {
    forward(motor1, motor2, speed);
    currentTime = millis();
    lastTime = currentTime;
    beginTime = millis();
    while (currentTime < beginTime + 10) {
      currentTime = millis();
      deltaTime = (currentTime - lastTime) / 1000.0;
      lastTime = currentTime;
      updateHeading();
      delay(10);
      speed +=10;
    } 
  }
  brake(motor1, motor2);

  // Calculate RPM for Motor 1
  float rpm1 = (counter1 / slots) * (60000.0 / interval);
  Serial.print("Motor 1 Speed: ");
  Serial.print(rpm1);
  Serial.println(" RPM");

  // Calculate RPM for Motor 2
  float rpm2 = (counter2 / slots) * (60000.0 / interval);
  Serial.print("Motor 2 Speed: ");
  Serial.print(rpm2);
  Serial.println(" RPM");

  // Reset counters
//  counter1 = 0;
//  counter2 = 0;
}

void updatePosition() {
    // Calculate the average heading in radians
    float averageHeadingRadians = ((heading_start + heading_final) / 2) * (PI / 180.0);

    // Calculate the change in position
    float deltaX = distanceTraveled * cos(averageHeadingRadians);
    float deltaY = distanceTraveled * sin(averageHeadingRadians);

    // Update the car's position
    posX += deltaX;
    posY += deltaY;

    // Print the new position for debugging
    Serial.print("New Position -> X: ");
    Serial.print(posX);
    Serial.print(", Y: ");
    Serial.println(posY);
}

void printData() {
  Serial.print("RPM1: ");
  Serial.print(rpm1);
  Serial.print(",  RPM2: ");
  Serial.print(rpm2);
  Serial.print(" Counter1 ");
  Serial.print(counter1);
  Serial.print(" Counter2 ");
  Serial.print(counter2);
  Serial.print(" distanceTraveled ");
  Serial.println(distanceTraveled);

  String rpm_packet = "RPM1 " + String(rpm1) + ", RPM2 " + String(rpm2) + ", Counter1 " + String(counter1) + ", Counter2 " + String(counter2) + ", distanceTraveled " + String(distanceTraveled) + ", posX " + String(posX) + ", posY " + String(posY) + "\n";
  client.print(rpm_packet);
}

