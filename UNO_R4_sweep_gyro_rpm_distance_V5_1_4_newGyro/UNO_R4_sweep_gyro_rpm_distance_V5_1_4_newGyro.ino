#include <WiFiS3.h>

// Network credentials
const char* ssid = "thisistheway";
const char* password = "eatpie69";

WiFiServer server(80);
WiFiClient client;

//*********************** SETUP FOR MPU6050 *****************************
// Demo for getting individual unified sensor data from the MPU6050 (from Adafruit examples UnifiedSensors)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_gyro;

sensors_event_t accel, gyro, temp;

//sensors_event_t gyro;

float meanGyroZ = 0.0;
float meanGyroX = 0.0;
float meanGyroY = 0.0;

float GyroZ_calibrated = 0.0;
float accelX_calibrated = 0.0;
float accelY_calibrated = 0.0;

// Define the maximum number of data points
const int maxDataPoints = 200;

// Declare arrays to store data
int dataIndex = 0;
int speed = 0;

String dataString;

double accel_X[maxDataPoints];
double accel_Y[maxDataPoints];
//double accel_Z[maxDataPoints];
//double gyro_X[maxDataPoints];
//double gyro_Y[maxDataPoints];
double gyro_Z[maxDataPoints];
int speed_array[maxDataPoints];
double timeStamps[maxDataPoints];
float heading_rad_accum[maxDataPoints];
float heading_deg_accum[maxDataPoints];



//float heading_rad = 1.5708;
float heading_rad = 0.0;
float heading_deg = 0.0;
float heading_start = 90.0;
float heading_final = 90.0;

static unsigned long heading_lastTime = 0;
unsigned long heading_currentTime = 0;
float heading_deltaTime = 0.0;

float posX, posY = 0.0;
float deltaX, deltaY = 0.0;

unsigned long currentTime, lastTime, beginTime=0;
float deltaTime = 0.0;

void updateHeading(); // Function prototype




// ***************** SETUP FOR THE MOTORS***************************
#include <SparkFun_TB6612.h>
// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 7 //2
#define BIN1 8 // 7
#define AIN2 9 // 4 no idea (9 works, but maybe this isn't used - check and delete later)
#define BIN2 4 // 10(causes servo to spin) // 8 no idea (10 works, but  maybe this isn't used - check and delete later)
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
  dataString.reserve(8000);
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

      if (client.available()) {
        char c = client.read();
        Serial.print("received : ");
        Serial.println(c);
        handleCommand(c);
      }
      if (acknowledge == 1) {
        //Serial.println("preparing a packet");
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
    //Serial.println("writing file");
    for (int i = 0; i < dataIndex; i++) {
      //dataString = "RAW: " + String(i) + ", s: " + String(speed_array[i]) + ", aX: " + String(accel_X[i]) + ", aY: " + String(accel_Y[i]) + ", aZ: " + String(accel_Z[i]) + ", gX: " + String(gyro_X[i]) + ", gY: " + String(gyro_Y[i]) + ", gZ: " + String(gyro_Z[i]) + ", timeS: " + String(timeStamps[i]) + "\n";
      dataString = "RAW: " + String(i) + ", s: " + String(speed_array[i]) + ", aX: " + String(accel_X[i]) + ", aY: " + String(accel_Y[i]) + ", gZ: " + String(gyro_Z[i]) + ", timeS: " + String(timeStamps[i]) + ", heading_rad: " + String(heading_rad_accum[i]) + ", heading_deg: " + String(heading_deg_accum[i]) + "\n";
      client.print(dataString);
      client.flush();
      delay(10); // Small delay to ensure data is sent and processed
    }
  }
}

void updateHeading() {
    //mpu_gyro->getEvent(&gyro);
    mpu.getEvent(&accel, &gyro, &temp);
    speed_array[dataIndex] = speed;
    accel_X[dataIndex] = accel.acceleration.x;
    accel_Y[dataIndex] = accel.acceleration.y;
    //accel_Z[dataIndex] = accel.acceleration.z;
    //gyro_X[dataIndex] = gyro.gyro.x;
    //gyro_Y[dataIndex] = gyro.gyro.y;
    gyro_Z[dataIndex] = gyro.gyro.z;
    timeStamps[dataIndex] = millis();
    

    heading_currentTime = millis();
    heading_deltaTime = ((float)heading_currentTime - (float)heading_lastTime) / 1000.0;
    heading_lastTime = heading_currentTime;

    // Integrate angular velocity to get the angle

    GyroZ_calibrated = gyro.gyro.z - meanGyroZ;  // correct GyroZ using calibration mean (instantaneous heading)
 
    float gZ_threshold = 0.03; // originally was set to 0.01

    if (abs(GyroZ_calibrated) < gZ_threshold) {  // if BELOW Threshold (0.01), ignore the gyro value
      GyroZ_calibrated = 0;
    }
    
    heading_rad += (GyroZ_calibrated) * heading_deltaTime;

    heading_rad_accum[dataIndex] = heading_rad;
    heading_deg_accum[dataIndex] = heading_rad*57.2958;

    dataIndex +=1;

}

void slowForward() {
  speed = 50;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading

  String heading_debug = "Heading (BEFORE) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);

  Serial.print("heading_rad(BEFORE) ");
  Serial.print(heading_rad);
  heading_rad = 0.0;

  heading_debug = "Heading (AFTER) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);

  Serial.print("heading_rad(AFTER) ");
  Serial.println(heading_rad);
  Serial.println("Slow FORWARD");
  dataIndex = 0;
  currentTime = millis();
  beginTime = currentTime;
  updateHeading();
  forward(motor1, motor2, speed);

 while (currentTime <= beginTime + 1000 ) {
    updateHeading();
    currentTime = millis();
    delay(10);
  }
  updateHeading();
  brake(motor1, motor2);
  for (int i=1; i<5; i++){
    updateHeading();
    delay(10);
  }

  if(1){
    if (counter1 == counter2) {
      // Don't adjust heading, already correct
      heading_final = heading_start;
    } else {
      // Adjust heading based on gyroZ, Finalize Heading, Convert to degrees
      heading_deg = heading_rad*57.2958; // 1 radian = 57.2958

      // Normalize to 0-360 degrees
      heading_final = heading_start + heading_deg;
      heading_final = fmod(heading_final, 360);
      if (heading_final < 0) heading_final +=360;
    }
    String heading_location_packet = "Heading " + String(heading_final) + "," + " deg " + "\n";
    client.print(heading_location_packet);
  }

 // Calculate RPM for Motors and distance traveled 
  delay(50); // give interrupts time to settle to final value
  calculate_RPM(counter1, counter2, rpm1, rpm2, distanceTraveled);
  updatePosition(1); // determines new posX and posY
  printData_Wifi();
  printData();
}

void slowBackward() {
  speed = 50;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading
  
  String heading_debug = "Heading (BEFORE) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);

  Serial.print("heading_rad(BEFORE) ");
  Serial.print(heading_rad);
  heading_rad = 0.0;
  
  heading_debug = "Heading (AFTER) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);
  
  Serial.print("heading_rad(AFTER) ");
  Serial.println(heading_rad);
  Serial.println("Slow BACK");
  dataIndex = 0;
  currentTime = millis();
  beginTime = currentTime;
  updateHeading();
  back(motor1, motor2, speed);

  while (currentTime <= beginTime + 1000 ) {
    updateHeading();
    currentTime = millis();
    delay(10);
  }
  updateHeading();
  brake(motor1, motor2);
  for (int i=1; i<5; i++){
    updateHeading();
    delay(10);
  }

 if(1){
    if (counter1 == counter2) {
      // Don't adjust heading, already correct
      heading_final = heading_start;
    } else {
      // Adjust heading based on gyroZ, Finalize Heading, Convert to degrees
      heading_deg = heading_rad*57.2958; // 1 radian = 57.2958

      // Normalize to 0-360 degrees
      heading_final = heading_start + heading_deg;
      heading_final = fmod(heading_final, 360);
      if (heading_final < 0) heading_final +=360;
    }
    String heading_location_packet = "Heading " + String(heading_final) + "," + " deg " + "\n";
    client.print(heading_location_packet);
  }

  // Calculate RPM for Motors and distance traveled 
  delay(50); // give interrupts time to settle to final values
  calculate_RPM(counter1, counter2, rpm1, rpm2, distanceTraveled);
  updatePosition(0); // determines new posX and posY
  printData_Wifi();
  printData();
}


void slowRightHead() {
  speed = 50;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading
  
  String heading_debug = "Heading (BEFORE) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);
  
  Serial.print("heading_rad(BEFORE) ");
  Serial.print(heading_rad);
  heading_rad = 0.0;
  
  heading_debug = "Heading (AFTER) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);
  
  Serial.print("heading_rad(AFTER) ");
  Serial.println(heading_rad);
  Serial.println("Turning RIGHT(Head)");
  dataIndex = 0;

  // -1.57
  while (heading_rad > -1.54) 
  {
    if (dataIndex < maxDataPoints-5) {
      updateHeading();
      left(motor1, motor2, speed); // Assuming right() is a function that correctly drives both motors
      delay(10);
      if (speed < 100) {
        speed += 5;
      }
    } else {
      Serial.println("Exceeded maxDataPonts");
      break;
    }
  }
  updateHeading();
  brake(motor1, motor2);
  for (int i = 1; i<5; i++) {
    updateHeading();
    delay(10);
  }

  // Finalize Heading, Convert to degrees
  heading_deg = heading_rad*57.2958; // 1 radian = 57.2958
  
  // Normalize to 0-360 degrees
  heading_final = heading_start + heading_deg;
  //heading_final -= 90;
  heading_final = fmod(heading_final, 360);
  if (heading_final < 0) heading_final +=360;

  printData_Wifi();
  String heading_location_packet = "Heading " + String(heading_final) + "," + " deg " + "\n";
  client.print(heading_location_packet);
}

void slowLeftHead() {
  speed = 50;
  counter1 = 0;
  counter2 = 0;
  heading_start = heading_final;  // set the heading to the previous heading

  String heading_debug = "Heading (BEFORE) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);

  Serial.print("heading_rad(BEFORE) ");
  Serial.print(heading_rad);
  heading_rad = 0.0;
  
  heading_debug = "Heading (AFTER) " + String(heading_rad) + "," + " rad " + "\n";
  client.print(heading_debug);
  
  Serial.print("heading_rad(AFTER) ");
  Serial.println(heading_rad);
  Serial.println("Turning LEFT (Head)");
  dataIndex = 0;
  
  // (was 1.57, but due to hysteresis in "brake" command, tell Batsie to stop slightly earlier than 90 degrees - this can be improved with PID later)
  while (heading_rad < 1.54) 
  {
    if (dataIndex < maxDataPoints-5) {
      updateHeading();
      right(motor1, motor2, speed);
      delay(10);
      if (speed < 100) {
        speed += 5;
      }
    } else {
      Serial.println("Exceeded maxDataPoints");
      break;
    }
  }
  Serial.print("Heading Accumulated ");
  Serial.println(heading_rad);
  delay(10);
  updateHeading();
  brake(motor1, motor2);
  for (int i = 1; i<5; i++){
    updateHeading();
    delay(10);
  }

  // Finalize Heading, Convert to degrees
  heading_deg = heading_rad*57.2958; // 1 radian = 57.2958
  
  // Normalize to 0-360 degrees
  heading_final = heading_start + heading_deg;
  //heading_final += 90;
  heading_final = fmod(heading_final, 360);
  if (heading_final < 0) heading_final +=360;

  printData_Wifi();
  String heading_location_packet = "Heading " + String(heading_final) + "," + " deg " + "\n";
  client.print(heading_location_packet);
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
      slowLeftHead();
      break;
    case 'R':
      slowRightHead();
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


void updatePosition(bool movingForward) {
    // Calculate the average heading in radians
    float averageHeadingRadians = ((heading_start + heading_final) / 2) * (PI / 180.0);

    // Try fusing
    // L (track width) = 12.62 cm
    // C = 20.5 cm, Num_slots = 20, dcount = C/Num_slots, -> dcount = 1.025 cm
    // dL = Counter1 (left), dR = Counter2 (right)
    // ex: dL 18*dcount = 18.45 cm, dR = 17*dcount = 17.425
    // deltaD = dL - dR = 1.025 cm
    // delta_theta =(approx) = deltaD / L -> = 1.025/12.62 = 0.0812 radians
    // heading_delta_degr = 0.0812 * (180/3.1415) = 4.65 degrees

    float L = 12.62;

    if(0) {
      float heading_delta_deg = (1.025*(float(counter1)-float(counter2))/L) * 180/3.1415;

      // Normalize to 0-360 degrees
      heading_final = heading_start + heading_delta_deg;
      heading_final = fmod(heading_final, 360);
      if (heading_final < 0) heading_final +=360;

      String heading_location_packet = "Heading " + String(heading_final) + "," + " deg " + "\n";
      client.print(heading_location_packet);

      // convert start and final heading to radians
      float heading_start_rad = heading_start * (PI / 180.0);
      float heading_final_rad = heading_final * (PI / 180.0);

      // Calculate vector components of each heading
      float x_start = cos(heading_start_rad);
      float y_start = sin(heading_start_rad);
      float x_final = cos(heading_final_rad);
      float y_final = sin(heading_final_rad);

      // calculate the average vector components
      float x_avg = (x_start + x_final) / 2;
      float y_avg = (y_start + y_final) / 2; 

      // Calculate average heading from average vector components
      float averageHeadingRadians = atan2(y_avg, x_avg);

      // Convert average heading to degrees
      float averageHeadingDegrees = averageHeadingRadians * 180.0/PI;

      // Calculate the average heading in radians
      //float averageHeadingRadians = ((heading_start + heading_final) / 2) * (PI / 180.0);  // old and not used anymore
    }

    // Convert average heading to degrees
    float averageHeadingDegrees = averageHeadingRadians * 180.0/PI;
    averageHeadingDegrees = fmod(averageHeadingDegrees, 360);
    if (averageHeadingDegrees < 0) averageHeadingDegrees +=360;


    Serial.print("counter1 ");
    Serial.print(counter1);
    Serial.print(" counter2 ");
    Serial.print(counter2);

    Serial.print(" posX_b4 ");
    Serial.print(deltaX);
    Serial.print(" posY_b4 ");
    Serial.print(deltaY);

    // Calculate the change in position
    deltaX = distanceTraveled * cos(averageHeadingRadians);
    deltaY = distanceTraveled * sin(averageHeadingRadians);

    // Update the car's position based on if moving forwards or backwards
    if (movingForward) {
      posX += deltaX;
      posY += deltaY;
    } else {
      // if moving backwards, reverse the direction of the update
      posX -= deltaX;
      posY -= deltaY;
    }

    // Print the new position for debugging
    Serial.print(" Heading Start: ");
    Serial.print(heading_start);
    //Serial.print(" Heading_delta_deg ");
    //Serial.print(heading_delta_deg);
    Serial.print(" Heading Final: ");
    Serial.print(heading_final);
    Serial.print(" Average Heading (Deg): ");
    Serial.print(averageHeadingDegrees);
    Serial.print(" New Position -> X: ");
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


void calibrateGyroSensor() {
  int numReadings = 50;
  float sumGyroZ = 0.0;
  float sumGyroX = 0.0;
  float sumGyroY = 0.0;

  float gyroZValues[numReadings];
  float gyroXValues[numReadings];
  float gyroYValues[numReadings];

  for (int i = 0; i < numReadings; i++) {
    mpu_gyro->getEvent(&gyro);
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

void calculate_RPM(int counter1, int counter2, float &rpm1, float &rpm2, float &distanceTraveled) {
  rpm1 = ((float)counter1 / (float)slots) * (60000.0 / (float)interval);
  rpm2 = ((float)counter2 / (float)slots) * (60000.0 / (float)interval);

  // calculate distance traveled
  float wheelCircumference = 20.5; // measured in cm (could be slightly off)

  distanceTraveled = ( (float)wheelCircumference / (float)slots ) * ( ((float)counter1+(float)counter2)/2); // average both counters together
  Serial.print("Wheel Cicumference ");
  Serial.print(wheelCircumference);
  Serial.print(" slots ");
  Serial.print(slots);
  Serial.print(" counter1 ");
  Serial.print(counter1);
  Serial.print(" counter2 ");
  Serial.print(counter2);
  Serial.print(" distanceTraveled ");
  Serial.println(distanceTraveled);
}


