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
float heading_final = 0.0;

unsigned long currentTime, lastTime, beginTime=0;
float deltaTime = 0.0;

// Define the maximum number of data points
const int maxDataPoints = 200;

// Declare arrays to store data
int dataIndex = 0;

double gyro_X[maxDataPoints];
double gyro_Y[maxDataPoints];
double gyro_Z[maxDataPoints];
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

int acknowledge = 1; // flag used to control synch Transmit data with plot

void setup(void) {
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);
  myservo.write(90);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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


void printData() {
  for (int i = 0; i < dataIndex; i++) {
    Serial.print(i);
    Serial.print(" gyro_Z: ");
    Serial.print(gyro_Z[i]);
    Serial.print(" timeStamp ");
    Serial.println(timeStamps[i]);
  }
}


void updateHeading() {
  //timeStamps[dataIndex] = millis();
    mpu_gyro->getEvent(&gyro);
    // Integrate angular velocity to get the angle
    Serial.print("gyroZ (raw): " );
    Serial.print(gyro.gyro.z);

    GyroZ_calibrated = gyro.gyro.z - meanGyroZ;  // correct GyroZ using calibration mean
    Serial.print(", GyroZ calibrated: ");
    Serial.print(GyroZ_calibrated);

    Serial.print(", Active GyroZ: ");
    if (abs(GyroZ_calibrated) < 0.01) {  // if BELOW Threshold (0.01), ignore the gyro value
      heading_rad +=0;
      Serial.print("0 BELOW threshold");
    }
    else {
      heading_rad += (GyroZ_calibrated) * deltaTime;
      Serial.print(GyroZ_calibrated);
    }

    Serial.print(", heading_rad+: ");
    Serial.print(heading_rad);

    // Convert to degrees
    heading_deg = heading_rad*57.2958; // 1 radian = 57.2958
    Serial.print(", heading_deg: ");
    Serial.print(heading_deg);

    // Normalize to 0-360 degrees
    heading_final = fmod(heading_deg, 360);
    if (heading_final < 0) heading_final +=360;
    Serial.print(", heading_final = ");
    Serial.println(heading_final);

    //speed_array[dataIndex] =speed;
    //dataIndex +=1;
    String heading_location_packet = "Heading " + String(heading_final) + "," + " deg " + "\n";
    client.print(heading_location_packet);
}

void slowForward() {
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
}

void slowBackward() {
  int speed = 100;
  while (speed <= 200 ) {
    back(motor1, motor2, -speed);
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
}

void slowLeft() {
  int speed = 100;
  Serial.println("MOVING LEFT");
  while (speed <= 200 ) {
    right(motor1, motor2, speed);
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
}

void slowRight() {
  Serial.println("MOVING RIGHT");
  int speed = 100;
  while (speed <= 200 ) {
    left(motor1, motor2, -speed);
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
