#include <SPI.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <NewPing.h>
#include <SparkFun_TB6612.h>
#include <Wire.h> // for MPU6050

// Network credentials
const char* ssid = "thisistheway";
const char* password = "eatpie69";

WiFiServer server(80);
WiFiClient client;  // global declaration of client

// MPU6050
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

// Calibration variables
long accelXOffset, accelYOffset, accelZOffset;
long gyroXOffset, gyroYOffset, gyroZOffset;

float heading = 90;

// Servo
Servo myservo;
#define SERVO_PIN 10

// Ultrasonic Sensor
const int trigPin = 13;
const int echoPin = 12;
const unsigned int maxDistance = 200;
NewPing sonar(trigPin, echoPin, maxDistance);

// Motor
#define AIN1 7
#define BIN1 8
#define AIN2 9  // not used?
#define BIN2 10 // not used?
#define PWMA 5
#define PWMB 6
#define STBY 3
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int acknowledge = 1; // flag used to control synch Transmit data with plot

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  myservo.write(90);

  Wire.begin();  // starts I2C 
  setupMPU();
  Serial.println("setupMPU successful");
  Serial.println("Starting Calibration of MPU6050");
  calibrateSensors();
  Serial.println("Cal complete");

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
}

void loop() {
  client = server.available();
  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      // MPU6050
      recordAccelRegisters();
      recordGyroRegisters();
      updateHeading();
      printData();
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


void handleCommand(char c) {
  switch(c) {
    case 'F':
      moveForward();
      break;
    case 'B':
      moveBackward();
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'X':
      acknowledge = 1; // enable next packet to transmit
      break;
  }
}

unsigned int sweepAndMeasure(int angle) {
  myservo.write(angle);
  delay(20);
  return sonar.ping_cm();
}

void moveForward() {
    forward(motor1, motor2, 200);
    delay(200);
    brake(motor1, motor2);
}

void moveBackward() {
    back(motor1, motor2, -200);
    delay(200);
    brake(motor1, motor2);
}

void turnLeft() {
    right(motor1, motor2, 200); 
    delay(200);
    brake(motor1, motor2);
}

void turnRight() {
    left(motor1, motor2, -200); 
    delay(200);
    brake(motor1, motor2);
}

// For MPU6050 (basis taken from Examples MPU6050)
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6); {
    accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
    accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
    accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
    processAccelData();
  }
  
}

void processAccelData(){
  //gForceX = accelX / 16384.0;
  //gForceY = accelY / 16384.0; 
  //gForceZ = accelZ / 16384.0;

  // calibrated accelerometer
  gForceX = (accelX - accelXOffset) / 16384.0;
  gForceY = (accelY - accelYOffset) / 16384.0; 
  gForceZ = (accelZ - accelZOffset) / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into gyroX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into gyroY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into gyroZ
  processGyroData();
}

void processGyroData() {
  //rotX = gyroX / 131.0;
  //rotY = gyroY / 131.0; 
  //rotZ = gyroZ / 131.0;

  // calibrated gyro
  float noise = 0.5;
  rotX = (gyroX - gyroXOffset) / 131.0;
  if (abs(rotX) < noise) {
    rotX = 0;
  }

  rotY = (gyroY - gyroYOffset) / 131.0; 
  if (abs(rotY) < noise) {
    rotY = 0;
  }

  rotZ = (gyroZ - gyroZOffset) / 131.0;
  if (abs(rotZ) < noise) {
    rotZ = 0;
  }
  if (rotZ < -250) { // centers z point
    rotZ += 500;
  }
}

void printData() {
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print(" deg ");
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);

  String heading_location_packet = "Heading " + String(heading) + "," + " deg " + "\n";
 // Serial.println(heading_location_packet);
  client.print(heading_location_packet);
}

void calibrateSensors() {
  long sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  int numReadings = 20;

  for (int i = 0; i < numReadings; i++) {
    Serial.print("Calibrating sensors, iteration #: ");
    Serial.println(i);
    recordAccelRegisters();
    recordGyroRegisters();

    sumAccelX += accelX;
    sumAccelY += accelY;
    sumAccelZ += accelZ;
    sumGyroX += gyroX;
    sumGyroY += gyroY;
    sumGyroZ += gyroZ;
    //sumGyroZ = 0;

    delay(10);
  }

  accelXOffset = sumAccelX / numReadings;
  accelYOffset = sumAccelY / numReadings;
  accelZOffset = ( sumAccelZ / numReadings ) - (16384 * (accelZ > 0 ? 1 : -1));
  gyroXOffset = sumGyroX / numReadings;
  gyroYOffset = sumGyroY / numReadings;
  gyroZOffset = sumGyroZ / numReadings;
}

void updateHeading() {
  float gyroZrate = rotZ; // Invert the sign for clockwise/counter-clockwise logic
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
  lastTime = currentTime;

  // Update heading
  heading += gyroZrate * deltaTime;

  // Normalize heading to [0,360) degrees
  heading = fmod(heading, 360);
  if (heading < 0) heading += 360;
}


