#include <SoftwareSerial.h>
const int BT_RX = 0; // Connect to TX of Bluetooth Module
const int BT_TX = 1; // Connect to RX of Bluetooth Module

// Constants
SoftwareSerial BTSerial(BT_RX, BT_TX); // RX, TX


#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

// Calibration variables
long accelXOffset, accelYOffset, accelZOffset;
long gyroXOffset, gyroYOffset, gyroZOffset;

float heading = 90;

void setup() {
  BTSerial.begin(9600);
  Wire.begin();
  BTSerial.println("wire.begin successful");
  setupMPU();
  BTSerial.println("setupMPU successful");
  BTSerial.println("Starting calibration");
  calibrateSensors();
  BTSerial.println("Calibration complete");
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  updateHeading();
  printData();
  delay(100);
}

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
  rotX = (gyroX - gyroXOffset) / 131.0;
  if (abs(rotX) < 0.3) {
    rotX = 0;
  }

  rotY = (gyroY - gyroYOffset) / 131.0; 
  if (abs(rotY) < 0.3) {
    rotY = 0;
  }

  rotZ = (gyroZ - gyroZOffset) / 131.0;
  if (abs(rotZ) < 0.3) {
    rotZ = 0;
  }
  if (rotZ < -250) {
    rotZ += 500;
  }
}

void printData() {
  BTSerial.print("Heading: ");
  BTSerial.print(heading);
  BTSerial.print(" deg ");
  BTSerial.print("Gyro (deg)");
  BTSerial.print(" X=");
  BTSerial.print(rotX);
  BTSerial.print(" Y=");
  BTSerial.print(rotY);
  BTSerial.print(" Z=");
  BTSerial.print(rotZ);
  BTSerial.print(" Accel (g)");
  BTSerial.print(" X=");
  BTSerial.print(gForceX);
  BTSerial.print(" Y=");
  BTSerial.print(gForceY);
  BTSerial.print(" Z=");
  BTSerial.println(gForceZ);
}

void calibrateSensors() {
  long sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  int numReadings = 200;

  for (int i = 0; i < numReadings; i++) {
    BTSerial.print("Calibrating sensors, iteration #: ");
    BTSerial.println(i);
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

