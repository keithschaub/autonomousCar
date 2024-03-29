//*********************** SETUP FOR MPU6050 *****************************
// Demo for getting individual unified sensor data from the MPU6050 (from Adafruit examples UnifiedSensors)
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

String accelDataString = "";
int count = 0;
unsigned long beginTime = 0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
float deltaT = 0.0;
int printOnce = 1;
double velocity = 0.0;
double totalDistance = 0.0;
sensors_event_t accel;
double accelYOffset = 0.0;
double accelZOffset = 0.0;

double acceleration_previous_y = 0.0;
double velocity_previous_y = 0.0;
double position_previous_y = 0.0;

double acceleration_current_y = 0.0;
double velocity_current_y = 0.0;
double position_current_y = 0.0;

double stdDevAccelY = 0.0;
double sumTime=0.0;


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


void setup(void) {
  accelDataString.reserve(32000);
  Serial.begin(115200);
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
  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();

  Serial.println("Begin Calibration");
  calibrateSensors();
  Serial.println("Calibration Complete");

  Serial.println("pausing 1 second");
  delay(1000);
  beginTime = millis();
  count = 0;

}



void loop() {
  
  currentTime = millis();

  float alpha = 0.5;
  float filteredAccelY = 0;

  if (count == 0) {
    forward(motor1, motor2, 200);
  }
  while (currentTime < beginTime + 500) {
    //sensors_event_t accel;
    mpu_accel->getEvent(&accel);

    float accel_calibrated_y = accel.acceleration.y - accelYOffset;  // this will add ~0.16 to the read value
    float accel_calibrated_z = accel.acceleration.z - accelZOffset; // this will subtract ~0.19

  
    //***************************** Filter acceleration values ************************************************
    // Applylow pass filter
    filteredAccelY = alpha * accel_calibrated_y + (1 - alpha) * filteredAccelY;

    acceleration_current_y = filteredAccelY;

    velocity += acceleration_current_y;
        

    if (count >=1 && currentTime > previousTime) {
      deltaT = (currentTime - previousTime)/1000.0; // seconds
      sumTime += deltaT;
      
      velocity_current_y = velocity_previous_y + 100*(((acceleration_current_y + acceleration_previous_y)/2 ) * deltaT);  // cm/s
      position_current_y = position_previous_y + ( ( (velocity_current_y + velocity_previous_y)/2 ) * deltaT);

      accelDataString += String(count) + ", accel_prev_y= " + String(acceleration_previous_y) + ", accel_current_y= " + String(acceleration_current_y) + ", vel_previous_y= " + String(velocity_previous_y) + ", vel_current_y= " + String(velocity_current_y) + ", pos_previous_y= " + String(position_previous_y) + ", pos_current_y= " + String(position_current_y) + ", " + String(previousTime) + ", " + String(currentTime) + ", " + String(deltaT, 5) + ", " + String(sumTime) + "\n";

      acceleration_previous_y = acceleration_current_y;
      velocity_previous_y = velocity_current_y;
      position_previous_y = position_current_y;

      
    }
    //accelDataString += String(count) + ", " + String(accel_calibrated_y) + ", " + String(velocity) + ", " + String(accel_calibrated_z) + ", " + String(currentAccelTime) + "\n";
    count +=1;

    /* Display the results (acceleration is measured in m/s^2) */
    //Serial.print(" \tY: ");
    //Serial.print(accel.acceleration.y);
    //Serial.print(" \tZ: ");
    //Serial.print(accel.acceleration.z);

    if (count > 50) {
      brake(motor1, motor2); //stop
    }

    delay(2);
    previousTime = currentTime;
    currentTime = millis();
  }

  brake(motor1, motor2); //stop
  if ( printOnce == 1) {
    //accelDataString = String(count) + ", accel_prev_y= " + String(acceleration_previous_y) + ", accel_current_y= " + String(acceleration_current_y) + ", vel_previous_y= " + String(velocity_previous_y) + ", vel_current_y= " + String(velocity_current_y) + ", pos_previous_y= " + String(position_previous_y) + ", pos_current_y= " + String(position_current_y) + ", " + String(previousTime) + ", " + String(currentTime) + ", " + String(deltaT, 5) + ", " + String(sumTime) + "\n";
    Serial.println(accelDataString);
    printOnce = 0;
    }
}


void calibrateSensors() {
  float sumAccelY = 0.0, sumAccelZ = 0.0;
  float accelYValues[20]; // Array to store individual accelY values
  int numReadings = 20;

  // First pass: Calculate sum and store individual values
  for (int i = 0; i < numReadings; i++) {
    mpu_accel->getEvent(&accel);
    
    Serial.print("Calibrating sensors, iteration #: ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(accel.acceleration.y);
    
    sumAccelY += accel.acceleration.y;
    accelYValues[i] = accel.acceleration.y; // Store individual value
    sumAccelZ += accel.acceleration.z;

    delay(10);
  }

  // Calculate mean (average)
  float meanAccelY = sumAccelY / numReadings;

  // Second pass: Calculate the squared differences from the mean
  float sumSquaredDiff = 0.0;
  for (int i = 0; i < numReadings; i++) {
    sumSquaredDiff += pow(accelYValues[i] - meanAccelY, 2);
  }

  // Calculate standard deviation
  stdDevAccelY = sqrt(sumSquaredDiff / numReadings);

  Serial.print("Mean AccelY = ");
  Serial.println(meanAccelY);
  Serial.print("Standard Deviation AccelY = ");
  Serial.println(stdDevAccelY);

  // Continue with your existing calculations
  accelYOffset = meanAccelY;
  accelZOffset = (sumAccelZ / numReadings) - 9.8;
  Serial.print("accelYOffset = ");
  Serial.println(accelYOffset);
  Serial.print("accelZOffset = ");
  Serial.println(accelZOffset);
}

