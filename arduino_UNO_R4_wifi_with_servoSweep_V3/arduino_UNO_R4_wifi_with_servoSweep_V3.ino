#include <SPI.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <NewPing.h>
#include <SparkFun_TB6612.h>

// Network credentials
const char* ssid = "thisistheway";
const char* password = "eatpie69";

WiFiServer server(80);
WiFiClient client;


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
#define AIN2 9
#define BIN2 10
#define PWMA 5
#define PWMB 6
#define STBY 3
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int acknowledge = 1; // OK to Transmit

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  myservo.write(90);

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
