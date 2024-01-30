#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>
#include <SPI.h>
#include <WiFiS3.h>
// Radar/Sonar setup
Servo myservo;
#define SERVO_PIN 10  // sets the servo control to pin 10

const int trigPin = 13;                // Trigger Pin connected to A13
const int echoPin = 12;                // Echo Pin connected to A12
const unsigned int maxDistance = 200;  // Maximum distance to measure

NewPing sonar(trigPin, echoPin, maxDistance);  // NewPing setup of pins and maximum distance.
unsigned int sweepAndMeasure(int angle) {
  myservo.write(angle);                     // radar to angle
  delay(20);                                // Short delay to allow the servo to reach the position
  unsigned int distance = sonar.ping_cm();  // Measure distance
  return distance;
}
//Wifi - Replace with your network credentials
const char* ssid = "WLAN-lumster";
const char* password = "401Cordoba";
WiFiServer server(80);
WiFiClient client;  // Declare WiFiClient object globally

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true)
      ;
  }

  delay(1000);
  Serial.print("====================\nConnecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  server.begin();

  delay(1000);
  Serial.println("Start python script");
}

int myTimeDelay = 0;
void handleClientInput() {
  if (client.available()) {
    char c = client.read();
    if (c == 'A') myTimeDelay = 86;
    if (c == 'F') myTimeDelay += 50;
    if (c == 'B') myTimeDelay -= 50;
    if (myTimeDelay < 0) myTimeDelay = 0;
    //Serial.print("time delay: ");
    //Serial.println(myTimeDelay);
  }
}

void loop() {
  client = server.available();  // Move this line to the beginning of the loop

  if (client) {
    Serial.println("Client connected");

    while (client.connected()) {
      for (int angle = 30; angle <= 150; angle += 5) {
        handleClientInput();
        if (myTimeDelay == 86) {
          client.stop();
          break;
        }
        unsigned int measuredDistance = sweepAndMeasure(angle);
        client.print(angle);
        client.print(", ");
        client.println(measuredDistance);
        delay(myTimeDelay);
      }

      for (int angle = 150; angle >= 30; angle -= 5) {
        handleClientInput();
        if (myTimeDelay == 86) {
          client.stop();
          break;
        }
        unsigned int measuredDistance = sweepAndMeasure(angle);
        client.print(angle);
        client.print(", ");
        client.println(measuredDistance);
        delay(myTimeDelay);
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
  unsigned int i = sweepAndMeasure(90);  //re-center
}
