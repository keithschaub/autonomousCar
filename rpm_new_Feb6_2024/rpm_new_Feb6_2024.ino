const byte MOTOR1 = 2; // Motor 1 interrupt pin
const byte MOTOR2 = 11; // Motor 2 interrupt pin

volatile unsigned int counter1 = 0;
volatile unsigned int counter2 = 0;

unsigned long previousMillis = 0;
const long interval = 1000;  // Interval at which to measure (1 second)

float slots = 20.00;  // Number of slots in the encoder

void ISR_count1() {
  counter1++;
}

void ISR_count2() {
  counter2++;
}

void setup() {
  Serial.begin(9600);

  // Attach the interrupt for Motor encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR1), ISR_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2), ISR_count2, RISING);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

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
    counter1 = 0;
    counter2 = 0;
  }
}
