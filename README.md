This is a DYI work in progress by Keith Schaub and Anthony Lum.
The objective is for a fully autonmous car to drive and create a map of its environment
Arduino UNO R4 WiFi module is the brains of the car.
Toshiba TB6612FNG is the driver IC for dual DC motor control.
MP6050 integrated gyro and accelerometer will be used for steering and direction
Ultrasonic sensor attached to low cost servo sweeps the forward direction of the car.
Data from sensor is streamed via wifi to windows PC
Python script residing on PC analyzes data and generates environment map.
ESP32-WROVER, with integrated camera, not currently used, but can be integrated easily at a later stage.
