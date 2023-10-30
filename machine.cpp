#include <Q2HX711.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Define the pins that the pressure sensor, motor driver, pressure valve, and switch are connected to
const int pressureSensorClockPin = 35;
const int pressureSensorOutputPin = 34;
const int motorDriverPin1 = 32;
const int motorDriverPin2 = 33;
const int pressureValvePin = 18;
const int switchPin = 19;

// Define the pressure sensor calibration parameters
const float pressureSensorOffset = 100.0;
const float pressureSensorScaleFactor = 1.0;

// Define the Korotkoff algorithm parameters
const int korotkoffThreshold = 5; // The minimum difference in pressure sensor readings to be considered a Korotkoff sound
const int korotkoffCountMax = 5; // The maximum number of Korotkoff sounds to detect

// Create an instance of the Q2HX711 class
Q2HX711 MPS20N0040D(pressureSensorOutputPin, pressureSensorClockPin);

// Define the pressure valve state
const int pressureValveOpen = HIGH;
const int pressureValveClosed = LOW;

void setup() {
  // Initialize the serial port
  Serial.begin(9600);

  // Initialize the Q2HX711 class
  MPS20N0040D.begin();

  // Set the motor driver pins to output mode
  pinMode(motorDriverPin1, OUTPUT);
  pinMode(motorDriverPin2, OUTPUT);

  // Set the pressure valve pin to output mode
  pinMode(pressureValvePin, OUTPUT);

  // Set the switch pin to input mode
  pinMode(switchPin, INPUT);
}

void loop() {
  // Read the switch state
  int switchState = digitalRead(switchPin);

  if (switchState == HIGH) {
    // Turn on the pump and open the pressure valve
    digitalWrite(motorDriverPin1, HIGH);
    digitalWrite(motorDriverPin2, LOW);
    digitalWrite(pressureValvePin, pressureValveOpen);

    // Measure blood pressure
    measureBloodPressure();
  } else {
    // Turn off the pump and close the pressure valve
    digitalWrite(motorDriverPin1, LOW);
    digitalWrite(motorDriverPin2, HIGH);
    digitalWrite(pressureValvePin, pressureValveClosed);

    // Add a delay to avoid rapid changes
    delay(1000);
  }
}

void measureBloodPressure() {
  // Read the pressure sensor data
  float pressure = MPS20N0040D.read();

  // Check for a valid pressure reading
  if (pressure >= 0.0 && pressure <= 1000.0) {
    // Calculate systolic and diastolic blood pressure using the Korotkoff algorithm
    int systolicPressure = 0;
    int diastolicPressure = 0;
    int korotkoffCount = 0;
    float previousPressure = 0.0;

    while (pressure > korotkoffThreshold && korotkoffCount < korotkoffCountMax) {
      if (pressure < systolicPressure) {
        systolicPressure = pressure;
      }

      if (pressure - previousPressure > korotkoffThreshold) {
        korotkoffCount++;
      }

      previousPressure = pressure;

      // Read the pressure sensor data
      pressure = MPS20N0040D.read();
    }

    // Display the systolic and diastolic blood pressure readings on the serial port
    Serial.print("Systolic pressure: ");
    Serial.println(systolicPressure);
    Serial.print("Diastolic pressure: ");
    Serial.println(diastolicPressure);
  }
}
