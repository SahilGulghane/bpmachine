#include <Arduino.h>

// Define the pins for the switch, air pump, pressure valve, and pressure sensor
const int switchPin = 25;
const int airPumpPin1 = 33;
const int airPumpPin2 = 32;
const int pressureValvePin = 18;
const int pressureSensorClockPin = 35;
const int pressureSensorOutputPin = 34;

// Define the calibration constants for the pressure sensor
const float pressureSensorCalibrationOffset = 0.0f;
const float pressureSensorCalibrationFactor = 1.0f;

// Initialize the air pump and pressure sensor
void setup() {
  Serial.begin(115200);

  pinMode(switchPin, INPUT);
  pinMode(airPumpPin1, OUTPUT);
  pinMode(airPumpPin2, OUTPUT);
  pinMode(pressureValvePin, OUTPUT);
  pinMode(pressureSensorClockPin, INPUT);
  pinMode(pressureSensorOutputPin, INPUT);
}

// Main loop
void loop() {
  // Check if the switch is turned on
  if (digitalRead(switchPin) == HIGH) {
    // Turn on the air pump
    digitalWrite(airPumpPin1, HIGH);
    digitalWrite(airPumpPin2, HIGH);

    // Inflate the cuff to a certain pressure
    delay(10000); // Inflate the cuff for 10 seconds

    // Read the pressure sensor to get the systolic pressure
    int systolicPressure = readPressureSensor();

    // Open the pressure valve to deflate the cuff
    digitalWrite(pressureValvePin, HIGH);

    // Deflate the cuff
    delay(5000); // Deflate the cuff for 5 seconds

    // Read the pressure sensor again to get the diastolic pressure
    int diastolicPressure = readPressureSensor();

    // Display the systolic and diastolic pressures on the LCD display
    Serial.println("Systolic pressure: " + String(systolicPressure));
    Serial.println("Diastolic pressure: " + String(diastolicPressure));

    // Turn off the air pump
    digitalWrite(airPumpPin1, LOW);
    digitalWrite(airPumpPin2, LOW);

    // Close the pressure valve
    digitalWrite(pressureValvePin, LOW);
  }
}

// Reads the pressure sensor and returns the pressure in mmHg
int readPressureSensor() {
  // Read the pressure sensor output pin
  int pressureSensorOutput = analogRead(pressureSensorOutputPin);

  // Convert the pressure sensor output to a pressure in mmHg
  float pressure = (pressureSensorOutput - pressureSensorCalibrationOffset) * pressureSensorCalibrationFactor;

  // Return the pressure
  return pressure;
}
