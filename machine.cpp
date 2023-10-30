#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Define the pins that the motor driver, pressure valve, and switch are connected to
const int motorDriverPin1 = 32;
const int motorDriverPin2 = 33;
const int pressureValvePin = 18;
const int switchPin = 19;

// Define the pressure sensor calibration parameters
const float pressureSensorOffset = 100.0;
const float pressureSensorScaleFactor = 1.0;

// Define the Korotkoff algorithm parameters
const int korotkoffThreshold = 5; // The minimum difference in pressure sensor readings to be considered a Korotkoff sound
const int korotkoffCountThreshold = 5; // The number of Korotkoff sounds to detect

// Define the pump states
const int OFF = 0;
const int INFLATING = 1;
const int MEASURING = 2;
const int DEFLATING = 3;

// Create an instance of the Adafruit_MPU6050 class
Adafruit_MPU6050 mpu;

// Define the pins for the pressure sensor
const int pressureSensorOutputPin = A0; // Assuming the pressure sensor is connected to analog pin A0

// Define variables for pump and valve control
int pumpState = OFF;
int valveState = LOW; // Assume the valve is initially closed
unsigned long inflatingStartTime = 0;
unsigned long measuringStartTime = 0;

// Variables for pressure measurement
int systolicPressure = 0;
int diastolicPressure = 0;
int korotkoffCount = 0;
int previousPressure = 0;

void setup() {
  // Initialize the serial port
  Serial.begin(9600);

  // Initialize the Adafruit_MPU6050 class
  mpu.begin();

  // Set the motor driver pins to output mode
  pinMode(motorDriverPin1, OUTPUT);
  pinMode(motorDriverPin2, OUTPUT);

  // Set the pressure valve pin to output mode
  pinMode(pressureValvePin, OUTPUT);
  digitalWrite(pressureValvePin, valveState); // Set valve to initial state (closed)

  // Set the switch pin to input mode
  pinMode(switchPin, INPUT);
}

void loop() {
  // Read the switch state
  int switchState = digitalRead(switchPin);

  if (switchState == HIGH) {
    // If the switch is ON, start the blood pressure measurement cycle
    pumpState = INFLATING;
    inflatingStartTime = millis(); // Record the start time of inflation
    digitalWrite(motorDriverPin1, HIGH);
    digitalWrite(motorDriverPin2, LOW);
    valveState = HIGH; // Open the valve
  }

  // Implement different states of the blood pressure cycle
  switch (pumpState) {
    case INFLATING:
      // Control the pump to inflate the cuff (you may need to set a time or pressure threshold)
      if (millis() - inflatingStartTime > 5000) { // Inflate for 5 seconds as an example
        pumpState = MEASURING;
        measuringStartTime = millis(); // Record the start time of measurement
      }
      break;

    case MEASURING:
      // Continue measuring for a specific duration (e.g., 10 seconds)
      if (millis() - measuringStartTime > 10000) { // Measure for 10 seconds as an example
        pumpState = DEFLATING;
        valveState = LOW; // Close the valve to release pressure
      } else {
        // During the measurement phase, read the pressure sensor data
        int pressure = analogRead(pressureSensorOutputPin);
        // Calculate systolic and diastolic pressure using the Korotkoff algorithm
        if (abs(pressure - previousPressure) > korotkoffThreshold) {
          korotkoffCount++;
          if (korotkoffCount == 1) {
            systolicPressure = pressure;
          }
          if (korotkoffCount == korotkoffCountThreshold) {
            diastolicPressure = pressure;
          }
        }
        previousPressure = pressure;
      }
      break;

    case DEFLATING:
      // Control the pressure valve to release the pressure
      digitalWrite(pressureValvePin, valveState); // Open or close the valve based on valveState
      // The pressure valve is controlled here to release pressure
      // You may need to set a time duration for deflation or control it based on your mechanism
      // Consult your hardware documentation for the control mechanism
      break;

    default:
      // Pump is OFF, do nothing
      break;
  }

  // Display systolic and diastolic pressure readings
  if (pumpState == DEFLATING) {
    Serial.print("Systolic pressure: ");
    Serial.println(systolicPressure);
    Serial.print("Diastolic pressure: ");
    Serial.println(diastolicPressure);
  }

  // Add any additional code here if needed
}
