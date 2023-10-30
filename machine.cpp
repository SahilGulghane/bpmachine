#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Define the pins for the motor driver, pressure valve, and switch
const int motorDriverPin1 = 32;
const int motorDriverPin2 = 33;
const int pressureValvePin = 18;
const int switchPin = 19;

// Define the pressure sensor calibration parameters
const float pressureSensorOffset = 100.0;
const float pressureSensorScaleFactor = 1.0;

// Define the Korotkoff algorithm parameters
const int korotkoffThreshold = 5; // Minimum difference for Korotkoff sound
const int korotkoffCountThreshold = 5; // Number of Korotkoff sounds to detect

// Pump and valve states
const int OFF = 0;
const int INFLATING = 1;
const int MEASURING = 2;
const int DEFLATING = 3;

// Create an instance of the Adafruit_MPU6050 class
Adafruit_MPU6050 mpu;

// Pressure sensor pin
const int pressureSensorOutputPin = A0; // Assuming the pressure sensor is connected to analog pin A0

int pumpState = OFF;
int valveState = LOW;
unsigned long inflatingStartTime = 0;
unsigned long measuringStartTime = 0;

int systolicPressure = 0;
int diastolicPressure = 0;
int korotkoffCount = 0;
int previousPressure = 0;

bool korotkoffSoundDetected = false;

void setup() {
  // Initialize the serial port
  Serial.begin(9600);

  // Initialize the Adafruit_MPU6050 class
  mpu.begin();

  // Set pins to the proper mode
  pinMode(motorDriverPin1, OUTPUT);
  pinMode(motorDriverPin2, OUTPUT);
  pinMode(pressureValvePin, OUTPUT);
  pinMode(switchPin, INPUT);
}

void loop() {
  int switchState = digitalRead(switchPin);

  if (switchState == HIGH) {
    pumpState = INFLATING;
    inflatingStartTime = millis();
    digitalWrite(motorDriverPin1, HIGH);
    digitalWrite(motorDriverPin2, LOW);
    valveState = HIGH; // Open the valve
  }

  switch (pumpState) {
    case INFLATING:
      if (millis() - inflatingStartTime > 5000) { // Inflate for 5 seconds as an example
        pumpState = MEASURING;
        measuringStartTime = millis();
      }
      break;

    case MEASURING:
      if (millis() - measuringStartTime > 10000) { // Measure for 10 seconds as an example
        pumpState = DEFLATING;
        valveState = LOW; // Close the valve to release pressure
      } else {
        int pressure = analogRead(pressureSensorOutputPin);
        // Korotkoff sound detection logic
        if (!korotkoffSoundDetected && abs(pressure - previousPressure) > korotkoffThreshold) {
          korotkoffSoundDetected = true;
          korotkoffCount++;
          if (korotkoffCount == 1) {
            systolicPressure = pressure;
          } else if (korotkoffCount == korotkoffCountThreshold) {
            diastolicPressure = pressure;
          }
        }
        previousPressure = pressure;
      }
      break;

    case DEFLATING:
      // Control the pressure valve to release the pressure
      digitalWrite(pressureValvePin, valveState);
      // You may need to control the valve based on your mechanism and timing
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
}
