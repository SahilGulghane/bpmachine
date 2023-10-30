#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Q2HX711.h>

const byte MPS_OUT_pin = 35; // OUT data pin
const byte MPS_SCK_pin = 34; // clock data pin
int avg_size = 10; // #pts to average over

const float ADC_mV = 4.8828125;   
const float sensitivity = 4.413;   
const float mmh2O_kpa = 0.00981;

Q2HX711 Q2HX711(MPS_OUT_pin, MPS_SCK_pin); // start comm with the HX710B

// Define the pins that the motor driver, pressure valve and switch are connected to
const int motorDriverPin1 = 32;
const int motorDriverPin2 = 33;
const int pressureValvePin = 18;
const int switchPin = 25;

// Define the pump state variable
int pumpState = OFF;

void setup() {
  // Initialize the serial port
  Serial.begin(9600);

  // Start the Q2HX711 sensor
  Q2HX711.start();

  // Set the motor driver pins to output mode
  pinMode(motorDriverPin1, OUTPUT);
  pinMode(motorDriverPin2, OUTPUT);

  // Set the pressure valve pin to output mode
  pinMode(pressureValvePin, OUTPUT);

  // Set the switch pin to input mode
  pinMode(switchPin, INPUT);

  // Initialize the pump state variable
  pumpState = OFF;
}

void loop() {
  // Read the switch state
  int switchState = digitalRead(switchPin);

  // If the switch is ON, turn on the pump
  if (switchState == HIGH) {
    pumpState = ON;
  } else {
    pumpState = OFF;
  }

  // Control the pump based on the pump state variable
  if (pumpState == ON) {
    digitalWrite(motorDriverPin1, HIGH);
    digitalWrite(motorDriverPin2, LOW);
  } else {
    digitalWrite(motorDriverPin1, LOW);
    digitalWrite(motorDriverPin2, HIGH);
  }

  // Read the pressure sensor data
  float pressure = Q2HX711.read();

  // Calculate the systolic and diastolic blood pressure using the Korotkoff algorithm
  int systolicPressure = 0;
  int diastolicPressure = 0;
  int korotkoffCount = 0;

  // Detect the Korotkoff sounds
  while (pressure > korotkoffThreshold) {
    if (pressure < systolicPressure) {
      systolicPressure = pressure;
    }

    if (korotkoffCount == korotkoffCount) {
      diastolicPressure = pressure;
      break;
    }

    if (pressure - previousPressure > korotkoffThreshold) {
      korotkoffCount++;
    }

    // Read the pressure sensor data
    previousPressure = pressure;
    pressure = Q2HX711.read();
  }

  // Turn off the pump
  digitalWrite(motorDriverPin1, LOW);
  digitalWrite(motorDriverPin2, HIGH);

  // Open the pressure valve to release pressure from the cuff
  digitalWrite(pressureValvePin, HIGH);
  delay(2000);

  // Close the pressure valve
  digitalWrite(pressureValvePin, LOW);

  // Display the systolic and diastolic blood pressure readings on the serial port
  Serial.print("Systolic pressure: ");
  Serial.println(systolicPressure);
  Serial.print("Diastolic pressure: ");
  Serial.println(diastolicPressure);
}
