#include <AccelStepper.h>

// Pin definitions
#define ENABLE_PIN 8
#define STEP_PIN 2
#define DIR_PIN 5

// Motor parameters
const int stepsPerRevolution = 200;  // Steps per revolution for the motor (without microstepping)
const int microstepping = 1;        // Microstepping setting (1/16 microstepping)
const float reductionRatio = 7.277;    // Gear reduction ratio

// Calculated steps per degree
const float stepsPerDegree = (stepsPerRevolution * microstepping * reductionRatio) / 360.0;

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Current position in degrees
float currentAngle = 0;

void setup() {

  Serial.begin(115200);

  // Set up the enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // Disable the motor driver

  Serial.println("[INFO] Initializing the stepper motor");

  // Set maximum speed and acceleration
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(2000);

  Serial.println("[INFO] Enabling the board");
  digitalWrite(ENABLE_PIN, LOW);  // Enable the motor driver

  Serial.print("[USER] Enter angle in degrees: ");
}

void loop() {

  // Check for serial input
  if (Serial.available() > 0) {
    
    // Read the input as a float
    float targetAngle = Serial.parseFloat();

    // Flush the remaining input from the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    Serial.println(targetAngle);

    // Compute the target position in steps
    long targetPosition = targetAngle * stepsPerDegree;

    // Move to the target position
    stepper.moveTo(targetPosition);
    stepper.runToPosition();

    // Update the current angle
    currentAngle = targetAngle;

    // Print the current angle
    Serial.print("[USER] Current angle: ");
    Serial.println(currentAngle);

    Serial.print("[USER] Enter angle in degrees: ");
  }
}
