#include <AccelStepper.h>

// Define the stepper motors
const int stepsPerRevolution = 200; // Assuming 1.8 degrees per step (200 steps per revolution)

// Create stepper motor instances
AccelStepper stepperX(AccelStepper::DRIVER, 2, 5);  // X-axis stepper
AccelStepper stepperY(AccelStepper::DRIVER, 3, 6);  // Y-axis stepper
AccelStepper stepperA(AccelStepper::DRIVER, 12, 13); // A-axis stepper (typically Z-axis or an additional axis)

void setup() {

  pinMode(8, OUTPUT); 
  digitalWrite(8, HIGH); // Disable the board

  // Initialize the stepper motors
  Serial.println("[INFO] Initializing the stepper motors");

  // Set the speed of the steppers (steps per second)
  stepperX.setMaxSpeed(2000); // Adjust speed as needed
  stepperY.setMaxSpeed(2000); // Adjust speed as needed
  stepperA.setMaxSpeed(2000); // Adjust speed as needed

  // Set acceleration (steps per second^2)
  stepperX.setAcceleration(1000); // Adjust acceleration as needed
  stepperY.setAcceleration(1000); // Adjust acceleration as needed
  stepperA.setAcceleration(1000); // Adjust acceleration as needed

  Serial.println("[INFO] Enabling the board");
  digitalWrite(8, LOW); // Enable the board

  // Move each stepper motor forward 200 steps and then backward 200 steps
  delay(1000);

  // Move X-axis stepper
  Serial.println("[INFO] Moving stepper on X Axis by 200 steps forward");
  stepperX.moveTo(200);    // Move forward 200 steps
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(1000);

  Serial.println("[INFO] Moving stepper on X Axis by 200 steps backward");
  stepperX.moveTo(0);   // Move backward 200 steps
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  delay(1000);

  // Move Y-axis stepper
  Serial.println("[INFO] Moving stepper on Y Axis by 200 steps forward");
  stepperY.moveTo(200);    // Move forward 200 steps
  while (stepperY.distanceToGo() != 0) {
    stepperY.run();
  }
  delay(1000);

  Serial.println("[INFO] Moving stepper on Y Axis by 200 steps backward");
  stepperY.moveTo(0);   // Move backward 200 steps
  while (stepperY.distanceToGo() != 0) {
    stepperY.run();
  }
  delay(1000);

  // Move A-axis stepper
  Serial.println("[INFO] Moving stepper on A Axis by 200 steps forward");
  stepperA.moveTo(200);    // Move forward 200 steps
  while (stepperA.distanceToGo() != 0) {
    stepperA.run();
  }
  delay(1000);

  Serial.println("[INFO] Moving stepper on A Axis by 200 steps backward");
  stepperA.moveTo(0);   // Move backward 200 steps
  while (stepperA.distanceToGo() != 0) {
    stepperA.run();
  }
  delay(1000);
}

void loop() {
  // Nothing to do here
}
