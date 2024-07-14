#include <MobaTools.h>
#include "config.h"

// Create stepper motor instances for X, Y, and A axes
MoToStepper motor_1(MOTOR_1_STEPS_PER_REVOLUTION, STEPDIR);
MoToStepper motor_2(MOTOR_2_STEPS_PER_REVOLUTION, STEPDIR);
MoToStepper motor_3(MOTOR_3_STEPS_PER_REVOLUTION, STEPDIR);


void setup() {

  Serial.begin(115200);

  // Initialize the enable pin
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);  // Setup the motors with the CNC shield disabled

  // Initialize the limit switch pins
  pinMode(MOTOR_1_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_2_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_3_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Initialize the stepper motors
  Serial.println("[INFO] Initializing the stepper motors");

  motor_1.attach(MOTOR_1_STEP_PIN, MOTOR_1_DIR_PIN);
  motor_2.attach(MOTOR_2_STEP_PIN, MOTOR_2_DIR_PIN);
  motor_3.attach(MOTOR_3_STEP_PIN, MOTOR_3_DIR_PIN);

  // Setup speed and ramp
  setupSpeedAndRamp();

  // Enable the CNC shield
  Serial.println("[INFO] Enabling the board");
  digitalWrite(ENABLE, LOW);

  delay(1000);

  // Perform the homing sequence
  homeAllAxes();

  // Move to position
  // reachCartesian(0, 0, 100);
}

bool isMoving(void) {
  /** 
   *  Return true if any one of the drivers are still moving.
   */

  return motor_1.moving() || motor_2.moving() || motor_3.moving();
}

void stepAll(long steps_1, long steps_2, long steps_3) {
  /**
   *  Move a relative displacement at the current speed, blocking until the move is done.
   */
  
  // Move the steppers
  motor_1.write(steps_1);
  // motor_2.write(steps_2);
  // motor_3.write(steps_3);

  while(isMoving()); // Wait for the movements to complete
}

void reachCartesian(float x, float y, float z) {
  /**
   *  Reach the target cartesian point:
   *  1. Compute the joint values with Inverse Kinematics;
   *  2. Translate joint values to steps;
   *  3. Move by the steps;
   */

  // Variables to store the joint angles
  float q1, q2, q3;
  bool result = inverseKinematics(x, y, z, q1, q2, q3);

  // If the point is reachable
  if (result) {

    // Print the cartesian point
    Serial.print("[INFO] Reaching cartesian point (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(")");
    
    // Print the computed joint values
    Serial.print("[INFO] Target joint values: ");
    Serial.print(q1);
    Serial.print(", ");
    Serial.print(q2);
    Serial.print(", ");
    Serial.println(q3);

    reachJoint(q1, q2, q3);
  
  } else {

    // Print the cartesian point
    Serial.print("[WARN] Cartesian point (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(") is not reachable");

  }
}

void reachJoint(float joint_1, float joint_2, float joint_3) {
  /**
   *  Reach the target configuration in joint space:
   *  2. Translate joint values to steps;
   *  3. Move by the steps;
   */

  float steps_1 = joint_1 * JOINT_1_REDUCTION * MOTOR_1_STEPS_PER_REVOLUTION;
  float steps_3 = joint_3 * JOINT_3_REDUCTION * MOTOR_3_STEPS_PER_REVOLUTION;

  // To move joint 2 we need to combine the motion of motor 2 and 3
  float steps_2 = 0;

  stepAll(steps_1, steps_2, steps_3);
}

void setupSpeedAndRamp() {

  // Set the speed of the stepper motors
  motor_1.setSpeed(MOTOR_1_SPEED);
  motor_2.setSpeed(MOTOR_2_SPEED);
  motor_3.setSpeed(MOTOR_3_SPEED);

  // Set the ramp length
  motor_1.setRampLen(MOTOR_1_RAMP_LENGTH);
  motor_2.setRampLen(MOTOR_2_RAMP_LENGTH);
  motor_3.setRampLen(MOTOR_3_RAMP_LENGTH);
}

void setupHomingSpeedAndRamp() {

  // Set the speed of the stepper motors
  motor_1.setSpeed(MOTOR_1_HOMING_SPEED);
  motor_2.setSpeed(MOTOR_2_HOMING_SPEED);
  motor_3.setSpeed(MOTOR_3_HOMING_SPEED);

  // Set the ramp length
  motor_1.setRampLen(MOTOR_1_HOMING_RAMP_LENGTH);
  motor_2.setRampLen(MOTOR_2_HOMING_RAMP_LENGTH);
  motor_3.setRampLen(MOTOR_3_HOMING_RAMP_LENGTH);
}

void homeAllAxes() {
  /**
   * Homing sequence for all axes
   */

  Serial.println("[INFO] Starting homing sequence");
  
  // Change speed and ramp length
  setupHomingSpeedAndRamp();

  // Home each motor
  homeMotor(motor_1, MOTOR_1_LIMIT_SWITCH_PIN);
  homeMotor(motor_2, MOTOR_2_LIMIT_SWITCH_PIN);
  homeMotor(motor_3, MOTOR_3_LIMIT_SWITCH_PIN);

  // Go back to previous speed and ramp values
  setupSpeedAndRamp();

}

void homeMotor(MoToStepper &motor, byte limitSwitchPin) {
  /**
   * Homing sequence for a single axis
   */

  // motor.setSpeed(homingSpeed);  // Set a slow speed for homing
  // motor.setRampLen(homingRampLength);
  
  motor.write(-10000);  // Move the motor in the negative direction by A LOT

  // Wait until the limit switch is triggered
  while (digitalRead(limitSwitchPin) == HIGH);
  Serial.println("[WARN] Limit switch triggered");

  // Stop the motor
  motor.stop();

  // Set the current position to zero
  motor.setZero();

  // Move the motor out of the limit switch
  motor.write(POST_HOMING_STEPS); 
}

bool isReachable(float x, float y, float z) {
  /**
   * TODO better define the workspace
   * Check if a cartesian point is reachable
   */

  float distance = sqrt(x * x + y * y);
  return distance <= (L1 + L2) && JOINT_1_MIN_LIMIT < z && z < JOINT_1_MAX_LIMIT;
}

bool inverseKinematics(float x, float y, float z, float &q1, float &q2, float &q3) {
  /**
    * Compute the inverse kinematics for a r
    */

  if (!isReachable(x, y, z)) {
    return false;  // The point is out of reach
  }

  // First joint (directly determined by z)
  q1 = z;

  // Third joint (calculating c3 and s3)
  float c3 = (x * x + y * y) / (L2 * L2 + L1 * L1 + 2 * L1 * L2);
  float s3_p = sqrt(1 - c3 * c3);
  float q3_p = atan2(s3_p, c3);

  // Second joint (calculating q2 for the positive s3 solution)
  float A = L2 * c3 + L1;
  float B_p = L2 * s3_p;
  float c2_p = (A * x + B_p * y);
  float s2_p = (-B_p * y - A * x);
  float q2_p = atan2(s2_p, c2_p);

  // Assign the solutions
  q2 = q2_p;
  q3 = q3_p;

  return true;
}


void loop() {
  // Nothing to do here
}
