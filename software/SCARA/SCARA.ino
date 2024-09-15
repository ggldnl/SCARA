#include "config.hpp"
#include "logger.hpp"
#include "stepper.hpp"
#include "button.hpp"
#include "kinematics.hpp"


// Define the steppers
StepperMotor stepper1(STEPPER_1_STEP_PIN, STEPPER_1_DIR_PIN, STEPPER_1_MAX_ACCEL);
StepperMotor stepper2(STEPPER_2_STEP_PIN, STEPPER_2_DIR_PIN, STEPPER_2_MAX_ACCEL);
StepperMotor stepper3(STEPPER_3_STEP_PIN, STEPPER_3_DIR_PIN, STEPPER_3_MAX_ACCEL);

// Define the buttons
Button button1(STEPPER_1_LIMIT_SWITCH_PIN);
Button button2(STEPPER_2_LIMIT_SWITCH_PIN);
Button button3(STEPPER_3_LIMIT_SWITCH_PIN);


void setup() {

  delay(2000);

  // Initialize serial communication
  Serial.begin(115200);

  // Set log level to DEBUG
  Logger::setLogLevel(Logger::DEBUG);

  // Initialize the limit switch pins
  pinMode(STEPPER_1_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEPPER_2_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEPPER_3_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  Logger::debug("Set up limit switches.");

  // Enable the board
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  Logger::info("Board enabled.");

  // Home axis
  homeAll();
  Logger::info("All axis homed.");
  
  // Reach cartesian point
  reachCartesian(120, 20, 100);
}

/* -------------------------------- Movement -------------------------------- */

void moveAll(
    int steps1, 
    int steps2, 
    int steps3, 
    int minDelay = MIN_PULSE_DELAY,
    int maxDelay = MAX_PULSE_DELAY, 
    int incr = INCREMENT
  ) {

  Logger::info("Moving to s1={}, s2={}, s3={}.", steps1, steps2, steps3);

  stepper1.setTargetPosition(steps1);
  stepper2.setTargetPosition(steps2);
  stepper3.setTargetPosition(steps3);

  // Find the minimum acceleration rate across all steppers
  int minAccRate = min(stepper1.getAccRate(), min(stepper2.getAccRate(), stepper3.getAccRate()));

  // Determine the total steps (maximum) to synchronize the movement
  int maxTotalSteps = max(steps1, max(steps2, steps3));

  int del = maxDelay;  // Maximum delay = minimum acceleration
  for (int i = 0; i < maxTotalSteps; i++) {
    
    // We need to take more steps than the steps required for an acceleration and deceleration phase
    if (maxTotalSteps > (2 * minAccRate + 1)) {
      if (i < minAccRate && del - incr >= minDelay) {
        // Acceleration phase
        del -= incr;
      } else if (i > (maxTotalSteps - minAccRate) && del + incr <= maxDelay) {
        // Deceleration phase
        del += incr;
      }

    // Not enough steps for an acceleration and deceleration phase
    } else {
      if (i < maxTotalSteps / 2 && del - incr >= minDelay) {
        // Accelerate until halfway
        del -= incr;
      } else if (i > ((maxTotalSteps + (maxTotalSteps % 2)) / 2) && del + incr <= maxDelay) {
        // Decelerate until end
        del += incr;
      }
    }

    // Step the motors
    stepper1.step();
    stepper2.step();
    stepper3.step();

    // Apply the delay
    delayMicroseconds(del);
  }
}

void reachCartesian(float x, float y, float z) {
  /**
   *  Reach the target cartesian point:
   *  1. Compute the joint values with Inverse Kinematics;
   *  2. Translate joint values to steps;
   *  3. Move by the steps;
   */

  Logger::info("Reaching cartesian point ({}, {}, {})", x, y, z);

  // Variables to store the joint angles
  float q1, q2, q3;
  bool result = inverseKinematics(x, y, z, q1, q2, q3);

  // If the point is reachable
  if (result) {

    reachJoint(q1, q2, q3);
  
  } else {

    Logger::error("Unable to reach cartesian point ({}, {}, {})", x, y, z);
  }
}

void reachJoint(float q1, float q2, float q3) {
  /**
   * Reach the target joint configuration:
   * 1. Translate joint values to steps;
   * 2. Move by the steps;
   */

  Logger::info("Reaching joint configuration q1={}, q2={}, q3={}.", q1, degrees(q2), degrees(q3));

  int steps1 = q1 / JOINT_1_DIST_PER_STEP;
  int steps2 = q2 * JOINT_2_STEPS_PER_RAD;
  
  // The third joint is a special case since we need to adjust it by a factor
  // to account for the variation produced by the movement of the second joint.
  int steps3 = q3 * JOINT_3_STEPS_PER_RAD;
  steps3 += q2/2 * JOINT_3_STEPS_PER_RAD;

  moveAll(steps1, steps2, steps3);

  // Logger::debug("Current position: p1={}, p2={}, p3={}", stepper1.getCurrentPosition(), stepper2.getCurrentPosition(), stepper3.getCurrentPosition());
}

/* --------------------------------- Homing --------------------------------- */

void homeAxis(StepperMotor& stepper, Button& limitSwitch, int del = MAX_PULSE_DELAY, int homingSteps = HOMING_STEPS, int postHomingSteps = POST_HOMING_STEPS) {
  /**
   * Home the specified axis by moving the motor with constant speed until button fires.
   * The motor stops after #homingSteps anyway. We can't use a bang-coast-bang speed profile
   * since we don't know in advance how many steps we need to take. We use the maximum
   * speed pulse delay to produce the smallest speed (better safe than sorrow).
   */

  stepper.setTargetPosition(-homingSteps);
  while(!stepper.isAtTarget()) {

    if (limitSwitch.pressed()) {
      Logger::warn("Button {} pressed.", limitSwitch.getPin());
      break;
    }

    // Perform a step with constant velocity
    stepper.step();
    delayMicroseconds(del);
  }
  Logger::debug("Endstop {} reached.", limitSwitch.getPin());

  stepper.setCurrentPosition(0);
  stepper.setTargetPosition(postHomingSteps);

  while(!stepper.isAtTarget()) {
    stepper.step();
    delayMicroseconds(del);
  }
  Logger::debug("Stepper currently {} steps above endstop {}", stepper.getCurrentPosition(), limitSwitch.getPin());

}

void homeAll() {
  /**
   * Home all the axis.
   */

  homeAxis(stepper1, button1);
  Logger::debug("Axis 0 homed.");

  // homeAxis(stepper2, button2);
  Logger::debug("Axis 1 homed.");

  // homeAxis(stepper3, button3);
  Logger::debug("Axis 2 homed.");

}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {
  // Nothing to do here
}
