#include "config.hpp"
#include "logger.hpp"
#include "stepper.hpp"
#include "button.hpp"
#include "kinematics.hpp"


// Define the steppers
StepperMotor stepper1(STEPPER_1_STEP_PIN, STEPPER_1_DIR_PIN, STEPPER_1_MAX_ACCEL);
StepperMotor stepper2(STEPPER_2_STEP_PIN, STEPPER_2_DIR_PIN, STEPPER_2_MAX_ACCEL);
StepperMotor stepper3(STEPPER_3_STEP_PIN, STEPPER_3_DIR_PIN, STEPPER_3_MAX_ACCEL);

StepperMotor* steppers[] = {&stepper1, &stepper2, &stepper3};
const uint8_t numSteppers = sizeof(steppers) / sizeof(steppers[0]);

// Define the buttons
Button button1(STEPPER_1_LIMIT_SWITCH_PIN);
Button button2(STEPPER_2_LIMIT_SWITCH_PIN);
Button button3(STEPPER_3_LIMIT_SWITCH_PIN);

Button* buttons[] = {&button1, &button2, &button3};
const uint8_t numButtons = sizeof(buttons) / sizeof(buttons[0]);


void setup() {

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
  Logger::debug("Board enabled.");

  // Home axis
  homeAll();
  Logger::debug("All axis homed.");

  // Reach cartesian point
  reachCartesian(120, 20, 100);
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

    Logger::info("Reach joint configuration q1={}, q2={}, q3={}.", q1, degrees(q2), degrees(q3));
    // reachJoint(q1, q2, q3);
  
  } else {

    Logger::error("Unable to reach cartesian point ({}, {}, {})", x, y, z);
  }
}

void homeAxis(StepperMotor* stepper, Button* limitSwitch, int del = BASE_DELAY, int homingSteps = HOMING_STEPS, int postHomingSteps = POST_HOMING_STEPS) {
  /**
   * Home the specified axis by moving the motor with constant speed until button fires.
   * The motor stops after #homingSteps anyway. 
   */

  stepper->setTargetPosition(-homingSteps);
  while(!stepper->isAtTarget()) {

    if (limitSwitch->pressed()) {
      Logger::warn("Button {} pressed.", limitSwitch->getPin());
      break;
    }

    // Perform a step with constant velocity
    stepper->step();
    delayMicroseconds(del);
  }
  Logger::debug("Endstop {} reached.", limitSwitch->getPin());

  stepper->setCurrentPosition(0);
  stepper->setTargetPosition(postHomingSteps);

  while(!stepper->isAtTarget()) {
    stepper->step();
    delayMicroseconds(del);
  }
  Logger::debug("Stepper currently {} steps above endstop {}", stepper->getCurrentPosition(), limitSwitch->getPin());

}

void homeAll() {
  /**
   * Home all the axis.
   */

  // homeAxis(steppers[0], buttons[0]);
  Logger::debug("Axis 0 homed.");

  // homeAxis(steppers[1], buttons[1]);
  Logger::debug("Axis 1 homed.");

  // homeAxis(steppers[2], buttons[2]);
  Logger::debug("Axis 2 homed.");

}

// @deprecated lol
void bangCoastBang(StepperMotor* stepper, int totalSteps, int baseDelay = BASE_DELAY, int incr = INCREMENT) {

  int del = baseDelay;              // Delay between pulses
  int acc = stepper->getAccRate();  // Maximum acceleration for the motor

  for (int i = 0; i < totalSteps; ++i) {

    // We need to take more steps than the steps required for an acceleration and deceleration phase
    if (totalSteps > (2 * acc + 1)) {
      if (i < acc)
        // Acceleration phase
        del -= incr;
      else if (i > (totalSteps - acc))
        // Deceleration phase
        del += incr;

    // Not enough steps for an acceleration and deceleration phase
    } else {
      if (i < totalSteps / 2)
        // Accelerate until halfway
        del -= incr;
      else
        // Decelerate until end
        del += incr;
    }
  }

  // Perform a step with the current delay value
  stepper->step();
  delayMicroseconds(del);
}

void loop() {
  // Nothing to do here
}
