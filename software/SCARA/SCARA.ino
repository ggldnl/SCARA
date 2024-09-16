#include "config.hpp"
#include "logger.hpp"
#include "button.hpp"
#include "vector.hpp"
#include "stepper.hpp"
#include "structs.hpp"
#include "kinematics.hpp"


// Define the steppers
StepperMotor stepper1(STEPPER_1_STEP_PIN, STEPPER_1_DIR_PIN, STEPPER_1_MAX_ACCEL);
StepperMotor stepper2(STEPPER_2_STEP_PIN, STEPPER_2_DIR_PIN, STEPPER_2_MAX_ACCEL);
StepperMotor stepper3(STEPPER_3_STEP_PIN, STEPPER_3_DIR_PIN, STEPPER_3_MAX_ACCEL);

// Define the buttons
Button button1(STEPPER_1_LIMIT_SWITCH_PIN);
Button button2(STEPPER_2_LIMIT_SWITCH_PIN);
Button button3(STEPPER_3_LIMIT_SWITCH_PIN);

// Define the trajectory to follow
Vector<Point> trajectory;

void setup() {

  delay(2000);

  // Initialize serial communication
  Serial.begin(115200);

  // Set log level to INFO
  Logger::setLogLevel(Logger::INFO);

  // Initialize the limit switch pins
  pinMode(STEPPER_1_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEPPER_2_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEPPER_3_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  Logger::info("Set up limit switches.");

  // Enable the board
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  Logger::info("Board enabled.");

  // Home axis
  homeAll();
  Logger::info("All axis homed.");

  // Reach cartesian point
  /*
  Point target(120, 20, 100);
  reachCartesian(target);
  */

  // Add the points to the trajectory (square)
  trajectory.push_back(Point(155.36, 35.36, 100.00));
  trajectory.push_back(Point(84.64, 35.36, 100.00));
  trajectory.push_back(Point(84.64, -35.36, 100.00));
  trajectory.push_back(Point(155.36, -35.36, 100.00));

  // Validate the trajectory and print a warn message if any of the point is not reachable
  for (size_t i = 0; i < trajectory.getSize(); i++) {
      IKSolution solution;
      if (!inverseKinematics(trajectory[i], solution)) {
          Logger::warn("Point ({}, {}, {}) is not reachable.", trajectory[i].x, trajectory[i].y, trajectory[i].z);  
      }
  }
   
  Vector<IKSolution> solutions = inverseKinematicsVector(trajectory);
  if (solutions.getSize() > 0) {
    executeTrajectory(solutions);
    Logger::info("Executing trajectory.");
  } else {
    Logger::warn("Unable to execute trajectory: one or more of the points are not reachable.");
  }

  delay(2000);
  reset();
}

/* ------------------------------ Trajectories ------------------------------ */

void executeTrajectory(const Vector<IKSolution>& solutions) {

    for (size_t i = 0; i < solutions.getSize(); i++) {
        const IKSolution& solution = solutions[i];

        // TODO provide a method that uses initial and final velocity
        reachJoint(solution);
    }
}

/* ------------------------------ Single point ------------------------------ */

void reachCartesian(const Point& p) {
  /**
   *  Reach the target cartesian point:
   *  1. Compute the joint values with Inverse Kinematics;
   *  2. Translate joint values to steps;
   *  3. Move by the steps;
   */

  // Logger::debug("Reaching cartesian point ({}, {}, {})", p.x, p.y, p.z);

  IKSolution solution;
  bool result = inverseKinematics(p, solution);

  // If the point is reachable
  if (result) {

    reachJoint(solution);
  
  } else {

    Logger::error("Unable to reach cartesian point ({}, {}, {})", p.x, p.y, p.z);
  }
}

void reachJoint(const IKSolution& solution) {
  /**
   * Reach the target joint configuration:
   * 1. Translate joint values to steps;
   * 2. Move by the steps;
   */

  // Logger::debug("Reaching joint configuration q1={}, q2={}, q3={}.", solution.q1, degrees(solution.q2), degrees(solution.q3));

  Steps steps;
  steps.s1 = solution.q1 / JOINT_1_DIST_PER_STEP;
  steps.s2 = solution.q2 * JOINT_2_STEPS_PER_RAD;
  
  // The third joint is a special case since we need to adjust it by a factor
  // to account for the variation produced by the movement of the second joint.
  steps.s3 = solution.q3 * JOINT_3_STEPS_PER_RAD;
  steps.s3 += solution.q2/2 * JOINT_3_STEPS_PER_RAD;

  moveAll(steps);

  // Logger::debug("Current position: p1={}, p2={}, p3={}", stepper1.getCurrentPosition(), stepper2.getCurrentPosition(), stepper3.getCurrentPosition());
}

void moveAll(
    const Steps& steps, 
    int minDelay = MIN_PULSE_DELAY,
    int maxDelay = MAX_PULSE_DELAY, 
    int incr = INCREMENT
  ) {

  // Logger::debug("Moving to s1={}, s2={}, s3={}.", steps.s1, steps.s2, steps.s3);

  stepper1.setTargetPosition(steps.s1);
  stepper2.setTargetPosition(steps.s2);
  stepper3.setTargetPosition(steps.s3);

  // Find the minimum acceleration rate across all steppers
  int minAccRate = min(stepper1.getAccRate(), min(stepper2.getAccRate(), stepper3.getAccRate()));

  // Determine the total steps (maximum) to synchronize the movement
  int maxTotalSteps = max(steps.s1, max(steps.s2, steps.s3));

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

void reset() {
  /**
   * Reset to initial position.
   */
   IKSolution initialPose(100, 0, 0);
   reachJoint(initialPose);
   Logger::debug("Position reset to q1={}, q2={}, q3={}", initialPose.q1, initialPose.q2, initialPose.q3);
}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {
  // Nothing to do here
}
