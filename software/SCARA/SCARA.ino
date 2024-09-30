#include "config.hpp"
#include "logger.hpp"
#include "button.hpp"
#include "vector.hpp"
#include "stepper.hpp"
#include "structs.hpp"
#include "kinematics.hpp"

// Function declarations
void executeTrajectory(Vector<Point>&, const double = MIN_VELOCITY_STEPS_S, const double = MIN_VELOCITY_STEPS_S, const double = MIN_VELOCITY_STEPS_S);
void moveStraightLine(const Point&, const Point&, const double = MIN_VELOCITY_STEPS_S, const double = MIN_VELOCITY_STEPS_S, const double =  MIN_VELOCITY_STEPS_S, const int = 20);
void moveArc(const Point&, const Point&, const Point&, const double = MIN_VELOCITY_STEPS_S, const double = MIN_VELOCITY_STEPS_S, const double =  MIN_VELOCITY_STEPS_S, const int = 20);
void reachCartesian(const Point&, const double = MIN_VELOCITY_STEPS_S, const double = MAX_VELOCITY_STEPS_S, const double = MIN_VELOCITY_STEPS_S, const double = ACCELERATION);
void reachJoint(const IKSolution&, const double = MIN_VELOCITY_STEPS_S, const double = MAX_VELOCITY_STEPS_S, const double = MIN_VELOCITY_STEPS_S, const double = ACCELERATION);
Steps IKSolution2Steps(const IKSolution&);
void homeAxis(StepperMotor&, Button&, long, double = HOMING_VELOCITY_STEPS_S);
void homeAll();
void moveAll();
void disable();
void enable();

// Define the steppers
StepperMotor stepper1(STEPPER_1_STEP_PIN, STEPPER_1_DIR_PIN);
StepperMotor stepper2(STEPPER_2_STEP_PIN, STEPPER_2_DIR_PIN);
StepperMotor stepper3(STEPPER_3_STEP_PIN, STEPPER_3_DIR_PIN);

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
  Logger::setLogLevel(Logger::DEBUG);

  // Initialize the limit switch pins
  pinMode(STEPPER_1_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEPPER_2_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(STEPPER_3_LIMIT_SWITCH_PIN, INPUT_PULLUP);
  Logger::info("Set up limit switches.");

  // Enable the board
  pinMode(ENABLE, OUTPUT);
  enable();
  
  // Home axis
  homeAll();
  Logger::info("All axis homed.");

  /* --------------------------- Execute trajectory --------------------------- */

  // Define an straight line
  trajectory.pushBack(Point(0.10, -0.05, 0.05));
  trajectory.pushBack(Point(0.10, -0.04, 0.05));
  trajectory.pushBack(Point(0.10, -0.03, 0.05));
  trajectory.pushBack(Point(0.10, -0.02, 0.05));
  trajectory.pushBack(Point(0.10, -0.01, 0.05));
  trajectory.pushBack(Point(0.10, 0.01, 0.05));
  trajectory.pushBack(Point(0.10, 0.02, 0.05));
  trajectory.pushBack(Point(0.10, 0.03, 0.05));
  trajectory.pushBack(Point(0.10, 0.04, 0.05));
  trajectory.pushBack(Point(0.10, 0.05, 0.05));
  
  // Position above the first point, a little higher
  reachCartesian(Point(trajectory[0].x, trajectory[0].y, trajectory[1].z + 0.01));

  // Wait for a keypress of the first endstop to start the motion
  while (!button1.pressed()) delay(10);

  executeTrajectory(trajectory, MIN_VELOCITY_STEPS_S, 2000, MIN_VELOCITY_STEPS_S); // peak at 2000 steps/s

  delay(2000);
  disable();
}

/* ------------------------------ Trajectories ------------------------------ */

void executeTrajectory(
    Vector<Point>& trajectory, 
    const double initialVelocity,
    const double maxVelocity,
    const double finalVelocity
    ) {

    for (int i = 0; i < trajectory.getSize() - 1; ++i) {

        // Define velocities
        double vi = i == 0 ? initialVelocity : maxVelocity;
        double vf = i == trajectory.getSize() - 2 ? finalVelocity : maxVelocity;

        moveStraightLine(trajectory[i], trajectory[i + 1], vi, maxVelocity, vf);
    }
}

void moveStraightLine(
  const Point& start,
  const Point& end,
  const double initialVelocity, 
  const double maxVelocity,
  const double finalVelocity,
  const int points
) {

  Vector<Point> interpolatedPoints = interpolateLine3D(start, end, points);

  for (int i = 0; i < interpolatedPoints.getSize(); ++i) {

    Point p = interpolatedPoints[i];

    // Compute the inverse kinematics
    IKSolution configuration;
    bool result = inverseKinematics(p, configuration);
    if (!result) {
      Logger::error("Point ({}, {}, {}) is not reachable.", p.x, p.y, p.z);
      return;
    }
    // Logger::debug("Inverse kinematics solution: ({}, {}, {})", configuration.q1, configuration.q2, configuration.q3);

    // Compute the steps it takes to reach the joint configuration
    Steps steps = IKSolution2Steps(configuration);
    // Logger::debug("Going from: ({}, {}, {})", currSteps.s1, currSteps.s2, currSteps.s3);
    // Logger::debug("        to: ({}, {}, {})", steps.s1, steps.s2, steps.s3);

    // Find longest distance to cover
    long distance1 = abs(stepper1.getCurrentPosition() - steps.s1);
    long distance2 = abs(stepper2.getCurrentPosition() - steps.s2);
    long distance3 = abs(stepper3.getCurrentPosition() - steps.s3);

    double maxSteps = abs(max(distance1, max(distance2, distance3)) * 1.0);

    // The motor that needs to move the most steps should move at full velocity and acceleration
    // while the other motor's velocity and acceleration are scaled proportionally to match 
    // the duration of the motion of the first one. We use 1 if the result is zero to prevent
    // divition by 0
    double scaleFactor1 = (distance1 != 0) ? (maxSteps / distance1) : 1.0;
    double scaleFactor2 = (distance2 != 0) ? (maxSteps / distance2) : 1.0;
    double scaleFactor3 = (distance3 != 0) ? (maxSteps / distance3) : 1.0;

    // Define velocities
    double v1i = i == 0 ? initialVelocity : maxVelocity;
    double v2i = i == 0 ? initialVelocity : maxVelocity;
    double v3i = i == 0 ? initialVelocity : maxVelocity;

    double v1f = i == trajectory.getSize() - 1 ? finalVelocity : maxVelocity;
    double v2f = i == trajectory.getSize() - 1 ? finalVelocity : maxVelocity;
    double v3f = i == trajectory.getSize() - 1 ? finalVelocity : maxVelocity;

    // Scale velocities and accelerations
    v1i = v1i / scaleFactor1;
    v2i = v2i / scaleFactor2;
    v3i = v3i / scaleFactor3;

    v1f = v1f / scaleFactor1;
    v2f = v2f / scaleFactor2;
    v3f = v3f / scaleFactor3;

    stepper1.moveToPosition(steps.s1, v1i, v1f);
    stepper2.moveToPosition(steps.s2, v2i, v2f);
    stepper3.moveToPosition(steps.s3, v3i, v3f);
 
    moveAll();
  }
}

void moveArc(
  const Point& start,
  const Point& end,
  const Point& center,
  const double initialVelocity, 
  const double maxVelocity,
  const double finalVelocity,
  const int points
) {

  Vector<Point> arcPoints = interpolateArc3D(start, end, center, points);

  for (int i = 0; i < arcPoints.getSize(); ++i) {

    Point p = arcPoints[i];

    // Compute the inverse kinematics
    IKSolution configuration;
    bool result = inverseKinematics(p, configuration);
    if (!result) {
      Logger::error("Point ({}, {}, {}) is not reachable.", p.x, p.y, p.z);
      return;
    }

    // Compute the steps it takes to reach the joint configuration
    Steps steps = IKSolution2Steps(configuration);

    // Find longest distance to cover
    long distance1 = abs(stepper1.getCurrentPosition() - steps.s1);
    long distance2 = abs(stepper2.getCurrentPosition() - steps.s2);
    long distance3 = abs(stepper3.getCurrentPosition() - steps.s3);

    double maxSteps = abs(max(distance1, max(distance2, distance3)) * 1.0);

    // Scale factors for proportional motion
    double scaleFactor1 = (distance1 != 0) ? (maxSteps / distance1) : 1.0;
    double scaleFactor2 = (distance2 != 0) ? (maxSteps / distance2) : 1.0;
    double scaleFactor3 = (distance3 != 0) ? (maxSteps / distance3) : 1.0;

    // Define velocities
    double v1i = i == 0 ? initialVelocity : maxVelocity;
    double v2i = i == 0 ? initialVelocity : maxVelocity;
    double v3i = i == 0 ? initialVelocity : maxVelocity;

    double v1f = i == arcPoints.getSize() - 1 ? finalVelocity : maxVelocity;
    double v2f = i == arcPoints.getSize() - 1 ? finalVelocity : maxVelocity;
    double v3f = i == arcPoints.getSize() - 1 ? finalVelocity : maxVelocity;

    // Scale velocities
    v1i = v1i / scaleFactor1;
    v2i = v2i / scaleFactor2;
    v3i = v3i / scaleFactor3;

    v1f = v1f / scaleFactor1;
    v2f = v2f / scaleFactor2;
    v3f = v3f / scaleFactor3;

    // Move motors
    stepper1.moveToPosition(steps.s1, v1i, v1f);
    stepper2.moveToPosition(steps.s2, v2i, v2f);
    stepper3.moveToPosition(steps.s3, v3i, v3f);

    moveAll();
  }
}

/* ------------------------------ Single point ------------------------------ */

void reachCartesian(
  const Point& p,
  const double initialVelocity, 
  const double maxVelocity,
  const double finalVelocity, 
  const double acceleration
  ) {
  /**
   * Reach the target cartesian point from the current position using a 
   * trapezoidal speed profile (if possible). 
   */

  Logger::debug("-------------------------------------------------");
  Logger::debug("Point: ({}, {}, {})", p.x, p.y, p.z);

  // If the point is reachable
  IKSolution solution;
  bool result = inverseKinematics(p, solution);

  if (!result) {
    Logger::error("Point ({}, {}, {}) is not reachable.", p.x, p.y, p.z);
    return;
  }
  Logger::debug("Inverse kinematics solution: ({}, {}, {})", solution.q1, solution.q2, solution.q3);

  reachJoint(solution, initialVelocity, maxVelocity, finalVelocity, acceleration);
}

void reachJoint(
    const IKSolution& solution, 
    const double initialVelocity, 
    const double maxVelocity, 
    const double finalVelocity,
    const double acceleration
  ) {
  /**
   * Reach the target joint configuration using a trapezoidal speed profile
   * (if possible).
   */

  Steps steps = IKSolution2Steps(solution);
  Logger::debug("Going from: ({}, {}, {})", stepper1.getCurrentPosition(), stepper2.getCurrentPosition(), stepper3.getCurrentPosition());
  Logger::debug("        to: ({}, {}, {})", steps.s1, steps.s2, steps.s3);

  // Find longest distance to cover
  long distance1 = abs(stepper1.getCurrentPosition() - steps.s1);
  long distance2 = abs(stepper2.getCurrentPosition() - steps.s2);
  long distance3 = abs(stepper3.getCurrentPosition() - steps.s3);

  double maxSteps = abs(max(distance1, max(distance2, distance3)) * 1.0);
  Logger::debug("Using scaling coefficient: {}", maxSteps);

  // The motor that needs to move the most steps should move at full velocity and acceleration
  // while the other motor's velocity and acceleration are scaled proportionally to match 
  // the duration of the motion of the first one
  double scaleFactor1 = (distance1 != 0) ? (maxSteps / distance1) : 1.0;
  double scaleFactor2 = (distance2 != 0) ? (maxSteps / distance2) : 1.0;
  double scaleFactor3 = (distance3 != 0) ? (maxSteps / distance3) : 1.0;

  // Define velocities
  double v1i = initialVelocity / scaleFactor1;
  double v2i = initialVelocity / scaleFactor2;
  double v3i = initialVelocity / scaleFactor3;

  double v1m = maxVelocity / scaleFactor1;
  double v2m = maxVelocity / scaleFactor2;
  double v3m = maxVelocity / scaleFactor3;

  double v1f = finalVelocity / scaleFactor1;
  double v2f = finalVelocity / scaleFactor2;
  double v3f = finalVelocity / scaleFactor3;

  Logger::debug("-------------------------------------------------");

  stepper1.moveToPosition(steps.s1, v1i, v1m, v1f, acceleration);
  stepper2.moveToPosition(steps.s2, v2i, v2m, v2f, acceleration);
  stepper3.moveToPosition(steps.s3, v3i, v3m, v3f, acceleration);
 
  moveAll(); 
}

void moveAll() {
  /**
   * Move all steppers until they reach the target position.
   */
  while (!(stepper1.isAtTarget() && stepper2.isAtTarget() && stepper3.isAtTarget())) {
    stepper1.step();
    stepper2.step();
    stepper3.step();
  }
}

Steps IKSolution2Steps(const IKSolution& solution) {
  /**
   * Convert the inverse kinematics solution to steps for each motor 
   * by applying reduction factors and compensations.
   */

  Steps steps;
  steps.s1 = solution.q1 * JOINT_1_STEPS_PER_M;
  steps.s2 = solution.q2 * JOINT_2_STEPS_PER_RAD;

  // The third joint is a special case since we need to adjust it by a factor
  // to account for the variation produced by the movement of the second joint.
  steps.s3 = solution.q3 * JOINT_3_STEPS_PER_RAD;
  steps.s3 += solution.q2 / 2 * JOINT_3_STEPS_PER_RAD;

  return steps;
}

/* --------------------------------- Homing --------------------------------- */

void homeAxis(
    StepperMotor& stepper, 
    Button& limitSwitch, 
    long homingSteps, 
    double velocity
  ) {
  /**
   * Home the specified axis by moving the motor with constant speed until button fires.
   * The motor stops after #homingSteps anyway. We can't use a tapezoidal speed profile
   * since we don't know in advance how many steps we need to take and thus we cannot
   * decelerate properly. We could gradually accelerate and until we reach the maximum
   * velocity but I prefer to cruise to the limit switch with minimum velocity in order 
   * to move safely to the target.
   */

  stepper.moveToPosition(homingSteps, velocity, velocity, velocity, 0.1);
  while (!stepper.isAtTarget()) {

    if (limitSwitch.pressed()) {
      Logger::debug("Endstop {} reached.", limitSwitch.getPin());
      break;
    }

    // Perform a step with constant velocity
    stepper.step();

    // Little delay to smooth things out
    delayMicroseconds(10);
  }

  // Set the zero
  stepper.setCurrentPosition(0);
}

void homeAll() {
  /**
   * Home all the axis.
   */

  // Move the first axis down for 10000 steps or until the limit switch registers a press
  homeAxis(stepper1, button1, -MAX_HOMING_STEPS);
  Logger::debug("Axis 0 homed.");

  // homeAxis(stepper2, button2);
  Logger::debug("Axis 1 homed.");

  // homeAxis(stepper3, button3);
  Logger::debug("Axis 2 homed.");

  // Reach the idle position with the arm fully stretched and the z axis at 1cm from the endstop
  reachJoint(IKSolution(0.01, 0, 0)); 
}

void enable() {
  digitalWrite(ENABLE, LOW);
  Logger::debug("Steppers enabled.");
}

void disable() {
  digitalWrite(ENABLE, HIGH);
  Logger::debug("Steppers disabled.");
}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {
  // Nothing to do here
}
