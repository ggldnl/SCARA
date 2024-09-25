#include "config.hpp"
#include "logger.hpp"
#include "button.hpp"
#include "vector.hpp"
#include "stepper.hpp"
#include "structs.hpp"
#include "kinematics.hpp"

// Function declarations
void executeTrajectory(Vector<Point>&, const float = MIN_VELOCITY_STEPS_S, const float = MID_VELOCITY_STEPS_S, const float = ACCELERATION);
void reachCartesian(const Point&, const float = MIN_VELOCITY_STEPS_S, const float = MID_VELOCITY_STEPS_S, const float = ACCELERATION);
void reachJoint(const IKSolution&, const float = MIN_VELOCITY_STEPS_S, const float = MID_VELOCITY_STEPS_S, const float = ACCELERATION);
void moveAll();
Steps IKSolution2Steps(const IKSolution&);
void enable();
void disable();


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
  // homeAll();
  Logger::info("All axis homed.");

  /* --------------------------- Execute trajectory --------------------------- */

  // Add the points to the trajectory

  /*
  // 3-point line
  trajectory.pushBack(Point(0.10, -0.14, 0.1));
  trajectory.pushBack(Point(0.10, 0.0, 0.1));;
  trajectory.pushBack(Point(0.10, 0.14, 0.1));
  */

  /*
  // 10-point line
  trajectory.pushBack(Point(0.10, -0.14, 0.1));
  trajectory.pushBack(Point(0.10, -0.11, 0.1));
  trajectory.pushBack(Point(0.10, -0.08, 0.1));
  trajectory.pushBack(Point(0.10, -0.05, 0.1));
  trajectory.pushBack(Point(0.10, -0.02, 0.1));
  trajectory.pushBack(Point(0.10, 0.02, 0.1));
  trajectory.pushBack(Point(0.10, 0.05, 0.1));
  trajectory.pushBack(Point(0.10, 0.08, 0.1));
  trajectory.pushBack(Point(0.10, 0.11, 0.1));
  trajectory.pushBack(Point(0.10, 0.14, 0.1));
  */

  // Circle
  trajectory.pushBack(Point(0.125, 0.000, 0.1));
  trajectory.pushBack(Point(0.124, 0.003, 0.1));
  trajectory.pushBack(Point(0.122, 0.005, 0.1));
  trajectory.pushBack(Point(0.118, 0.005, 0.1));
  trajectory.pushBack(Point(0.116, 0.003, 0.1));
  trajectory.pushBack(Point(0.115, 0.000, 0.1));
  trajectory.pushBack(Point(0.116, -0.003, 0.1));
  trajectory.pushBack(Point(0.118, -0.005, 0.1));
  trajectory.pushBack(Point(0.122, -0.005, 0.1));
  trajectory.pushBack(Point(0.124, -0.003, 0.1));

  executeTrajectory(trajectory);
  // reachCartesian(trajectory[0]);

  delay(2000);
  disable();
}

/* ------------------------------ Trajectories ------------------------------ */

void executeTrajectory(
    Vector<Point>& trajectory, 
    const float restVelocity = MIN_VELOCITY_STEPS_S, 
    const float cruiseVelocity = MID_VELOCITY_STEPS_S, 
    const float acceleration = ACCELERATION
  ) {

  // Vectors to buffer computations
  Vector<Steps> trajectorySteps;
  Matrix<float> initialVelocities(trajectory.getSize(), 3); // {{v1i, v2i, v3i}, ...}
  Matrix<float> finalVelocities(trajectory.getSize(), 3);   // {{v1f, v2f, v3f}, ...}

  // Emulate the motors during debug
  Steps currSteps(0, 0, 0);
  
  Logger::debug("-------------------------------------------------");
  for (size_t i = 0; i < trajectory.getSize(); i++) {

    Point point = trajectory[i];
    Logger::debug("Point {}: ({}, {}, {})", i, point.x, point.y, point.z);

    // Compute the inverse kinematics
    IKSolution configuration;
    bool result = inverseKinematics(point, configuration);
    if (!result) {
      Logger::error("Point ({}, {}, {}) is not reachable.", point.x, point.y, point.z);
      return;
    }
    Logger::debug("Inverse kinematics solution: ({}, {}, {})", configuration.q1, configuration.q2, configuration.q3);

    // Compute the steps it takes to reach the joint configuration
    Steps steps = IKSolution2Steps(configuration);
    Logger::debug("Going from: ({}, {}, {})", currSteps.s1, currSteps.s2, currSteps.s3);
    Logger::debug("        to: ({}, {}, {})", steps.s1, steps.s2, steps.s3);

    // Find longest distance to cover
    int distance1 = abs(currSteps.s1 - steps.s1);
    int distance2 = abs(currSteps.s2 - steps.s2);
    int distance3 = abs(currSteps.s3 - steps.s3);
    Logger::debug("Distances: ({}, {}, {})", distance1, distance2, distance3);
    Logger::debug("");

    float maxSteps = max(distance1, max(distance2, distance3)) * 1.0;
    Logger::debug("Using scaling coefficient: {}", maxSteps);

    // The motor that needs to move the most steps should move at full velocity and acceleration
    // while the other motor's velocity and acceleration are scaled proportionally to match 
    // the duration of the motion of the first one
    float scaleFactor1 = distance1 != 0 ? maxSteps / distance1 : 1;
    float scaleFactor2 = distance2 != 0 ? maxSteps / distance2 : 1;
    float scaleFactor3 = distance3 != 0 ? maxSteps / distance3 : 1;

    // Define velocities
    float v1i = i == 0 ? restVelocity : cruiseVelocity;
    float v2i = i == 0 ? restVelocity : cruiseVelocity;
    float v3i = i == 0 ? restVelocity : cruiseVelocity;

    float v1f = i == trajectory.getSize() - 1 ? restVelocity : cruiseVelocity;
    float v2f = i == trajectory.getSize() - 1 ? restVelocity : cruiseVelocity;
    float v3f = i == trajectory.getSize() - 1 ? restVelocity : cruiseVelocity;

    Logger::debug("Original velocities:");
    Logger::debug("{} -> {}", v1i, v1f);
    Logger::debug("{} -> {}", v2i, v2f);
    Logger::debug("{} -> {}", v3i, v3f);
    Logger::debug("");
  
    // Scale velocities and accelerations
    v1i = v1i / scaleFactor1;
    v2i = v2i / scaleFactor2;
    v3i = v3i / scaleFactor3;

    v1f = v1f / scaleFactor1;
    v2f = v2f / scaleFactor2;
    v3f = v3f / scaleFactor3;

    Logger::debug("Scaled velocities:");
    Logger::debug("{} -> {}", v1i, v1f);
    Logger::debug("{} -> {}", v2i, v2f);
    Logger::debug("{} -> {}", v3i, v3f);
    Logger::debug("");

    Vector<float> vi(3);
    vi.pushBack(v1i);
    vi.pushBack(v2i);
    vi.pushBack(v3i);

    Vector<float> vf(3);
    vf.pushBack(v1f);
    vf.pushBack(v2f);
    vf.pushBack(v3f);

    // Buffer the computation
    trajectorySteps.pushBack(steps);
    initialVelocities.fillRow(i, vi);
    finalVelocities.fillRow(i, vf);

    Logger::debug("stepper1.moveToPosition({}, {}, {}, {}, {})", steps.s1, v1i, cruiseVelocity, v1f, acceleration);
    Logger::debug("stepper2.moveToPosition({}, {}, {}, {}, {})", steps.s2, v2i, cruiseVelocity, v2f, acceleration);
    Logger::debug("stepper3.moveToPosition({}, {}, {}, {}, {})", steps.s3, v3i, cruiseVelocity, v3f, acceleration);
    Logger::debug("");

    // Update the emulated steppers
    currSteps.s1 = steps.s1;
    currSteps.s2 = steps.s2;
    currSteps.s3 = steps.s3;

    Logger::debug("-------------------------------------------------");
  }

  Logger::debug("Starting trajectory execution.");
  for (size_t i = 0; i < trajectorySteps.getSize(); i++) {

    Steps steps = trajectorySteps[i];

    stepper1.moveToPosition(steps.s1, initialVelocities(i, 0), cruiseVelocity, finalVelocities(i, 0), acceleration);
    stepper2.moveToPosition(steps.s2, initialVelocities(i, 1), cruiseVelocity, finalVelocities(i, 1), acceleration);
    stepper3.moveToPosition(steps.s3, initialVelocities(i, 2), cruiseVelocity, finalVelocities(i, 2), acceleration);
 
    moveAll();
  }

  Logger::debug("Done.");
}

/* ------------------------------ Single point ------------------------------ */

void reachCartesian(const Point& p,
  const float restVelocity = MIN_VELOCITY_STEPS_S, 
  const float cruiseVelocity = MID_VELOCITY_STEPS_S, 
  const float acceleration = ACCELERATION
  ) {
  /**
   * Reach the target cartesian point. We will first compute the joint values with 
   * the Inverse Kinematics routine, then translate them to steps and then move
   * to the position.
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

void reachJoint(
    const IKSolution& solution, 
    const float restVelocity = MIN_VELOCITY_STEPS_S, 
    const float cruiseVelocity = MID_VELOCITY_STEPS_S, 
    const float acceleration = ACCELERATION
  ) {
  /**
   * Reach the target joint configuration. Compute the steps required to reach
   * the joint configuration and then move by those steps.
   */

  // Logger::debug("Reaching joint configuration q1={}, q2={}, q3={}.", solution.q1, degrees(solution.q2), degrees(solution.q3));

  Steps steps = IKSolution2Steps(solution);

  // Find longest distance to cover
  int distance1 = abs(stepper1.getCurrentPosition() - steps.s1);
  int distance2 = abs(stepper2.getCurrentPosition() - steps.s2);
  int distance3 = abs(stepper3.getCurrentPosition() - steps.s3);

  float maxSteps = max(distance1, max(distance2, distance3)) * 1.0;

  // The motor that needs to move the most steps should move at full velocity and acceleration
  // while the other motor's velocity and acceleration are scaled proportionally to match 
  // the duration of the motion of the first one
  float scaleFactor1 = distance1 != 0 ? maxSteps / distance1 : 1;
  float scaleFactor2 = distance2 != 0 ? maxSteps / distance2 : 1;
  float scaleFactor3 = distance3 != 0 ? maxSteps / distance3 : 1;

  // Define velocities
  float v1i = restVelocity / scaleFactor1;
  float v2i = restVelocity / scaleFactor2;
  float v3i = restVelocity / scaleFactor3;

  float v1f = restVelocity / scaleFactor1;
  float v2f = restVelocity / scaleFactor2;
  float v3f = restVelocity / scaleFactor3;

  stepper1.moveToPosition(steps.s1, v1i, cruiseVelocity, v1f, acceleration);
  stepper2.moveToPosition(steps.s2, v2i, cruiseVelocity, v2f, acceleration);
  stepper3.moveToPosition(steps.s3, v3i, cruiseVelocity, v3f, acceleration);
 
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
  steps.s1 = -solution.q1 * JOINT_1_STEPS_PER_M;
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
    int homingSteps, 
    float initialVelocity = MIN_VELOCITY_STEPS_S, 
    float maxVelocity = MIN_VELOCITY_STEPS_S, 
    float finalVelocity = MIN_VELOCITY_STEPS_S,
    float acceleration = 1
  ) {
  /**
   * Home the specified axis by moving the motor with constant speed until button fires.
   * The motor stops after #homingSteps anyway. We can't use a tapezoidal speed profile
   * since we don't know in advance how many steps we need to take and thus we cannot
   * decelerate properly. We could gradually accelerate and until we reach the maximum
   * velocity but I prefer to cruise to the limit switch with minimum velocity in order 
   * to move safely to the target.
   */

  stepper.moveToPosition(homingSteps, initialVelocity, maxVelocity, finalVelocity, acceleration);
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
  homeAxis(stepper1, button1, MAX_HOMING_STEPS);
  Logger::debug("Axis 0 homed.");

  // homeAxis(stepper2, button2);
  Logger::debug("Axis 1 homed.");

  // homeAxis(stepper3, button3);
  Logger::debug("Axis 2 homed.");

  reachJoint(IKSolution(0.1, 0, 0));
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
