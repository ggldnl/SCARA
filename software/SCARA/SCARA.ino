#include "config.hpp"
#include "logger.hpp"
#include "button.hpp"
#include "vector.hpp"
#include "stepper.hpp"
#include "structs.hpp"
#include "kinematics.hpp"

// Function declarations
Vector<float> convertVelocity(Vector<float>&);
void executeTrajectory(Vector<Point>&, Vector<float>&, const float = MIN_VELOCITY_STEPS_S, const float = MAX_VELOCITY_STEPS_S, const float = ACCELERATION);
void reachCartesian(const Point&);
void reachJoint(const IKSolution&);
void moveAll();
Steps IKSolution2Steps(const IKSolution&);


// Define the steppers
StepperMotor stepper1(STEPPER_1_STEP_PIN, STEPPER_1_DIR_PIN);
StepperMotor stepper2(STEPPER_2_STEP_PIN, STEPPER_2_DIR_PIN);
StepperMotor stepper3(STEPPER_3_STEP_PIN, STEPPER_3_DIR_PIN);

// Define the buttons
Button button1(STEPPER_1_LIMIT_SWITCH_PIN);
Button button2(STEPPER_2_LIMIT_SWITCH_PIN);
Button button3(STEPPER_3_LIMIT_SWITCH_PIN);

// Define the trajectory to follow and the desired end effector velocity (m/s)
Vector<Point> trajectory;
Vector<float> velocity;

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
  digitalWrite(ENABLE, LOW);
  Logger::info("Board enabled.");

  // Home axis
  // homeAll();
  Logger::info("All axis homed.");

  /* --------------------------- Execute trajectory --------------------------- */

  // Add the points to the trajectory (square)
  /*
  trajectory.pushBack(Point(L1 + L2, 0.0, 0.10000));  // Reach the height with the arm fully stretched
  trajectory.pushBack(Point(0.15536, 0.03536, 0.10000));
  trajectory.pushBack(Point(0.08464, 0.03536, 0.10000));
  trajectory.pushBack(Point(0.08464, -0.03536, 0.10000));
  trajectory.pushBack(Point(0.15536, -0.03536, 0.10000));
  */

  trajectory.pushBack(Point(0.10, -0.14, 0.1));
  trajectory.pushBack(Point(0.10, 0.0, 0.1));;
  trajectory.pushBack(Point(0.10, 0.14, 0.1));

  /*
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

  /*
  // End effector velocity of 0.025 m/s in all directions
  velocity.pushBack(0.025);
  velocity.pushBack(0.025);
  velocity.pushBack(0.025);
  */

  velocity.pushBack(0.01);
  velocity.pushBack(0.01);
  velocity.pushBack(0.01);

  executeTrajectory(trajectory, velocity);

}

/* ------------------------------ Trajectories ------------------------------ */

Vector<float> convertVelocity(Vector<float>& velocity) {
  /**
   * Converts the input velocity vector from m/s (or rad/s) to steps/s
   */
  Vector<float> vector(3);

  // Absolute values since the stepper motor library will use the position to 
  // determine the direction of rotation 
  vector.pushBack(abs(velocity[0] * JOINT_1_STEPS_PER_M));
  vector.pushBack(abs(velocity[1] * JOINT_2_STEPS_PER_RAD));
  vector.pushBack(abs(velocity[2] * JOINT_3_STEPS_PER_RAD));
  
  return vector;
}

void executeTrajectory(
    Vector<Point>& trajectory, 
    Vector<float>& velocity, 
    const float restVelocity = MIN_VELOCITY_STEPS_S, 
    const float maxVelocity = MAX_VELOCITY_STEPS_S, 
    const float acceleration = ACCELERATION
  ) {

  Vector<Steps> trajectorySteps;
  Vector<Vector<float>> velocities;
  Logger::debug("-------------------------------------------------");
  for (size_t i = 0; i < trajectory.getSize(); i++) {

    Point point = trajectory[i];
    Logger::debug("Point {}: ({}, {}, {})", i + 1, point.x, point.y, point.z);

    // Compute the inverse kinematics
    IKSolution configuration;
    bool result = inverseKinematics(point, configuration);
    if (!result) {
      Logger::error("Point ({}, {}, {}) is not reachable.", point.x, point.y, point.z);
      return;
    }
    Logger::debug("Inverse kinematics solution: {}, {}, {}", configuration.q1, configuration.q2, configuration.q3);

    // Compute the steps it takes to reach the joint configuration
    Steps steps = IKSolution2Steps(configuration);
    Logger::debug("Steps needed: {}, {}, {}", steps.s1, steps.s2, steps.s3);

    // Buffer the computation
    trajectorySteps.pushBack(steps);

    // Transform the end effector velocity to joint velocity
    Vector<float> jointVelocities = computeJointVelocities(configuration, velocity);

    // Express the joint velocities in steps/s (from m/s or rad/s)
    Vector<float> translatedVelocities = convertVelocity(jointVelocities);
    Logger::debug("Joint velocities (steps/s): {}, {}, {}", translatedVelocities[0], translatedVelocities[1], translatedVelocities[2]);

    // Buffer the computation
    velocities.pushBack(translatedVelocities);

    Logger::debug("-------------------------------------------------");
  }

  for (size_t i = 0; i < trajectorySteps.getSize(); i++) {

    Steps steps = trajectorySteps[i];

    // Same number of elements
    Vector<float> velocity = velocities[i];

    float v1 = i == restVelocity ? 0 : velocity[0];
    float v2 = i == restVelocity ? 0 : velocity[1];
    float v3 = i == restVelocity ? 0 : velocity[2];

    stepper1.moveToPosition(steps.s1, v1, maxVelocity, velocity[0], acceleration);
    stepper2.moveToPosition(steps.s2, v2, maxVelocity, velocity[1], acceleration);
    stepper3.moveToPosition(steps.s3, v3, maxVelocity, velocity[2], acceleration);

    moveAll();
  }
}

/* ------------------------------ Single point ------------------------------ */

void reachCartesian(const Point& p) {
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

void reachJoint(const IKSolution& solution) {
  /**
   * Reach the target joint configuration. Compute the steps required to reach
   * the joint configuration and then move by those steps.
   */

  // Logger::debug("Reaching joint configuration q1={}, q2={}, q3={}.", solution.q1, degrees(solution.q2), degrees(solution.q3));

  Steps steps = IKSolution2Steps(solution);

  float initialVelocity = MIN_VELOCITY_STEPS_S;
  float maxVelocity = MAX_VELOCITY_STEPS_S;
  float finalVelocity = MIN_VELOCITY_STEPS_S;
  float acceleration = ACCELERATION;

  stepper1.moveToPosition(steps.s1, initialVelocity, maxVelocity, finalVelocity, acceleration);
  stepper2.moveToPosition(steps.s2, initialVelocity, maxVelocity, finalVelocity, acceleration);
  stepper3.moveToPosition(steps.s3, initialVelocity, maxVelocity, finalVelocity, acceleration);

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
  homeAxis(stepper1, button1, -10000);
  Logger::debug("Axis 0 homed.");

  // homeAxis(stepper2, button2);
  Logger::debug("Axis 1 homed.");

  // homeAxis(stepper3, button3);
  Logger::debug("Axis 2 homed.");

  reachJoint(IKSolution(0.1, 0, 0));
}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {
  // Nothing to do here
}
