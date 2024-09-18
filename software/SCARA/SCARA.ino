#include "config.hpp"
#include "logger.hpp"
#include "button.hpp"
#include "vector.hpp"
#include "stepper.hpp"
#include "structs.hpp"
#include "kinematics.hpp"


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
  digitalWrite(ENABLE, LOW);
  Logger::info("Board enabled.");

  // Home axis
  homeAll();
  Logger::info("All axis homed.");

  /* -------------------------- Reach cartesian point ------------------------- */

  /*
  Point target(0.120, 0.020, 0.100);  // m
  reachCartesian(target);
  */

  /* --------------------------- Execute trajectory --------------------------- */

  // Add the points to the trajectory (square)
  trajectory.push_back(Point(0.15536, 0.03536, 0.10000));
  trajectory.push_back(Point(0.08464, 0.03536, 0.10000));
  trajectory.push_back(Point(0.08464, -0.03536, 0.10000));
  trajectory.push_back(Point(0.15536, -0.03536, 0.10000));

  float velocity = 0.25e-4;
  executeTrajectory(trajectory, velocity);

  /* ------------------------------ Go back home ------------------------------ */

  delay(2000);
  reset();
}

/* ------------------------------ Trajectories ------------------------------ */

void executeTrajectory(Vector<Point> trajectory, float velocity) {
  /**
   * Execute a trajectory (sequence of points), with a given end effector velocity:
   *
   * 1. For each point, compute the inverse kinematics solution.
   *
   * 2. Each inverse kinematics solution consists of the joint values (3 in our case).
   *
   * 3. We want the end effector to maintain the specified velocity. The velocity of the 
   *      end effector is tied to the velocity of the joints by the inverse of the Jacobian.
   *
   * 4. For each inverse kinematics solution (configuration of the arm in joint space) we
   *      compute the Jacobian and then the joint velocities:
   *
   *      v_ee = J * v_j -> v_j = J^-1 * v_ee
   *
   * 5. Now we have the velocity vector for each joint at each point of the trajectory.
   *
   * 6. Each joint velocity is expressed in m/s (for prismatic joints) or rad/s (for 
   *      revolute joints). We need to translate them to steps/s to control the stepper motors.
   *
   * 7. steps/s conversion:
   *
   *        prismatic -> v [steps/s] = v [m/s] / (distance_per_steps [mm] * 0.001) 
   *        revolute -> v [steps/s] = v [rad/s] * steps_per_rad [steps/rad]
   *      
   *      distance_per_step is JOINT_1_DIST_PER_STEP in our case
   *      steps_per_rad are JOINT_2_STEPS_PER_RAD and JOINT_3_STEPS_PER_RAD in our case
   *
   * 8. Now we have the steps/s velocity for each joint. We work at a microsecond scale:
   *
   *      steps_per_microseond = steps_per_second / 1000000
   * 
   * 9. In our case, for each stepper, it's the delay between two steps that determines
   *      the velocity. The smaller the delay, the faster the movement and vice versa.
   *      We know how many steps each joint should take in a second (microsecond to be
   *      precise) but we need to know how many microseconds pass between two stesp:
   *
   *      delay_between_steps = 1000000 / steps_per_second = 1 / steps_per_microsecond
   *
   * 10. Now we know the delay that we should use while moving the steppers to realize 
   *      a certain end effector velocity. Buffer all these values for each start and 
   *      final point couple and then use them to perform the trajectory. This way we 
   *      compute everything beforehand and we don't waste time during the movement
   *      with computations. 
   */

  const float maxVelocity = MAX_VELOCITY;
  const float accRate = ACCELERATION;

  Vector<Steps> trajectorySteps;
  for (size_t i = 0; i < trajectory.getSize(); i++) {

    Point point = trajectory[i];

    // Compute the inverse kinematics
    IKSolution solution;
    bool result = inverseKinematics(point, solution);
    if (!result) {
      Logger::error("Point ({}, {}, {}) is not reachable.", point.x, point.y, point.z);  
      return;
    }

    // Compute the steps it takes to reach the joint configuration
    Steps steps = IKSolution2Steps(solution);

    // Buffer the computation
    trajectorySteps.push_back(steps);
  }

  for (size_t i = 0; i < trajectorySteps.getSize() - 1; i++) {
  
    Steps startSteps = trajectorySteps[i];
    Steps endSteps = trajectorySteps[i + 1];

    float initialVelocity = i == 0 ? 0 : velocity;
    float finalVelocity = velocity;

    moveStraightLine(startSteps, endSteps, initialVelocity, finalVelocity, maxVelocity, accRate);
  }
}

void moveStraightLine(const Steps& startSteps, const Steps& endSteps, float vinit, float vfinal, float vmax, float acc) {
  /**
   * Move in a straight line from the current position (in steps) to the final position (in steps). 
   * We follow a trapezoidal speed profile. In this scenario, motion is divided into three phases:
   *
   * 1. Acceleration phase: the velocity increases linearly from the initial velocity (vinit) to the
   *    maximum velocity (vmax) at a constant acceleration (acc);
   *
   * 2. Constant velocity phase: the velocity remains constant at its peek (vmax);
   *
   * 3. Deceleration phase: the velocity decreases from the maximum (vmax) to the final value (vfin)
   *    at a constant deceleration (acc).
   *
   * If the distance to cover is too short for a constant velocity phase, the motion becomes triangular
   * (no constant velocity phase). In this case the maximum velocity (vmax) is not reached.
   *
   * The distance covered during acceleration (dacc), constant velocity (dconst) and deceleration (ddec)
   * phases can be found this way:
   *
   * 1. dacc   = (vmax^2 - vinit^2) / (2*acc)
   *
   * 2. ddec   = (vmax^2 - vfin^2) / (2*acc)
   *
   * 3. dconst = L - dacc - ddec
   *
   * with L being the distance to cover (length of the linear path).
   * 
   * We take only the dacc, dconst and ddec as arguments and move all the steppers accordingly,
   * without performing any computation during movement other than a simple linear
   * interpolation and some multiplication. The input velocities are specified
   * in steps/s and the acceleration in steps/s^2.
   *
   * We don’t need intermediate velocity computations since the trapezoidal velocity profile 
   * will handle the transition from initial to final velocities within each segment.
   */

  Logger::debug("Steps {}, {}, {} -> {}, {}, {}", startSteps.s1, startSteps.s2, startSteps.s3, endSteps.s1, endSteps.s2, endSteps.s3);

  // Calculate the total number of steps to move in each joint
  int steps_total_1 = abs(endSteps.s1 - startSteps.s1);
  int steps_total_2 = abs(endSteps.s2 - startSteps.s2);
  int steps_total_3 = abs(endSteps.s3 - startSteps.s3);

  stepper1.setTargetPosition(endSteps.s1);
  stepper2.setTargetPosition(endSteps.s2);
  stepper3.setTargetPosition(endSteps.s3);

  // Compute the maximum number of steps for synchronization
  int max_steps = max(steps_total_1, max(steps_total_2, steps_total_3));

  // Compute the distances covered during acceleration and deceleration
  float d_acc = (vmax * vmax - vinit * vinit) / (2 * acc);
  float d_dec = (vmax * vmax - vfinal * vfinal) / (2 * acc);
  float d_const = max_steps - d_acc - d_dec;

  if (max_steps < d_acc + d_dec) {
    d_const = 0;
    d_acc = max_steps / 2;
    d_dec = max_steps / 2;
  }

  // Compute delays for each phase (in microseconds)
  int delay_init = 1000000 / vinit;
  int delay_max = 1000000 / vmax;
  int delay_final = 1000000 / vfinal;

  Logger::debug("Acc {} | Const {} | Dec {}", d_acc, d_const, d_dec);

  int stepCount = 0;
  
  // Acceleration phase
  for (int i = 0; i < d_acc; i++) {

    // Increase speed linearly
    int d = delay_init - ((delay_init - delay_max) * i / d_acc);
    
    Logger::debug("Acceleration: {}", d);

    stepper1.step();
    stepper2.step();
    stepper3.step();
    
    delayMicroseconds(d);
    stepCount++;
  }
  
  // Constant velocity phase
  for (int i = 0; i < d_const; i++) {

    Logger::debug("Const: {}", delay_max);

    stepper1.step();
    stepper2.step();
    stepper3.step();
    
    delayMicroseconds(delay_max);  // Move at constant speed
    stepCount++;
  }

  // Deceleration phase
  for (int i = 0; i < d_dec; i++) {

    // Decrease speed linearly
    int d = delay_max + ((delay_final - delay_max) * i / d_dec);

   Logger::debug("Deceleration: {}", d);

    stepper1.step();
    stepper2.step();
    stepper3.step();

    delayMicroseconds(d);
    stepCount++;
  }

  Logger::debug("\n");

}

/* ------------------------------ Single point ------------------------------ */

/**
 * Put the arm in a certain configuration without velocity control.
 */

void moveAll(
    const Steps& steps, 
    int minDelay = MIN_PULSE_DELAY,
    int maxDelay = MAX_PULSE_DELAY, 
    int accRate = ACC_RATE, 
    int incr = INCREMENT
  ) {
    /**
     * Low level function to move all the steppers simultaneously by the specified steps.
     * The arm will try to use a trapezoidal speed profile. If not possible, it will
     * accelerate if possible for half the space and then decelerate.
     */

  // Logger::debug("Moving to s1={}, s2={}, s3={}.", steps.s1, steps.s2, steps.s3);

  stepper1.setTargetPosition(steps.s1);
  stepper2.setTargetPosition(steps.s2);
  stepper3.setTargetPosition(steps.s3);

  // Determine the total steps (maximum) to synchronize the movement
  int maxTotalSteps = max(steps.s1, max(steps.s2, steps.s3));

  int del = maxDelay;  // Maximum delay = minimum acceleration
  for (int i = 0; i < maxTotalSteps; i++) {
    
    // We need to take more steps than the steps required for an acceleration and deceleration phase
    if (maxTotalSteps > (2 * accRate + 1)) {
      if (i < accRate && del - incr >= minDelay) {
        // Acceleration phase
        del -= incr;
      } else if (i > (maxTotalSteps - accRate) && del + incr <= maxDelay) {
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

void reachJoint(const IKSolution& solution) {
  /**
   * Reach the target joint configuration:
   * 1. Translate joint values to steps;
   * 2. Move by the steps;
   */

  // Logger::debug("Reaching joint configuration q1={}, q2={}, q3={}.", solution.q1, degrees(solution.q2), degrees(solution.q3));

  Steps steps = IKSolution2Steps(solution);
  moveAll(steps);

  // Logger::debug("Current position: p1={}, p2={}, p3={}", stepper1.getCurrentPosition(), stepper2.getCurrentPosition(), stepper3.getCurrentPosition());
}

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

Steps IKSolution2Steps(const IKSolution& solution) {
  /**
   * Convert the inverse kinematics solution to steps for each motor 
   * by applying reduction factors and compensations.
   */ 

  Steps steps;
  steps.s1 = solution.q1 / JOINT_1_DIST_PER_STEP;
  steps.s2 = solution.q2 * JOINT_2_STEPS_PER_RAD;
  
  // The third joint is a special case since we need to adjust it by a factor
  // to account for the variation produced by the movement of the second joint.
  steps.s3 = solution.q3 * JOINT_3_STEPS_PER_RAD;
  steps.s3 += solution.q2/2 * JOINT_3_STEPS_PER_RAD;

  return steps;
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
      Logger::debug("Endstop {} reached.", limitSwitch.getPin());
      break;
    }

    // Perform a step with constant velocity
    stepper.step();
    delayMicroseconds(del);
  }

  /*
  stepper.setTargetPosition(postHomingSteps);

  while(!stepper.isAtTarget()) {
    stepper.step();
    delayMicroseconds(del);
  }
  */

  stepper.setCurrentPosition(0);
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

  reachJoint(IKSolution(0.1, 0, 0));
}

void reset() {
  /**
   * Reset to initial position.
   */
   reachJoint(IKSolution(0.1, 0, 0)); // prismatic joint displaced by 10 cm
   Logger::debug("Position reset to q1={}, q2={}, q3={}", 0.1, 0, 0);
}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {
  // Nothing to do here
}
