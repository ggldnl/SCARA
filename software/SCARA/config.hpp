#ifndef CONFIG_HPP
#define CONFIG_HPP

// CNC Shield pinout

const byte STEPPER_1_STEP_PIN = 12; // Axis A requires an optional jumper
const byte STEPPER_2_STEP_PIN = 3;
const byte STEPPER_3_STEP_PIN = 2;

const byte STEPPER_1_DIR_PIN = 13;  // Axis A requires an optional jumper
const byte STEPPER_2_DIR_PIN = 6;
const byte STEPPER_3_DIR_PIN = 5;

const byte ENABLE = 8;  // active-low (i.e. LOW turns on the drivers)

// Speed profile

// Low level control of the actuators
const int MIN_PULSE_DELAY = 200;
const int MAX_PULSE_DELAY = 800;
const int ACC_RATE = 800;
const int INCREMENT = 1;

// Velocity and acceleration

/*
 * I empirically found that with my combination of steppers and drivers
 * the stepper work best if we use a pulse with a frequency between
 * 400 and 800 microseconds. A shorter frequency results in a higher
 * velocity. The stepper class works with velocity and accelerations 
 * expressed in steps/s and steps/s^2 respectively. This means that 
 * we would have to use a minimum and maximum velocity in the range 
 * of 1250 to 2500 steps/s, according to the following computation:
 *
 * delay = 1000000/velocity
 * 400 = 1000000/x -> x = 2500
 * 800 = 1000000/x -> x = 1250
 */

const float MAX_VELOCITY_STEPS_S = 2500.0;
const float MIN_VELOCITY_STEPS_S = 1250.0;
const float ACCELERATION = 1000.0;

const float MAX_VELOCITY_DELAY = 400.0;
const float MIN_VELOCITY_DELAY = 800.0;

// Homing and limit switches

const int MAX_HOMING_STEPS = 10000;

const byte STEPPER_1_LIMIT_SWITCH_PIN = 9;
const byte STEPPER_2_LIMIT_SWITCH_PIN = 10;
const byte STEPPER_3_LIMIT_SWITCH_PIN = 11;

// Reduction ratios

const int STEPPER_1_MICROSTEPPING = 2;
const int STEPPER_2_MICROSTEPPING = 2;
const int STEPPER_3_MICROSTEPPING = 2;

const float STEPPER_1_STEPS_PER_REVOLUTION = 200.0;
const float STEPPER_2_STEPS_PER_REVOLUTION = 200.0;
const float STEPPER_3_STEPS_PER_REVOLUTION = 200.0;

const float JOINT_1_LEAD = 8.0;                                 // leadscrew pitch, mm
const float JOINT_2_REDUCTION = (72.0 / 16.0);                  // first stage
const float JOINT_3_REDUCTION = (62.0 / 16.0) * (62.0 / 33.0);  // first stage * second stage

const float JOINT_1_STEPS_PER_M = (STEPPER_1_STEPS_PER_REVOLUTION * STEPPER_1_MICROSTEPPING) / (JOINT_1_LEAD / 1000.0);           // steps/m
const float JOINT_2_STEPS_PER_RAD = (STEPPER_2_STEPS_PER_REVOLUTION * STEPPER_2_MICROSTEPPING / (2 * M_PI)) * JOINT_2_REDUCTION;  // steps/rad
const float JOINT_3_STEPS_PER_RAD = (STEPPER_3_STEPS_PER_REVOLUTION * STEPPER_3_MICROSTEPPING / (2 * M_PI)) * JOINT_3_REDUCTION;  // steps/rad

// Link lenghts

const float L1 = 0.094; // Link 2 length in mm
const float L2 = 0.080; // Link 3 length in mm

// Joint limits

const float JOINT_1_MIN_LIMIT = 0;
const float JOINT_1_MAX_LIMIT = 0.2;  // 200 mm, 0.2 m

// The second joint angle should be within the [-pi/2, +pi/2] range
const float JOINT_2_MIN_LIMIT = -M_PI / 2;
const float JOINT_2_MAX_LIMIT = M_PI / 2;

// The third joint angle should be within the [-8pi/9, +8pi/9] range
const float JOINT_3_MIN_LIMIT = -(8 * M_PI) / 9;
const float JOINT_3_MAX_LIMIT = (8 * M_PI) / 9;

#endif // CONFIG_HPP