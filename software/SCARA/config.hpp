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

const int STEPPER_1_MAX_ACCEL = 800;
const int STEPPER_2_MAX_ACCEL = 800;
const int STEPPER_3_MAX_ACCEL = 800;

const int MIN_PULSE_DELAY = 200;
const int MAX_PULSE_DELAY = 800;
const int INCREMENT = 1;

// Endstops

const byte STEPPER_1_LIMIT_SWITCH_PIN = 9;
const byte STEPPER_2_LIMIT_SWITCH_PIN = 10;
const byte STEPPER_3_LIMIT_SWITCH_PIN = 11;

// Homing

const int HOMING_STEPS = 10000;
const int POST_HOMING_STEPS = 250;

// Reduction ratios

const int STEPPER_1_MICROSTEPPING = 2;
const int STEPPER_2_MICROSTEPPING = 2;
const int STEPPER_3_MICROSTEPPING = 2;

const float STEPPER_1_STEPS_PER_REVOLUTION = 200.0;
const float STEPPER_2_STEPS_PER_REVOLUTION = 200.0;
const float STEPPER_3_STEPS_PER_REVOLUTION = 200.0;

const float JOINT_1_LEAD = 8.0;                                 // leadscrew pitch
const float JOINT_2_REDUCTION = (72.0 / 16.0);                  // first stage
const float JOINT_3_REDUCTION = (62.0 / 16.0) * (62.0 / 33.0);  // first stage * second stage

const float JOINT_1_DIST_PER_STEP = (JOINT_1_LEAD / (STEPPER_1_STEPS_PER_REVOLUTION * STEPPER_1_MICROSTEPPING));
const float JOINT_2_STEPS_PER_RAD = (STEPPER_2_STEPS_PER_REVOLUTION * STEPPER_2_MICROSTEPPING / (2 * M_PI)) * JOINT_2_REDUCTION; 
const float JOINT_3_STEPS_PER_RAD = (STEPPER_3_STEPS_PER_REVOLUTION * STEPPER_3_MICROSTEPPING / (2 * M_PI)) * JOINT_3_REDUCTION;

// Link lenghts

const float L1 = 94; // Link 2 length in mm
const float L2 = 80; // Link 3 length in mm

// Joint limits

const float JOINT_1_MIN_LIMIT = 0;
const float JOINT_1_MAX_LIMIT = 200;  // 200 mm

const float JOINT_2_MIN_LIMIT = 0;
const float JOINT_2_MAX_LIMIT = 90;

const float JOINT_3_MIN_LIMIT = 0;
const float JOINT_3_MAX_LIMIT = 3.14;

#endif // CONFIG_HPP