#ifndef CONFIG_HPP
#define CONFIG_HPP

// CNC Shield pinout

const byte STEPPER_1_STEP_PIN = 12; // Axis A requires an optional jumper
const byte STEPPER_2_STEP_PIN = 2;
const byte STEPPER_3_STEP_PIN = 3;

const byte STEPPER_1_DIR_PIN = 13;  // Axis A requires an optional jumper
const byte STEPPER_2_DIR_PIN = 5;
const byte STEPPER_3_DIR_PIN = 6;

const byte ENABLE = 8;  // active-low (i.e. LOW turns on the drivers)

// Speed profile

const int STEPPER_1_MAX_ACCEL = 250;
const int STEPPER_2_MAX_ACCEL = 250;
const int STEPPER_3_MAX_ACCEL = 250;

const int BASE_DELAY = 800;
const int INCREMENT = 10;

// Endstops

const byte STEPPER_1_LIMIT_SWITCH_PIN = 9;
const byte STEPPER_2_LIMIT_SWITCH_PIN = 10;
const byte STEPPER_3_LIMIT_SWITCH_PIN = 11;

// Homing

const int HOMING_STEPS = 10000;
const int POST_HOMING_STEPS = 250;

// Reduction ratios

const byte STEPPER_1_MICROSTEPPING = 1;
const byte STEPPER_2_MICROSTEPPING = 1;
const byte STEPPER_3_MICROSTEPPING = 1;

const int STEPPER_1_STEPS_PER_REVOLUTION = 200;
const int STEPPER_2_STEPS_PER_REVOLUTION = 200;
const int STEPPER_3_STEPS_PER_REVOLUTION = 200;

const int JOINT_1_REDUCTION = STEPPER_1_STEPS_PER_REVOLUTION / 4;  // steps_per_revolution / leadscrew_pitch
const int JOINT_3_REDUCTION = STEPPER_3_STEPS_PER_REVOLUTION * (62 / 16) * (62 / 33);  // steps_per_revolution * reduction_1 * reduction_2
const int JOINT_2_REDUCTION = STEPPER_2_STEPS_PER_REVOLUTION * 1;

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