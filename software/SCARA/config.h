#ifndef CONFIG_H
#define CONFIG_H

// CNC Shield pinout

const byte MOTOR_1_STEP_PIN = 12; // Axis A requires an optional jumper
const byte MOTOR_2_STEP_PIN = 2;
const byte MOTOR_3_STEP_PIN = 3;

const byte MOTOR_1_DIR_PIN = 13;  // Axis A requires an optional jumper
const byte MOTOR_2_DIR_PIN = 5;
const byte MOTOR_3_DIR_PIN = 6;

const byte ENABLE = 8;  // active-low (i.e. LOW turns on the drivers)

// Speed 

const float MOTOR_1_SPEED = 2000.0;
const float MOTOR_2_SPEED = 2000.0;
const float MOTOR_3_SPEED = 2000.0;

const float MOTOR_1_HOMING_SPEED = 1000.0;
const float MOTOR_2_HOMING_SPEED = 1000.0;
const float MOTOR_3_HOMING_SPEED = 1000.0;

const int POST_HOMING_STEPS = 250;

// Ramp

const int MOTOR_1_RAMP_LENGTH = 0;
const int MOTOR_2_RAMP_LENGTH = 0;
const int MOTOR_3_RAMP_LENGTH = 0;

const int MOTOR_1_HOMING_RAMP_LENGTH = 0;
const int MOTOR_2_HOMING_RAMP_LENGTH = 0;
const int MOTOR_3_HOMING_RAMP_LENGTH = 0;

// Endstops

const byte MOTOR_1_LIMIT_SWITCH_PIN = 9;
const byte MOTOR_2_LIMIT_SWITCH_PIN = 10;
const byte MOTOR_3_LIMIT_SWITCH_PIN = 11;

// Microstepping

const byte MOTOR_1_MICROSTEPPING = 1;
const byte MOTOR_2_MICROSTEPPING = 1;
const byte MOTOR_3_MICROSTEPPING = 1;

// Steps per revolution

const int MOTOR_1_STEPS_PER_REVOLUTION = 200;
const int MOTOR_2_STEPS_PER_REVOLUTION = 200;
const int MOTOR_3_STEPS_PER_REVOLUTION = 200;

// Reduction ratios

const int JOINT_1_REDUCTION = MOTOR_1_STEPS_PER_REVOLUTION / 4;  // steps_per_revolution / leadscrew_pitch
const int JOINT_3_REDUCTION = MOTOR_3_STEPS_PER_REVOLUTION * (62 / 16) * (62 / 33);  // steps_per_revolution * reduction_1 * reduction_2
const int JOINT_2_REDUCTION = MOTOR_2_STEPS_PER_REVOLUTION * 1;

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

#endif // CONFIG_H