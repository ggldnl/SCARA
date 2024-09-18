#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

class StepperMotor {
public:
    // Constructor with pins and max acceleration
    StepperMotor(uint8_t pulPin, uint8_t dirPin)
        : pulPin_(pulPin), dirPin_(dirPin), currentPosition_(0), targetPosition_(0), stepState_(LOW) {
        pinMode(pulPin_, OUTPUT);
        pinMode(dirPin_, OUTPUT);
        digitalWrite(pulPin_, LOW);
        digitalWrite(dirPin_, LOW);
    }

    // Set the target position
    void setTargetPosition(int targetPosition) {
        targetPosition_ = targetPosition;
    }

    // Set the current position (e.g. after homing)
    void setCurrentPosition(int currentPosition) {
        currentPosition_ = currentPosition;
    }

    // Perform a single step
    bool step() {
        if (currentPosition_ == targetPosition_) {
            return false; // No need to move
        }

        // Set direction based on whether to move forward or backward
        if (currentPosition_ < targetPosition_) {
            digitalWrite(dirPin_, HIGH);  // Move forward
            currentPosition_++;
        } else {
            digitalWrite(dirPin_, LOW);   // Move backward
            currentPosition_--;
        }

        // Toggle the pulse pin between HIGH and LOW
        stepState_ = !stepState_;
        digitalWrite(pulPin_, stepState_);

        return true; // A step was made
    }

    // Get the current position of the stepper motor
    int getCurrentPosition() const {
        return currentPosition_;
    }

    // Check if the motor has reached its target position
    bool isAtTarget() const {
        return currentPosition_ == targetPosition_;
    }

private:
    uint8_t pulPin_;        // Pin for pulse signal
    uint8_t dirPin_;        // Pin for direction signal
    int currentPosition_;   // Current position (in steps) of the stepper
    int targetPosition_;    // Target position (in steps) to reach
    bool stepState_;        // Boolean to alternate between HIGH and LOW for pulsing
};

#endif // STEPPER_MOTOR_HPP
