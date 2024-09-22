#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP


class StepperMotor {
public:

    StepperMotor(uint8_t pulPin, uint8_t dirPin)
        : pulPin_(pulPin), dirPin_(dirPin), currentPosition_(0), targetPosition_(0),
          stepState_(LOW), velocity_(0), acceleration_(0), lastStepTime_(0), stepInterval_(0),
          maxVelocity_(0), finalVelocity_(0), stepsAccel_(0), stepsDecel_(0), stepsConstant_(0) {

        // Setup the pins
        pinMode(pulPin_, OUTPUT);
        pinMode(dirPin_, OUTPUT);
        digitalWrite(pulPin_, LOW);
        digitalWrite(dirPin_, LOW);
    }

    void moveToPosition(int targetPosition, float initialVelocity, float maxVelocity, float finalVelocity, float acceleration) {
        /**
         * Set the target position, maximum velocity, final velocity, and acceleration.
         * Once we call this method it will compute the constants to 
         */

        targetPosition_ = targetPosition;
        velocity_ = initialVelocity;
        maxVelocity_ = maxVelocity;
        finalVelocity_ = finalVelocity;
        acceleration_ = acceleration;
        lastStepTime_ = micros();  // Initialize the step timer

        // Prevent negative velocity and division by 0
        if (velocity_ <= 0) velocity_ = 0.01;

        // Calculate acceleration and deceleration steps
        stepsAccel_ = computeSteps(velocity_, maxVelocity_, acceleration_);
        stepsDecel_ = computeSteps(finalVelocity_, maxVelocity_, acceleration_);

        // Adjust max velocity if there aren't enough steps to reach the desired speed
        if (stepsAccel_ + stepsDecel_ > abs(targetPosition_ - currentPosition_)) {

            // Compute the maximum velocity achievable given acceleration and distance to cover.
            maxVelocity_ = sqrt(velocity_ * velocity_ + acceleration_ * abs(targetPosition_ - currentPosition_));
          
            stepsAccel_ = computeSteps(velocity_, maxVelocity_, acceleration_);
            stepsDecel_ = computeSteps(finalVelocity_, maxVelocity_, acceleration_);
        }

        // Remaining steps are at constant velocity
        stepsConstant_ = abs(targetPosition_ - currentPosition_) - stepsAccel_ - stepsDecel_;

        // Compute the initial step interval
        stepInterval_ = 1000000.0 / velocity_;

    }

    bool step() {
        /**
         * Perform a single step if we are not at target position yet.
         * Use the with trapezoidal speed profile to compute the velocity
         * (delay between pulses).
         */

        if (currentPosition_ == targetPosition_) {
            return false;  // No need to move
        }

        unsigned long currentTime = micros();
        unsigned long timeSinceLastStep = currentTime - lastStepTime_;

        if (timeSinceLastStep >= stepInterval_) {
            // Set direction based on target position
            if (currentPosition_ < targetPosition_) {
                digitalWrite(dirPin_, HIGH);  // Move forward
                currentPosition_++;
            } else {
                digitalWrite(dirPin_, LOW);   // Move backward
                currentPosition_--;
            }

            // Toggle the pulse pin
            stepState_ = !stepState_;
            digitalWrite(pulPin_, stepState_);

            // We made a step, update the timestamp of the last step taken
            lastStepTime_ = currentTime;

            // Adjust the velocity if in acceleration phase
            if (currentPosition_ < stepsAccel_) {
                velocity_ += acceleration_ * (timeSinceLastStep / 1000000.0);  // v = u + at
                if (velocity_ > maxVelocity_) velocity_ = maxVelocity_; // Cap the velocity if it's over the maximum
            }
            // Adjust the velocity if in constant velocity phase
            else if (currentPosition_ >= stepsAccel_ && currentPosition_ < stepsAccel_ + stepsConstant_) {
                velocity_ = maxVelocity_;  // Maintain constant velocity
            }
            // Adjust the velocity if in deceleration phase
            else if (currentPosition_ >= stepsAccel_ + stepsConstant_) {
                velocity_ -= acceleration_ * (timeSinceLastStep / 1000000.0);  // v = u - at
                if (velocity_ < finalVelocity_) velocity_ = finalVelocity_; // Cap the velocity if it's below the minimum
            }

            // Prevent negative velocity and division by 0
            if (velocity_ <= 0) velocity_ = 0.01;

            // Compute new delay interval for next step
            stepInterval_ = 1000000.0 / velocity_;
        }

        return true;
    }

    bool isAtTarget() const {
        /**
         * Check if the motor has reached its target position.
         */
        return currentPosition_ == targetPosition_;
    }

    int getCurrentPosition() const {
        /**
         * Get the current position of the stepper motor.
         */
        return currentPosition_;
    }

    void setCurrentPosition(const int currentPosition) {
        currentPosition_ = currentPosition;
    }

    int getTargetPosition() const {
        /**
         * Get the target position of the stepper motor.
         */
    }

    float getCurrentVelocity() const {
        /** 
         * Get the current velocity of the stepper motor.
         */
        return velocity_;
    }

    float getTargetVelocity() const {
        /** 
         * Get the target velocity of the stepper motor.
         */
        return finalVelocity_;
    }

    float getCurrentDelay() const {
        /**
         * Return the current delay.
         */
        return 1000000.0 / velocity_;
    }

    float getAcceleration() const {
        /** 
         * Get the acceleration of the stepper motor.
         */
        return acceleration_;
    }

private:

    uint8_t pulPin_;         // Pin for pulse signal
    uint8_t dirPin_;         // Pin for direction signal

    int currentPosition_;    // Current position (in steps)
    int targetPosition_;     // Target position (in steps)

    // Alternates between HIGH and LOW for pulsing so we can repeatedly call the step() method
    bool stepState_;
    
    float velocity_;         // Current velocity (steps per second)
    float finalVelocity_;    // Final velocity (steps per second)
    float maxVelocity_;      // Maximum velocity (steps per second)
    float acceleration_;     // Acceleration (steps per second^2)

    unsigned long lastStepTime_;  // Last time a step was taken (in microseconds)
    unsigned long stepInterval_;  // Time interval between steps (in microseconds)

    int stepsAccel_;         // Steps for acceleration phase
    int stepsConstant_;      // Steps for constant velocity phase
    int stepsDecel_;         // Steps for deceleration phase

    int computeSteps(float startVelocity, float endVelocity, float accel) {
        /**
         * Helper function to compute the steps needed to reach a target velocity (expressed in steps/s).
         */
        return (int)((endVelocity * endVelocity - startVelocity * startVelocity) / (2.0 * accel));
    }
};

#endif // STEPPER_MOTOR_HPP
