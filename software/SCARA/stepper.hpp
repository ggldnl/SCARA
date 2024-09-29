#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP


class SpeedProfile {
  public:
    virtual void compute(long totalSteps, double initialVelocity, double finalVelocity, double maxVelocity = 0, double acceleration = 0) = 0;
    virtual double update(long currentStep) = 0;
};

class TrapezoidalSpeedProfile : public SpeedProfile {
  private:
    double maxVelocity;
    double initialVelocity, finalVelocity, acceleration;
    long accelSteps, decelSteps, constSteps, totalSteps;

  public:
    void compute(long _totalSteps, double _initialVelocity, double _finalVelocity, double _maxVelocity = 0, double _acceleration = 0) override {
      totalSteps = _totalSteps;
      initialVelocity = _initialVelocity;
      maxVelocity = _maxVelocity;
      finalVelocity = _finalVelocity;
      acceleration = _acceleration;

      // Calculate accelSteps, decelSteps, etc.
      accelSteps = abs((maxVelocity * maxVelocity - initialVelocity * initialVelocity) / (2 * acceleration));
      decelSteps = abs((maxVelocity * maxVelocity - finalVelocity * finalVelocity) / (2 * acceleration));
      constSteps = totalSteps - (accelSteps + decelSteps);

      // Fallback to triangular profile if necessary
      if (accelSteps + decelSteps > totalSteps) {
          maxVelocity = sqrt(acceleration * totalSteps + 0.5 * (initialVelocity * initialVelocity + finalVelocity * finalVelocity));
          accelSteps = abs((maxVelocity * maxVelocity - initialVelocity * initialVelocity) / (2 * acceleration));
          decelSteps = totalSteps - accelSteps;
          constSteps = 0;
      }
    }

    double update(long currentStep) override {
      // Handle the trapezoidal speed profile update logic
      if (currentStep < accelSteps) {
        return sqrt(initialVelocity * initialVelocity + 2 * acceleration * currentStep);
      } else if (currentStep < totalSteps - decelSteps) {
        return maxVelocity;
      } else {
        return sqrt(finalVelocity * finalVelocity + 2 * acceleration * (totalSteps - currentStep));
      }
    }
};

class LinearSpeedProfile : public SpeedProfile {
  private:
    double initialVelocity, finalVelocity, increment;
    long totalSteps;

  public:
    void compute(long _totalSteps, double _initialVelocity, double _finalVelocity, double _maxVelocity = 0, double _acceleration = 0) override {
      totalSteps = _totalSteps;
      initialVelocity = _initialVelocity;
      finalVelocity = _finalVelocity;

      // Calculate increment for linear velocity change
      increment = (finalVelocity - initialVelocity) / totalSteps;
    }

    double update(long currentStep) override {
      // Linearly interpolate the velocity
      return initialVelocity + currentStep * increment;
    }
};

class StepperMotor {
  private:
    
    // Pins
    uint8_t pulPin, dirPin;
    
    // Position
    long currentPosition, targetPosition;
    long totalSteps, currentStep;
    bool direction;
    
    // Velocity
    unsigned long stepInterval, lastStepTime;
    double currentVelocity;
    
    // Speed profile
    SpeedProfile* speedProfile; // Pointer to current speed profile
    TrapezoidalSpeedProfile trapezoidalProfile;
    LinearSpeedProfile linearProfile;
    
    void initializeMove(long _targetPosition, double _initialVelocity) {
        targetPosition = _targetPosition;
        currentStep = 0;
        
        totalSteps = abs(targetPosition - currentPosition);
        direction = (targetPosition > currentPosition) ? HIGH : LOW;
        digitalWrite(dirPin, direction);
        
        // Set initial delay
        stepInterval = 1e6 / _initialVelocity;
        lastStepTime = micros();
    }

  public:
    // Constructor
    StepperMotor(uint8_t _pulPin, uint8_t _dirPin) : 
        pulPin(_pulPin), dirPin(_dirPin),
        currentPosition(0), targetPosition(0),
        currentVelocity(0),
        totalSteps(0), currentStep(0),
        stepInterval(0), lastStepTime(0), 
        speedProfile(nullptr),
        direction(true) 
    {
        pinMode(pulPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    // Check if the stepper has reached the target position
    bool isAtTarget() {
        return currentPosition == targetPosition;
    }
    
    // Linear speed profile
    void moveToPosition(long _targetPosition, double _initialVelocity, double _finalVelocity) {
      speedProfile = &linearProfile;
      initializeMove(_targetPosition, _initialVelocity);
      speedProfile->compute(totalSteps, _initialVelocity, _finalVelocity);
    }
    
    // Trapezoidal speed profile
    void moveToPosition(long _targetPosition, double _initialVelocity, double _maxVelocity, double _finalVelocity, double _acceleration) {
      speedProfile = &trapezoidalProfile;
      initializeMove(_targetPosition, _initialVelocity);
      speedProfile->compute(totalSteps, _initialVelocity, _finalVelocity, _maxVelocity, _acceleration);
    }

    // Perform a step and print current velocity
    void step() {
        
        unsigned long currentTime = micros();
        unsigned long elapsedTime = currentTime - lastStepTime;

        if (currentPosition != targetPosition && elapsedTime >= stepInterval) {

            currentPosition += (direction == HIGH) ? 1 : -1;
            currentStep ++;
            
            if (speedProfile) {
                currentVelocity = speedProfile->update(currentStep);
                stepInterval = 1e6 / currentVelocity;
            }

            // Update the last step time
            lastStepTime = currentTime;

            digitalWrite(pulPin, HIGH);
            delayMicroseconds(1);
            digitalWrite(pulPin, LOW);

            // Simulate step by printing velocity (this would step the motor in a real setup)
            std::cout << "Step: " << currentStep << ", Position: " << currentPosition
                      << ", Velocity: " << currentVelocity << " steps/sec" << std::endl;
        }
    }

    // Set current position manually
    void setCurrentPosition(long _currentPosition) {
        currentPosition = _currentPosition;
    }

    // Get current position
    long getCurrentPosition() {
        return currentPosition;
    }

    // Get current velocity
    double getCurrentVelocity() {
        return currentVelocity;
    }
};

#endif // STEPPER_MOTOR_HPP
