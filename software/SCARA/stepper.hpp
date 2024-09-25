#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP


class StepperMotor {
  private:

    uint8_t pulPin, dirPin;
    long currentPosition, targetPosition;
    float currentVelocity, initialVelocity, finalVelocity, maxVelocity;
    float acceleration;
    long totalSteps, currentStep, accelSteps, decelSteps, constSteps;
    unsigned long stepInterval, lastStepTime;
    bool direction;

    void computeProfile() {

      // Compute the total steps
      totalSteps = abs(targetPosition - currentPosition);

      // Distance covered during acceleration and deceleration phases
      accelSteps = abs((maxVelocity * maxVelocity - initialVelocity * initialVelocity) / (2 * acceleration));
      decelSteps = abs((maxVelocity * maxVelocity - finalVelocity * finalVelocity) / (2 * acceleration));
        
      // If the distance is too short to reach max velocity, we use a triangular profile
      if (accelSteps + decelSteps > totalSteps) {
          
          // Maximum achievable velocity
          maxVelocity = sqrt(acceleration * totalSteps + 0.5 * (initialVelocity * initialVelocity + finalVelocity * finalVelocity));
        
          // Recompute acceleration and deceleration steps based on maximum achievable velocity
          accelSteps = abs((maxVelocity * maxVelocity - initialVelocity * initialVelocity) / (2 * acceleration));
          decelSteps = totalSteps - accelSteps;

          constSteps = 0;  // No constant velocity phase in triangular profile
      } else {
          // Trapezoidal profile
          constSteps = totalSteps - (accelSteps + decelSteps);
      }
      
      direction = (targetPosition > currentPosition) ? HIGH : LOW;
      digitalWrite(dirPin, direction);
      
      // Initial delay
      stepInterval = 1e6 / initialVelocity;

      // Initialize timing
      lastStepTime = micros();
    }

  public:
  
    StepperMotor(uint8_t _pulPin, uint8_t _dirPin):
      pulPin(_pulPin), dirPin(_dirPin),
      currentPosition(0), targetPosition(0),
      currentVelocity(0), initialVelocity(0), finalVelocity(0), maxVelocity(0),
      acceleration(0),
      totalSteps(0), currentStep(0), accelSteps(0), decelSteps(0), constSteps(0),
      stepInterval(0), lastStepTime(0),
      direction(HIGH)
    {
      pinMode(pulPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
    }

    void moveToPosition(long _targetPosition, float _initialVelocity, float _maxVelocity, float _finalVelocity, float _acceleration) {

      targetPosition = _targetPosition;
      initialVelocity = _initialVelocity;
      finalVelocity = _finalVelocity;
      maxVelocity = _maxVelocity;
      acceleration = _acceleration;
      currentStep = 0;

      computeProfile();
    }

    bool isAtTarget() {
      return currentPosition == targetPosition;
    }

    void setCurrentPosition(long _currentPosition) {
      currentPosition = _currentPosition;
    }

    long getCurrentPosition() {
      return currentPosition;
    }

    void step() {
        
      unsigned long currentTime = micros();
      unsigned long elapsedTime = currentTime - lastStepTime;
      float elapsedTimeS = elapsedTime / 1e6;
      
      if (currentPosition != targetPosition && elapsedTime >= stepInterval) {
          
        // Update position based on direction
        if (direction == HIGH) {
          currentPosition++;
        } else {
          currentPosition--;
        }

        // Acceleration phase
        if (currentStep < accelSteps) {  
            currentVelocity = sqrt(initialVelocity * initialVelocity + 2 * acceleration * currentStep);
            currentVelocity = min(currentVelocity, maxVelocity);  // Cap velocity to maxVelocity
        }
        // Constant velocity phase (only in trapezoidal profile)
        else if (constSteps > 0 && currentStep < totalSteps - decelSteps) {
            currentVelocity = maxVelocity;
        }
        // Deceleration phase
        else if (currentStep < totalSteps) {
            currentVelocity = sqrt(finalVelocity * finalVelocity + 2 * acceleration * (totalSteps - currentStep));
            currentVelocity = max(currentVelocity, finalVelocity);  // Ensure it doesn't go below final velocity
        }
    
        currentStep ++;

        // Update step interval based on current velocity
        stepInterval = 1e6 / currentVelocity;

        // Update the last step time
        lastStepTime = currentTime;
        
        // Step the motor
        digitalWrite(pulPin, HIGH);
        delayMicroseconds(1); // Short pulse, most steppers are good with this, else try 2 or 5 us
        digitalWrite(pulPin, LOW);
      }
    }
};

#endif // STEPPER_MOTOR_HPP
