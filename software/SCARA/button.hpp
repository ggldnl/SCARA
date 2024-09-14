#ifndef BUTTON_HPP
#define BUTTON_HPP

class Button {
public:
    // Constructor: initializes the button pin and sets it up
    Button(uint8_t buttonPin)
        : buttonPin_(buttonPin), lastState_(HIGH), lastDebounceTime_(0), debounceDelay_(50) {
        pinMode(buttonPin_, INPUT_PULLUP);  // Set pin as input with internal pull-up resistor
    }

    // Check if the button is pressed (LOW state)
    bool pressed() {
        int currentState = digitalRead(buttonPin_);
        updateState(currentState);

        // Return true only if the button was pressed (transition from HIGH to LOW)
        return (lastState_ == LOW && stateChanged_ && currentState_ == LOW);
    }

    // Check if the button is released (HIGH state)
    bool released() {
        int currentState = digitalRead(buttonPin_);
        updateState(currentState);

        // Return true only if the button was released (transition from LOW to HIGH)
        return (lastState_ == HIGH && stateChanged_ && currentState_ == HIGH);
    }

    int getPin() {
        return buttonPin_;
    }

private:
    uint8_t buttonPin_;       // Button pin
    int lastState_;           // Last known state of the button
    int currentState_;        // Current state of the button
    bool stateChanged_;       // Has the state changed since last read?
    unsigned long lastDebounceTime_;  // Timestamp of last state change
    const unsigned long debounceDelay_;  // Debounce time in milliseconds

    // Debounce logic to filter out mechanical noise
    void updateState(int currentState) {
        unsigned long currentTime = millis();

        // Check if state has changed and debounce
        if (currentState != lastState_) {
            lastDebounceTime_ = currentTime;
        }

        if ((currentTime - lastDebounceTime_) > debounceDelay_) {
            if (currentState != currentState_) {
                currentState_ = currentState;
                stateChanged_ = true;
            } else {
                stateChanged_ = false;
            }
        }

        lastState_ = currentState;
    }
};

#endif // BUTTON_HPP
