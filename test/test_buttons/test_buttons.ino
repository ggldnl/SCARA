const int buttonPin1 = 9;  // Pin for button 1
const int buttonPin2 = 10; // Pin for button 2
const int buttonPin3 = 11; // Pin for button 3

unsigned long lastDebounceTime1 = 0; // the last time the output pin was toggled
unsigned long lastDebounceTime2 = 0; // the last time the output pin was toggled
unsigned long lastDebounceTime3 = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int buttonState1 = HIGH;    // the current reading from the input pin
int lastButtonState1 = HIGH; // the previous reading from the input pin

int buttonState2 = HIGH;    // the current reading from the input pin
int lastButtonState2 = HIGH; // the previous reading from the input pin

int buttonState3 = HIGH;    // the current reading from the input pin
int lastButtonState3 = HIGH; // the previous reading from the input pin

void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  // Initialize the pushbutton pins as input with internal pull-up resistors:
  Serial.println("[INFO] Setting up button pins");
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
}

void loop() {
  // Read the state of the pushbuttons:
  int reading1 = digitalRead(buttonPin1);
  int reading2 = digitalRead(buttonPin2);
  int reading3 = digitalRead(buttonPin3);

  // Debounce for button 1
  if (reading1 != lastButtonState1) {
    lastDebounceTime1 = millis();
  }
  if ((millis() - lastDebounceTime1) > debounceDelay) {
    if (reading1 != buttonState1) {
      buttonState1 = reading1;
      if (buttonState1 == LOW) {
        Serial.print("[INFO] Button on pin ");
        Serial.print(buttonPin1);
        Serial.println(" pressed");
      }
    }
  }
  lastButtonState1 = reading1;

  // Debounce for button 2
  if (reading2 != lastButtonState2) {
    lastDebounceTime2 = millis();
  }
  if ((millis() - lastDebounceTime2) > debounceDelay) {
    if (reading2 != buttonState2) {
      buttonState2 = reading2;
      if (buttonState2 == LOW) {
        Serial.print("[INFO] Button on pin ");
        Serial.print(buttonPin2);
        Serial.println(" pressed");
      }
    }
  }
  lastButtonState2 = reading2;

  // Debounce for button 3
  if (reading3 != lastButtonState3) {
    lastDebounceTime3 = millis();
  }
  if ((millis() - lastDebounceTime3) > debounceDelay) {
    if (reading3 != buttonState3) {
      buttonState3 = reading3;
      if (buttonState3 == LOW) {
        Serial.print("[INFO] Button on pin ");
        Serial.print(buttonPin3);
        Serial.println(" pressed");
      }
    }
  }
  lastButtonState3 = reading3;
}
