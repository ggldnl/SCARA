// Define the struct to hold motor pin configurations
struct Motor {
  byte stepPin;
  byte directionPin;
};

// Instantiate three motors with their respective pins
Motor MOTOR_X = {2, 5};
Motor MOTOR_Y = {3, 6};
Motor MOTOR_Z = {12, 13};

// Create a motor array
Motor motors[3] = {
  MOTOR_X,
  MOTOR_Y,
  MOTOR_Z
};
byte currentMotorIndex = 0;

// Define the pins for the buttons
const byte BUTTON_CW = 10;
const byte BUTTON_CCW = 9;
const byte BUTTON_MTR = 11;

// Define the enable pin
const byte ENABLE = 8;

// Debounce variables
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Function to step motor clockwise
void clockwise(int SMSpeed=2000) {
  Motor& motor = motors[currentMotorIndex];
  digitalWrite(motor.directionPin, HIGH);
  digitalWrite(motor.stepPin, HIGH);
  delayMicroseconds(SMSpeed);
  digitalWrite(motor.stepPin, LOW);
  delayMicroseconds(SMSpeed);
  // Serial.println("state -> clockwise");
}

// Function to step motor counterclockwise
void counterclockwise(int SMSpeed=2000) {
  Motor& motor = motors[currentMotorIndex];
  digitalWrite(motor.directionPin, LOW);
  digitalWrite(motor.stepPin, HIGH);
  delayMicroseconds(SMSpeed);
  digitalWrite(motor.stepPin, LOW);
  delayMicroseconds(SMSpeed);
  // Serial.println("state -> counterclockwise");
}

// Function to cycle through motors
void cycleMotor() {
  currentMotorIndex = (currentMotorIndex + 1) % 3;
  // Serial.print("state -> current motor set to ");
  // Serial.println(currentMotorIndex);
}

void setup() {
  
  // Set motor pins as outputs using a for loop
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].directionPin, OUTPUT);
    pinMode(motors[i].stepPin, OUTPUT);
  }

  // Set button pins as inputs with pullup resistors
  pinMode(BUTTON_CW, INPUT_PULLUP);
  pinMode(BUTTON_CCW, INPUT_PULLUP);
  pinMode(BUTTON_MTR, INPUT_PULLUP);

  // Enable the board
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);

  // Initialize serial communication for debugging
  // Serial.begin(9600);

  // Init routine
  for (int i=0; i<100; i++, clockwise());
  delay(100);
  for (int i=0; i<100; i++, counterclockwise());
  delay(100);
}

void loop() {

  // Read the state of the buttons
  byte currentStateCW = digitalRead(BUTTON_CW);
  byte currentStateCCW = digitalRead(BUTTON_CCW);
  byte currentStateMTR = digitalRead(BUTTON_MTR);

  // Handle clockwise button press
  if (currentStateCW == LOW) {
    clockwise();
  }

  // Handle counterclockwise button press
  if (currentStateCCW == LOW) {
    counterclockwise();
  }

  // Handle cycle button press with debounce
  if (currentStateMTR == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    cycleMotor();
    lastDebounceTime = millis();
    delay(200);
  }
}