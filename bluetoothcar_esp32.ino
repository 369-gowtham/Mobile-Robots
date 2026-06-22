char data;  // Fixed: added semicolon

// Motor A - Left
#define A_ForwardPin 27
#define A_BackwardPin 26
#define A_EnablePin 25
#define A_EncoderPinA 18
#define A_EncoderPinB 21

// Motor B - Right
#define B_ForwardPin 14
#define B_BackwardPin 12
#define B_EnablePin 13
#define B_EncoderPinA 32
#define B_EncoderPinB 33

// Encoder Values
volatile long A_Encodervalue = 0;
volatile long B_Encodervalue = 0;

void setup() {
  Serial.begin(9600);

  // Motor A setup
  pinMode(A_ForwardPin, OUTPUT);
  pinMode(A_BackwardPin, OUTPUT);
  pinMode(A_EnablePin, OUTPUT);
  pinMode(A_EncoderPinA, INPUT);
  pinMode(A_EncoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(A_EncoderPinA), updateEncoderA, RISING);

  // Motor B setup
  pinMode(B_ForwardPin, OUTPUT);
  pinMode(B_BackwardPin, OUTPUT);
  pinMode(B_EnablePin, OUTPUT);
  pinMode(B_EncoderPinA, INPUT);
  pinMode(B_EncoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(B_EncoderPinA), updateEncoderB, RISING);
}

void loop() {
  if (Serial.available()) {
    data = Serial.read();  // Corrected
    switch (data) {
      case 'F':
        moveForward();
        break;
      case 'B':
        moveBackward();
        break;
      case 'R':
        turnRight();
        break;
      case 'L':
        turnLeft();  // Corrected from turnRight()
        break;
      default:
        stopMotors();
        break;
    }
  }
}

// ---------------- Motor Functions ----------------
void moveForward() {
  digitalWrite(A_ForwardPin, HIGH);
  digitalWrite(A_BackwardPin, LOW);
  analogWrite(A_EnablePin, 90);

  digitalWrite(B_ForwardPin, HIGH);
  digitalWrite(B_BackwardPin, LOW);
  analogWrite(B_EnablePin, 90);
}

void turnRight() {
  digitalWrite(A_ForwardPin, HIGH);    // Left motor moves
  digitalWrite(A_BackwardPin, LOW);
  analogWrite(A_EnablePin, 90);

  digitalWrite(B_ForwardPin, LOW);     // Right motor stops
  digitalWrite(B_BackwardPin, LOW);
  analogWrite(B_EnablePin, 0);
}

void turnLeft() {
  digitalWrite(A_ForwardPin, LOW);     // Left motor stops
  digitalWrite(A_BackwardPin, LOW);
  analogWrite(A_EnablePin, 0);

  digitalWrite(B_ForwardPin, HIGH);    // Right motor moves
  digitalWrite(B_BackwardPin, LOW);
  analogWrite(B_EnablePin, 90);
}

void moveBackward() {
  digitalWrite(A_ForwardPin, LOW);
  digitalWrite(A_BackwardPin, HIGH);
  analogWrite(A_EnablePin, 90);

  digitalWrite(B_ForwardPin, LOW);
  digitalWrite(B_BackwardPin, HIGH);
  analogWrite(B_EnablePin, 90);
}

void stopMotors() {
  digitalWrite(A_ForwardPin, LOW);
  digitalWrite(A_BackwardPin, LOW);
  analogWrite(A_EnablePin, 0);

  digitalWrite(B_ForwardPin, LOW);
  digitalWrite(B_BackwardPin, LOW);
  analogWrite(B_EnablePin, 0);
}

// ---------------- Encoder Interrupts ----------------
void updateEncoderA() {
  if (digitalRead(A_EncoderPinA) > digitalRead(A_EncoderPinB))
    A_Encodervalue++;
  else
    A_Encodervalue--;
}

void updateEncoderB() {
  if (digitalRead(B_EncoderPinA) > digitalRead(B_EncoderPinB))
    B_Encodervalue++;
  else
    B_Encodervalue--;
}
