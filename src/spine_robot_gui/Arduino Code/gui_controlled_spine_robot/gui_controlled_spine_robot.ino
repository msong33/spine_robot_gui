// ============================================================
// Arduino Mega - Main Controller
// Controls: Clamp motors (A & B via TB6612)
// Coordinates with OpenRB-150 via syncPin/donePin for screw
// Communicates with Python GUI via Serial (115200 baud)
//
// Serial command protocol from GUI:
//   1\n  -> Tighten top clamp  (motor 1 CW  270deg)
//   2\n  -> Release top clamp  (motor 1 CCW 270deg)
//   3\n  -> Tighten bottom clamp (motor 2 CW  270deg)
//   4\n  -> Release bottom clamp (motor 2 CCW 270deg)
//   5\n  -> Contract screw (signal OpenRB: pulse LOW)
//   6\n  -> Extend screw   (signal OpenRB: pulse HIGH)
//   7\n  -> Release all clamps (turn1 + turn2)
//   8,<cycles>,<speed>\n -> Forward needle drive
//   9,<cycles>,<speed>\n -> Backward needle drive
//   S    -> Stop current task
//
// Serial responses to GUI:
//   "true"       -> task started/running
//   "false"      -> task finished or stopped
//   "CYCLE:X,Y"  -> cycle progress update
//
// OpenRB signaling:
//   Rather than holding syncPin HIGH or LOW, we pulse it:
//   - Extend:   syncPin LOW -> HIGH -> wait for done -> LOW
//   - Contract: syncPin HIGH -> LOW -> wait for done -> HIGH
//   This guarantees the OpenRB always sees a state change,
//   even if the same command is sent twice in a row.
// ============================================================

#define STBY1 13
#define AIN1  12
#define AIN2  11
#define BIN1  10
#define BIN2   9
#define STBY2  8

#define encoder1A  3
#define encoder1B  2
#define encoder2A 18
#define encoder2B 19

const int syncPin = 53;  // Output to OpenRB SYNC_PIN
const int donePin = 52;  // Input from OpenRB DONE_PIN

volatile int encoder1Pos = 0;
volatile int encoder2Pos = 0;
const int encoderCountsOneRev = 596;
const int counts270 = (int)(encoderCountsOneRev * 270.0 / 360.0);  // = 447 counts

char GUIInput[10];
int parsedInputValues[3];
int pos = 0;

bool stopRequested   = false;
bool functionRunning = false;


// ---- Encoder ISRs ----

void doEncoder1() {
  encoder1Pos += (digitalRead(encoder1B) == digitalRead(encoder1A)) ? 1 : -1;
}

void doEncoder2() {
  encoder2Pos += (digitalRead(encoder2B) == digitalRead(encoder2A)) ? 1 : -1;
}


// ---- Setup ----

void setup() {
  Serial.begin(115200);

  pinMode(STBY1, OUTPUT);
  pinMode(AIN1,  OUTPUT);
  pinMode(AIN2,  OUTPUT);
  pinMode(BIN1,  OUTPUT);
  pinMode(BIN2,  OUTPUT);
  pinMode(STBY2, OUTPUT);

  pinMode(syncPin, OUTPUT);
  pinMode(donePin, INPUT);

  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1A), doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), doEncoder2, CHANGE);

  digitalWrite(STBY1, HIGH);
  digitalWrite(STBY2, HIGH);
  digitalWrite(syncPin, LOW);

  Serial.println("false");  // Tell GUI we are ready
}


// ---- Motor control ----

void move(int motor, int speed, int cw) {
  digitalWrite(STBY1, HIGH);
  if (motor == 1) {
    if (cw == 1) { digitalWrite(AIN1, LOW);  analogWrite(AIN2, 255 - speed); }
    else         { analogWrite(AIN1, speed); digitalWrite(AIN2, HIGH); }
  } else if (motor == 2) {
    if (cw == 1) { digitalWrite(BIN1, LOW);  analogWrite(BIN2, 255 - speed); }
    else         { analogWrite(BIN1, speed); digitalWrite(BIN2, HIGH); }
  }
}

void stopMotor(int motor) {
  if (motor == 1) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
  else if (motor == 2) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
}


// ---- Core movement: turn a motor 270 degrees ----
// motor: 1 or 2
// cw:    1 = clockwise, 2 = counter-clockwise

void turn270(int motor, int speed, int cw) {
  int initialPos = (motor == 1) ? encoder1Pos : encoder2Pos;
  move(motor, speed, cw);
  while (true) {
    int currentPos = (motor == 1) ? encoder1Pos : encoder2Pos;
    if (abs(currentPos - initialPos) >= counts270) break;
    checkStopRequested();
    if (stopRequested) break;
    move(motor, speed, cw);
  }
  stopMotor(motor);
}


// ---- Named clamp/turn wrappers ----

void clamp1(int speed) {
  // Tighten top clamp: motor 1 CW 270deg
  Serial.println("true");
  turn270(1, speed, 1);
  if (!functionRunning) { stopRequested = false; Serial.println("false"); }
}

void turn1(int speed) {
  // Release top clamp: motor 1 CCW 270deg
  Serial.println("true");
  turn270(1, speed, 2);
  if (!functionRunning) { stopRequested = false; Serial.println("false"); }
}

void clamp2(int speed) {
  // Tighten bottom clamp: motor 2 CW 270deg
  Serial.println("true");
  turn270(2, speed, 1);
  if (!functionRunning) { stopRequested = false; Serial.println("false"); }
}

void turn2(int speed) {
  // Release bottom clamp: motor 2 CCW 270deg
  Serial.println("true");
  turn270(2, speed, 2);
  if (!functionRunning) { stopRequested = false; Serial.println("false"); }
}

// Release all clamps: turn1 then turn2 sequentially
void turn12(int speed) {
  functionRunning = true;
  Serial.println("true");
  checkStopRequested();
  if (!stopRequested) turn1(speed);
  checkStopRequested();
  if (!stopRequested) turn2(speed);
  stopRequested = false;
  Serial.println("false");
  functionRunning = false;
}


// ---- OpenRB coordination ----

// Wait for donePin to pulse HIGH then return LOW.
// Returns true on success, false if stopped or timed out.
bool waitForDone() {
  unsigned long start = millis();
  // Wait for donePin to go HIGH
  while (digitalRead(donePin) == LOW) {
    checkStopRequested();
    if (stopRequested) return false;
    if (millis() - start > 15000) {
      Serial.println("OpenRB timeout");
      return false;
    }
  }
  // Wait for donePin to return LOW (end of pulse)
  while (digitalRead(donePin) == HIGH);
  delay(50);  // Short settle time
  return true;
}

// Signal OpenRB using a pulse so repeated same-direction commands
// always trigger a state change on the OpenRB side.
//
// Extend  (signalLevel = HIGH): pulse LOW->HIGH, OpenRB moves to 360deg
// Contract (signalLevel = LOW):  pulse HIGH->LOW, OpenRB moves to 0deg
//
// After done, syncPin is reset to idle (opposite of active level)
// so the next call always starts from the correct baseline.

void triggerOpenRB(int signalLevel) {
  Serial.println("true");

  // Pulse: go to opposite first, then to desired level
  digitalWrite(syncPin, !signalLevel);
  delay(50);
  digitalWrite(syncPin, signalLevel);

  bool ok = waitForDone();

  // Reset syncPin to idle state (opposite of what we just sent)
  digitalWrite(syncPin, !signalLevel);

  if (!functionRunning) {
    stopRequested = false;
    Serial.println("false");
  }
}


// ---- Composite drive functions ----

// Forward needle drive:
//   extend -> tighten top -> release bottom ->
//   contract -> tighten bottom -> release top
void forwardDrive(int cycles, int speed) {
  functionRunning = true;
  Serial.println("true");

  for (int i = 0; i < cycles; i++) {

    // Extend screw
    checkStopRequested(); if (stopRequested) break;
    triggerOpenRB(HIGH);
    delay(200);

    // Tighten top clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(1, speed, 1);
    delay(200);

    // Release bottom clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(2, speed, 2);
    delay(200);

    // Contract screw
    checkStopRequested(); if (stopRequested) break;
    triggerOpenRB(LOW);
    delay(200);

    // Tighten bottom clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(2, speed, 1);
    delay(200);

    // Release top clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(1, speed, 2);
    delay(200);

    // Report cycle progress to GUI
    Serial.print("CYCLE:");
    Serial.print(i + 1);
    Serial.print(",");
    Serial.println(cycles);
  }

  stopRequested = false;
  Serial.println("false");
  functionRunning = false;
}

// Backward needle drive:
//   contract -> tighten top -> release bottom ->
//   extend -> tighten bottom -> release top
void backwardDrive(int cycles, int speed) {
  functionRunning = true;
  Serial.println("true");

  for (int i = 0; i < cycles; i++) {

    // Contract screw
    checkStopRequested(); if (stopRequested) break;
    triggerOpenRB(LOW);
    delay(200);

    // Tighten top clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(1, speed, 1);
    delay(500);

    // Release bottom clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(2, speed, 2);
    delay(500);

    // Extend screw
    checkStopRequested(); if (stopRequested) break;
    triggerOpenRB(HIGH);
    delay(200);

    // Tighten bottom clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(2, speed, 1);
    delay(500);

    // Release top clamp
    checkStopRequested(); if (stopRequested) break;
    turn270(1, speed, 2);
    delay(500);

    // Report cycle progress to GUI
    Serial.print("CYCLE:");
    Serial.print(i + 1);
    Serial.print(",");
    Serial.println(cycles);
  }

  stopRequested = false;
  Serial.println("false");
  functionRunning = false;
}


// ---- Serial helpers ----

void checkStopRequested() {
  while (Serial.available()) {
    char b = Serial.read();
    if (b == 'S') stopRequested = true;
  }
}


// ---- Main loop ----

void loop() {
  stopMotor(1);
  stopMotor(2);

  while (Serial.available()) {
    char commandByte = Serial.read();

    if (commandByte == '\n') {
      GUIInput[pos] = '\0';

      char* token = strtok(GUIInput, ",");
      int i = 0;
      while (token != NULL && i < 3) {
        parsedInputValues[i++] = atoi(token);
        token = strtok(NULL, ",");
      }

      switch (parsedInputValues[0]) {
        case 1: clamp1(150);                                               break;  // Tighten top clamp
        case 2: turn1(200);                                                break;  // Release top clamp
        case 3: clamp2(150);                                               break;  // Tighten bottom clamp
        case 4: turn2(200);                                                break;  // Release bottom clamp
        case 5: triggerOpenRB(LOW);                                        break;  // Contract screw
        case 6: triggerOpenRB(HIGH);                                       break;  // Extend screw
        case 7: turn12(200);                                               break;  // Release all clamps
        case 8: forwardDrive(parsedInputValues[1], parsedInputValues[2]);  break;  // Forward drive
        case 9: backwardDrive(parsedInputValues[1], parsedInputValues[2]); break;  // Backward drive
      }

      pos = 0;

    } else {
      if (pos < sizeof(GUIInput) - 1) GUIInput[pos++] = commandByte;
      else pos = 0;
    }
  }
}