// ============================================================
// OpenRB-150 - Screw (Dynamixel) Controller
// Slave to Arduino Mega via hardware sync/done pins.
//
// Pin connections to Arduino Mega:
//   SYNC_PIN (pin 3) <- Mega pin 53 (syncPin)
//   DONE_PIN (pin 5) -> Mega pin 52 (donePin)
//
// Signaling protocol:
//   Mega pulses syncPin to signal a command:
//     HIGH pulse (LOW->HIGH->LOW)  -> Extend:   rotate 360deg CW
//     LOW  pulse (HIGH->LOW->HIGH) -> Contract: rotate 360deg CCW
//
//   Each command moves relative to current position, so the same
//   command can be repeated any number of times in a row.
//
//   After move completes, OpenRB pulses DONE_PIN HIGH briefly
//   to notify the Mega.
// ============================================================

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial

const int DXL_DIR_PIN = -1;  // Not used on OpenRB-150
const int SYNC_PIN    = 3;   // Input: signal from Arduino Mega
const int DONE_PIN    = 5;   // Output: pulse HIGH when move complete

const float TOLERANCE        = 2.0;     // Degrees within which target is reached
const float MY_CURRENT_LIMIT = 1300.0;  // mA — stall protection
const float MOVE_DEGREES     = 360.0;   // Degrees to move per command

const uint8_t DXL_ID               = 1;
const float   DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;


// ---- Setup ----

void setup() {
  pinMode(SYNC_PIN, INPUT);
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW);

  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);

  // Use extended position mode so the motor can rotate past 360deg
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 800);

  DEBUG_SERIAL.println("OpenRB ready — waiting for sync pulse...");
}


// ---- Move Dynamixel relative to current position ----
// degrees > 0: CW (extend)
// degrees < 0: CCW (contract)

void moveRelative(float degrees) {
  dxl.torqueOn(DXL_ID);

  // Read current position and compute target
  float currentPos = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  float targetPos  = currentPos + degrees;

  DEBUG_SERIAL.print("Current: ");
  DEBUG_SERIAL.print(currentPos);
  DEBUG_SERIAL.print("deg -> Target: ");
  DEBUG_SERIAL.print(targetPos);
  DEBUG_SERIAL.println("deg");

  dxl.setGoalPosition(DXL_ID, targetPos, UNIT_DEGREE);

  while (true) {
    float pos     = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    float current = dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE);

    if (abs(pos - targetPos) <= TOLERANCE) {
      DEBUG_SERIAL.println("Reached target");
      break;
    }
    if (current > MY_CURRENT_LIMIT) {
      DEBUG_SERIAL.println("Current limit exceeded — stopping");
      break;
    }
    delay(50);
  }

  dxl.torqueOff(DXL_ID);

  // Pulse DONE_PIN to notify Mega
  digitalWrite(DONE_PIN, HIGH);
  delay(100);
  digitalWrite(DONE_PIN, LOW);
  DEBUG_SERIAL.println("Done signal sent to Mega.");
}


// ---- Main loop ----
//
// Pulse detection:
//   HIGH pulse (LOW->HIGH->LOW)  -> extend  (CW  +360deg)
//   LOW  pulse (HIGH->LOW->HIGH) -> contract (CCW -360deg)

void loop() {
  int syncState = digitalRead(SYNC_PIN);
  static int lastState = LOW;

  // HIGH pulse detected -> Extend CW
  if (syncState == HIGH) {
    DEBUG_SERIAL.println("HIGH pulse -> Extending +360deg CW");
    // Wait for pin to return LOW (end of pulse)
    while (digitalRead(SYNC_PIN) == HIGH);
    moveRelative(MOVE_DEGREES);
  }

  // HIGH->LOW transition detected -> Contract CCW
  else if (lastState == HIGH && syncState == LOW) {
    DEBUG_SERIAL.println("LOW pulse -> Contracting -360deg CCW");
    // Wait for pin to return HIGH (end of pulse)
    while (digitalRead(SYNC_PIN) == LOW);
    moveRelative(-MOVE_DEGREES);
  }

  lastState = syncState;
  delay(20);
}