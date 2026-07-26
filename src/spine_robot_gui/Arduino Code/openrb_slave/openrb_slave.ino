// ============================================================
// OpenRB-150 - Screw (Dynamixel) Controller
// Slave to Arduino Mega via hardware sync/done pins.
//
// Pin connections to Arduino Mega:
//   SYNC_PIN (pin 3) <- Mega pin 53 (syncPin)
//   DONE_PIN (pin 5) -> Mega pin 52 (donePin)
//
// Signaling protocol:
//   SYNC_PIN idles LOW. The Mega sends a HIGH pulse to command one move,
//   and the width of that pulse selects the direction:
//     short pulse (~100ms) -> Extend:   rotate 360deg CW
//     long  pulse (~500ms) -> Contract: rotate 360deg CCW
//
//   The move begins on the falling edge, once the width is known.
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

// Sync pulse classification. The Mega sends 100ms to extend and 500ms to
// contract, so the split sits halfway between with wide margin either side.
const unsigned long PULSE_SPLIT_MS = 300;   // Below = extend, at/above = contract
const unsigned long PULSE_MAX_MS   = 3000;  // Longer than this = stuck line, ignore

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
// SYNC_PIN idles LOW. A HIGH pulse commands one move and the width of
// that pulse selects the direction:
//   short pulse (< PULSE_SPLIT_MS) -> extend   (CW  +360deg)
//   long  pulse (>= PULSE_SPLIT_MS) -> contract (CCW -360deg)
//
// The move starts on the falling edge, once the full width is known.
// Measuring width instead of reading a resting level means the line is
// never ambiguous: a pin sitting at idle can't be mistaken for a command.

void loop() {
  if (digitalRead(SYNC_PIN) == LOW) {
    delay(5);
    return;
  }

  // Rising edge — time how long the line stays HIGH.
  // Nothing is printed inside this loop to keep the measurement tight.
  unsigned long start = millis();
  while (digitalRead(SYNC_PIN) == HIGH) {
    if (millis() - start > PULSE_MAX_MS) {
      DEBUG_SERIAL.println("Sync line stuck HIGH - ignoring");
      while (digitalRead(SYNC_PIN) == HIGH);  // Wait it out, no move
      return;
    }
  }
  unsigned long width = millis() - start;

  if (width < PULSE_SPLIT_MS) {
    DEBUG_SERIAL.print(width);
    DEBUG_SERIAL.println("ms pulse -> Extending +360deg CW");
    moveRelative(MOVE_DEGREES);
  } else {
    DEBUG_SERIAL.print(width);
    DEBUG_SERIAL.println("ms pulse -> Contracting -360deg CCW");
    moveRelative(-MOVE_DEGREES);
  }
}
