// ============================================================
// OpenRB-150 - Screw Motor Bench Test
//
// Standalone diagnostic for the screw Dynamixel ONLY. Nothing here
// touches SYNC_PIN or DONE_PIN, so it isolates the motor, its power and
// its wiring from the Arduino Mega, the sync handshake and the ROS GUI.
// If the motor misbehaves here, the problem is not the handshake.
//
// Upload to the OpenRB-150, then open the Serial Monitor at 115200 with
// line ending set to "Newline" and type commands:
//
//   ?         Show this help
//   p         Ping the motor and print its model / firmware
//   n         Scan IDs 1-20 for any motor that answers
//   s         Status: position, current, voltage, temperature, torque
//   e         Extend   (+360 deg, same as the real sketch)
//   c         Contract (-360 deg, same as the real sketch)
//   <number>  Move that many degrees, relative (e.g. 90, -45, 720)
//   v <n>     Set profile velocity (default 800; lower = slower)
//   t         Toggle torque on/off
//   x         Torque off (also aborts a move in progress)
//
// The move constants match openrb_slave.ino so results carry over.
// ============================================================

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial

const int DXL_DIR_PIN = -1;  // Not used on OpenRB-150

// Kept identical to openrb_slave.ino so this test reproduces real behavior
const float TOLERANCE        = 2.0;     // Degrees within which target is reached
const float MY_CURRENT_LIMIT = 1300.0;  // mA — stall protection
const float MOVE_DEGREES     = 360.0;   // Degrees per extend/contract

const unsigned long MOVE_TIMEOUT_MS = 10000;  // Give up on a move after this
const unsigned long SERIAL_WAIT_MS  = 5000;   // Max wait for the Serial Monitor

uint8_t     dxlId               = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
int         profileVelocity      = 800;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

char cmdBuf[24];
int  cmdLen = 0;
bool torqueIsOn = false;


// ---- Help ----

void printHelp() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("Commands:");
  DEBUG_SERIAL.println("  ?         this help");
  DEBUG_SERIAL.println("  p         ping motor");
  DEBUG_SERIAL.println("  n         scan IDs 1-20");
  DEBUG_SERIAL.println("  s         status (pos/current/volts/temp)");
  DEBUG_SERIAL.println("  e         extend   +360 deg");
  DEBUG_SERIAL.println("  c         contract -360 deg");
  DEBUG_SERIAL.println("  <number>  move that many degrees, relative");
  DEBUG_SERIAL.println("  v <n>     set profile velocity (now 800)");
  DEBUG_SERIAL.println("  t         toggle torque");
  DEBUG_SERIAL.println("  x         torque off / abort move");
  DEBUG_SERIAL.println();
}


// ---- Diagnostics ----

void pingMotor() {
  DEBUG_SERIAL.print("Pinging ID ");
  DEBUG_SERIAL.print(dxlId);
  DEBUG_SERIAL.println("...");

  if (dxl.ping(dxlId)) {
    DEBUG_SERIAL.print("  OK - model ");
    DEBUG_SERIAL.print(dxl.getModelNumber(dxlId));
    DEBUG_SERIAL.print(", firmware ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(FIRMWARE_VERSION, dxlId));
  } else {
    DEBUG_SERIAL.println("  NO RESPONSE.");
    DEBUG_SERIAL.println("  Check, in this order:");
    DEBUG_SERIAL.println("   1. 12V supply actually powering the servo bus");
    DEBUG_SERIAL.println("   2. Dynamixel cable seated at both ends");
    DEBUG_SERIAL.println("   3. Motor ID (try 'n' to scan)");
    DEBUG_SERIAL.println("   4. Baud rate - this sketch uses 57600");
  }
}

void scanIds() {
  DEBUG_SERIAL.println("Scanning IDs 1-20 at 57600...");
  int found = 0;
  for (uint8_t id = 1; id <= 20; id++) {
    if (dxl.ping(id)) {
      DEBUG_SERIAL.print("  found ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.print(" (model ");
      DEBUG_SERIAL.print(dxl.getModelNumber(id));
      DEBUG_SERIAL.println(")");
      found++;
    }
  }
  if (found == 0) {
    DEBUG_SERIAL.println("  none found - suspect power, cabling, or baud rate");
  } else {
    DEBUG_SERIAL.print("  ");
    DEBUG_SERIAL.print(found);
    DEBUG_SERIAL.println(" motor(s) responding");
  }
}

void printStatus() {
  if (!dxl.ping(dxlId)) {
    DEBUG_SERIAL.println("No response - run 'p' for troubleshooting");
    return;
  }
  DEBUG_SERIAL.print("pos ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(dxlId, UNIT_DEGREE), 1);
  DEBUG_SERIAL.print(" deg | current ");
  DEBUG_SERIAL.print(dxl.getPresentCurrent(dxlId, UNIT_MILLI_AMPERE), 0);
  DEBUG_SERIAL.print(" mA | ");
  DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, dxlId) / 10.0, 1);
  DEBUG_SERIAL.print(" V | ");
  DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_TEMPERATURE, dxlId));
  DEBUG_SERIAL.print(" C | torque ");
  DEBUG_SERIAL.println(torqueIsOn ? "ON" : "OFF");
}


// ---- Movement ----
// Same control strategy as openrb_slave.ino's moveRelative(), plus a
// timeout, a live progress trace and an 'x' abort so a stall is obvious
// instead of just hanging.

void moveRelative(float degrees) {
  if (!dxl.ping(dxlId)) {
    DEBUG_SERIAL.println("No response - aborting move");
    return;
  }

  dxl.torqueOn(dxlId);
  torqueIsOn = true;

  float startPos  = dxl.getPresentPosition(dxlId, UNIT_DEGREE);
  float targetPos = startPos + degrees;

  DEBUG_SERIAL.print("Move ");
  DEBUG_SERIAL.print(degrees, 1);
  DEBUG_SERIAL.print(" deg: ");
  DEBUG_SERIAL.print(startPos, 1);
  DEBUG_SERIAL.print(" -> ");
  DEBUG_SERIAL.println(targetPos, 1);

  dxl.setGoalPosition(dxlId, targetPos, UNIT_DEGREE);

  unsigned long start    = millis();
  unsigned long lastTrace = 0;
  float peakCurrent      = 0;
  const char* outcome    = "timed out";

  while (true) {
    float pos     = dxl.getPresentPosition(dxlId, UNIT_DEGREE);
    float current = dxl.getPresentCurrent(dxlId, UNIT_MILLI_AMPERE);
    if (current > peakCurrent) peakCurrent = current;

    if (abs(pos - targetPos) <= TOLERANCE) { outcome = "reached target"; break; }

    if (current > MY_CURRENT_LIMIT) {
      outcome = "STALLED (current limit)";
      break;
    }
    if (millis() - start > MOVE_TIMEOUT_MS) break;

    // 'x' aborts without waiting for the move to finish
    if (DEBUG_SERIAL.available() && DEBUG_SERIAL.read() == 'x') {
      outcome = "aborted by user";
      break;
    }

    if (millis() - lastTrace > 250) {
      lastTrace = millis();
      DEBUG_SERIAL.print("  at ");
      DEBUG_SERIAL.print(pos, 1);
      DEBUG_SERIAL.print(" deg, ");
      DEBUG_SERIAL.print(current, 0);
      DEBUG_SERIAL.println(" mA");
    }
    delay(20);
  }

  float endPos = dxl.getPresentPosition(dxlId, UNIT_DEGREE);
  dxl.torqueOff(dxlId);
  torqueIsOn = false;

  DEBUG_SERIAL.print(outcome);
  DEBUG_SERIAL.print(": moved ");
  DEBUG_SERIAL.print(endPos - startPos, 1);
  DEBUG_SERIAL.print(" of ");
  DEBUG_SERIAL.print(degrees, 1);
  DEBUG_SERIAL.print(" deg in ");
  DEBUG_SERIAL.print(millis() - start);
  DEBUG_SERIAL.print(" ms, peak ");
  DEBUG_SERIAL.print(peakCurrent, 0);
  DEBUG_SERIAL.println(" mA");
  DEBUG_SERIAL.println();
}


// ---- Command dispatch ----

void runCommand(char* cmd) {
  // Trim trailing whitespace/CR
  int n = strlen(cmd);
  while (n > 0 && (cmd[n - 1] == '\r' || cmd[n - 1] == ' ')) cmd[--n] = '\0';
  if (n == 0) return;

  switch (cmd[0]) {
    case '?': printHelp();                 return;
    case 'p': pingMotor();                 return;
    case 'n': scanIds();                   return;
    case 's': printStatus();               return;
    case 'e': moveRelative(MOVE_DEGREES);  return;
    case 'c': moveRelative(-MOVE_DEGREES); return;

    case 'v': {
      int v = atoi(cmd + 1);
      if (v <= 0) { DEBUG_SERIAL.println("Usage: v <velocity>, e.g. v 400"); return; }
      profileVelocity = v;
      dxl.writeControlTableItem(PROFILE_VELOCITY, dxlId, profileVelocity);
      DEBUG_SERIAL.print("Profile velocity = ");
      DEBUG_SERIAL.println(profileVelocity);
      return;
    }

    case 't':
      if (torqueIsOn) { dxl.torqueOff(dxlId); torqueIsOn = false; }
      else            { dxl.torqueOn(dxlId);  torqueIsOn = true;  }
      DEBUG_SERIAL.print("Torque ");
      DEBUG_SERIAL.println(torqueIsOn ? "ON" : "OFF");
      return;

    case 'x':
      dxl.torqueOff(dxlId);
      torqueIsOn = false;
      DEBUG_SERIAL.println("Torque OFF");
      return;
  }

  // Anything starting with a digit or sign is a relative move in degrees
  if (cmd[0] == '-' || cmd[0] == '+' || (cmd[0] >= '0' && cmd[0] <= '9')) {
    moveRelative(atof(cmd));
    return;
  }

  DEBUG_SERIAL.print("Unknown command: ");
  DEBUG_SERIAL.println(cmd);
  DEBUG_SERIAL.println("Type ? for help");
}


// ---- Setup / loop ----

void setup() {
  DEBUG_SERIAL.begin(115200);
  // Bounded wait: never block forever on an absent Serial Monitor.
  unsigned long serialWait = millis();
  while (!DEBUG_SERIAL && millis() - serialWait < SERIAL_WAIT_MS);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("=== OpenRB screw motor bench test ===");

  bool alive = dxl.ping(dxlId);
  if (alive) {
    dxl.torqueOff(dxlId);
    dxl.setOperatingMode(dxlId, OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, dxlId, profileVelocity);
    DEBUG_SERIAL.print("Motor ID ");
    DEBUG_SERIAL.print(dxlId);
    DEBUG_SERIAL.println(" ready, extended-position mode, torque OFF.");
    printStatus();
  } else {
    DEBUG_SERIAL.println("Motor did NOT answer at startup.");
    pingMotor();
  }

  printHelp();
}

void loop() {
  while (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      runCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdLen = 0;  // Overlong input, discard
    }
  }
}
