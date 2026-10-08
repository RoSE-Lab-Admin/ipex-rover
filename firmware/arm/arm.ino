#include <Encoder.h>
#include <AccelStepper.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// =============================================================================
// IPEX ARM TEENSY (front and rear arm run the same firmware)
//
// Based on sketch_sep22_da_integration (see legacy/). Movement sequence is
// the original one:
//   release brake (200 ms) -> step both motors at 800 us half-period
//   -> settle (50 ms) -> engage brake (200 ms)
// Changes: the step loops check the serial line before every pulse, so a
// STOP or a new ARM target interrupts the move, and a 50 ms settle was added
// before the brake engages.
//
// BRAKE INTERLOCK (safety-critical):
//   - armStepPulse() refuses to step unless the brake was released at least
//     BRAKE_RELEASE_MS ago.
//   - Step loops exit only between pulses, so step pins are LOW on exit.
//   - engageBrake() refuses while any step loop is running (inStepLoop), and
//     waits SETTLE_MS after the last step before engaging.
//   - An interrupted move ends exactly like a finished one: exit loop ->
//     settle -> engage. A new ARM target then starts a fresh move:
//     release -> step -> settle -> engage.
//
// USB SERIAL PROTOCOL (one command per line, case-insensitive)
//
// Raspberry Pi -> Teensy:
//   CAL               Home arm to limit switch (rejected while moving).
//   ARM <deg>         Absolute arm target, clamped to [ARM_MIN_DEG, ARM_MAX_DEG].
//                     Mid-move: interrupts the current move (stop, brake),
//                     then starts the new move (release, step, brake).
//   DRUM <steps_s>    Drum speed, +/-DRUM_MAX_STEPS_S. Continuous (no timeout).
//                     Applied immediately, does not interrupt the arm.
//   STOP              Interrupt arm (stop, then brake) and ramp drums to 0.
//
// Teensy -> Raspberry Pi:
//   CAL_START / CAL_DONE / CAL_ABORTED   Homing status (ABORTED = not homed)
//   ARM_DONE <deg>                       Move complete, brake engaged
//   STOPPED                              Stop complete, brake engaged
//   STATE <homed> <arm_deg> <IDLE|MOVING|HOMING> <drum_steps_s>   10 Hz
//   ERR NOT_HOMED | ERR BUSY | ERR UNKNOWN_COMMAND
//   # <text>                             Info; Pi ignores
// =============================================================================

#define LED_PIN 13

// --- ARM MOTOR & SENSOR PINS ---
const int MOTOR_ENC_STEP = 8;   // Encoder Motor PUL+
const int MOTOR_ENC_DIR  = 9;   // Encoder Motor DIR+
const int MOTOR_BRK_STEP = 2;   // Brake Motor PUL+
const int MOTOR_BRK_DIR  = 3;   // Brake Motor DIR+

const int ENCODER_A      = 6;
const int ENCODER_B      = 7;
const int LIMIT_SW_PIN   = 10;
const int BRAKE_RELAY    = 23;  // Transistor Base for 24V Brake Relay

// --- DRUM / ACTUATOR PINS (NEMA 17 on TB6600) ---
#define N17_STEP  24  // PUL+
#define N17_DIR   25  // DIR+
#define N17_EN    26  // EN+

// --- HARDWARE & GEARING CONSTANTS ---
const float GEAR_RATIO          = 100.0;
const float MOTOR_STEPS_PER_REV = 400.0;
const float ENCODER_MOTOR_CPR   = 4000.0;

const float STEPS_PER_ARM_DEG   = (MOTOR_STEPS_PER_REV * GEAR_RATIO) / 360.0;
const float ENCODER_CPR_ARM_DEG = (ENCODER_MOTOR_CPR * GEAR_RATIO) / 360.0;

// --- ARM LIMITS ---
const float ARM_MIN_DEG = -40.0;
const float ARM_MAX_DEG =  45.0;

// Arm angle assigned after homing (limit switch hit + backoff).
// If the homing switch is moved, change ONLY this value.
const float HOME_ANGLE_DEG = 45.0;
const int   HOMING_BACKOFF_STEPS = 200;

// --- BRAKE TIMING ---
const unsigned long BRAKE_RELEASE_MS = 200;  // Relay on -> brake fully released, safe to step
const unsigned long SETTLE_MS        = 50;   // Last step -> motors settled, safe to brake
const unsigned long BRAKE_ENGAGE_MS  = 200;  // Relay off -> brake fully holding

const long DRUM_MAX_STEPS_S = 3200;

const unsigned long TELEMETRY_INTERVAL_MS = 100;

// --- OBJECTS & GLOBAL STATES ---
Encoder myEncoder(ENCODER_A, ENCODER_B);
AccelStepper drum1(AccelStepper::DRIVER, N17_STEP, N17_DIR);

bool isHomed = false;
bool isDrumMoving = false;
long drumCommand = 0;

// Arm activity, for telemetry and command gating.
enum ArmActivity { ACT_IDLE, ACT_MOVING, ACT_HOMING };
ArmActivity armActivity = ACT_IDLE;

// Commands that run the arm are queued here and executed by loop().
// STOP and ARM also act as interrupts for an in-progress move.
enum PendingCmd { CMD_NONE, CMD_ARM, CMD_CAL, CMD_STOP };
PendingCmd pendingCmd = CMD_NONE;
float pendingArmDeg = 0.0;

const size_t RX_BUFFER_SIZE = 64;
char rxBuffer[RX_BUFFER_SIZE];
size_t rxIndex = 0;

unsigned long lastTelemetryMs = 0;

void setDrumSpeed(long target_speed);

// --- BRAKE INTERLOCK STATE ---
bool brakeReleased = false;          // Relay HIGH
unsigned long brakeReleasedAtMs = 0; // When the relay went HIGH
bool inStepLoop = false;             // True while any arm step loop is running

// --- BRAKE CONTROL FUNCTIONS ---
// Same relay action and 200 ms delays as the original sketch, plus:
//   - releaseBrake() records when the brake was released.
//   - engageBrake() refuses while a step loop is running, and waits
//     SETTLE_MS after the last step before engaging.
void releaseBrake() {
  digitalWrite(BRAKE_RELAY, HIGH); // Transistor ON -> IN1 Grounded -> Releases 24V Brake
  brakeReleased = true;
  brakeReleasedAtMs = millis();
  delay(BRAKE_RELEASE_MS);
}

bool engageBrake() {
  if (inStepLoop) {
    Serial.println("# REFUSED brake engage: arm step loop active");
    return false;
  }
  delay(SETTLE_MS);                // Motors stopped and settled before locking
  digitalWrite(BRAKE_RELAY, LOW);  // Transistor OFF -> Spring-locks Brake
  brakeReleased = false;
  delay(BRAKE_ENGAGE_MS);
  return true;
}

// --- CALCULATION HELPERS ---
float getPhysicalArmDegrees() {
  long rawCounts = myEncoder.read();
  return rawCounts / ENCODER_CPR_ARM_DEG;
}

// --- ONE STEP PULSE ON BOTH ARM MOTORS (unchanged timing) ---
// Ends with step pins LOW. Refuses (returns false) unless the brake has been
// released for at least BRAKE_RELEASE_MS.
bool armStepPulse() {
  if (!brakeReleased || millis() - brakeReleasedAtMs < BRAKE_RELEASE_MS) {
    Serial.println("ERR BRAKE_NOT_RELEASED");
    return false;
  }

  digitalWrite(MOTOR_ENC_STEP, HIGH);
  digitalWrite(MOTOR_BRK_STEP, HIGH);
  delayMicroseconds(800);
  digitalWrite(MOTOR_ENC_STEP, LOW);
  digitalWrite(MOTOR_BRK_STEP, LOW);
  delayMicroseconds(800);

  // Keep drum motor moving if active
  if (isDrumMoving) drum1.run();
  return true;
}

// =============================================================================
// SERIAL (non-blocking line reader, safe to call inside step loops)
// =============================================================================

const char *activityName() {
  switch (armActivity) {
    case ACT_MOVING: return "MOVING";
    case ACT_HOMING: return "HOMING";
    default:         return "IDLE";
  }
}

void sendTelemetryIfDue() {
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = millis();

  Serial.print("STATE ");
  Serial.print(isHomed ? 1 : 0);
  Serial.print(' ');
  Serial.print(getPhysicalArmDegrees(), 2);
  Serial.print(' ');
  Serial.print(activityName());
  Serial.print(' ');
  Serial.println(drumCommand);
}

void handleLine(char *line) {
  for (char *p = line; *p; ++p) {
    *p = toupper(*p);
  }

  if (strcmp(line, "STOP") == 0) {
    setDrumSpeed(0);
    pendingCmd = CMD_STOP;             // Overrides any queued command
  }
  else if (strncmp(line, "DRUM ", 5) == 0) {
    setDrumSpeed(atol(line + 5));      // Immediate; does not touch the arm
  }
  else if (strncmp(line, "ARM ", 4) == 0) {
    if (armActivity == ACT_HOMING) {
      Serial.println("ERR BUSY");
    } else if (pendingCmd != CMD_STOP) {
      pendingCmd = CMD_ARM;
      pendingArmDeg = atof(line + 4);
    }
  }
  else if (strcmp(line, "CAL") == 0 || strcmp(line, "CALIBRATE") == 0) {
    if (armActivity != ACT_IDLE || pendingCmd != CMD_NONE) {
      Serial.println("ERR BUSY");
    } else {
      pendingCmd = CMD_CAL;
    }
  }
  else {
    Serial.println("ERR UNKNOWN_COMMAND");
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';
      if (rxIndex > 0) handleLine(rxBuffer);
      rxIndex = 0;
      continue;
    }

    if (rxIndex < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxIndex++] = c;
    } else {
      rxIndex = 0;  // Oversized line: discard
    }
  }
}

// Called at the top of every step-loop iteration (step pins are LOW here).
bool moveInterrupted() {
  pollSerial();
  sendTelemetryIfDue();
  return pendingCmd == CMD_STOP || pendingCmd == CMD_ARM;
}

bool homingInterrupted() {
  pollSerial();
  sendTelemetryIfDue();
  return pendingCmd == CMD_STOP;
}

// =============================================================================
// ARM MOVEMENT FUNCTIONS
// =============================================================================

void homeSystem() {
  Serial.println("CAL_START");
  armActivity = ACT_HOMING;
  isHomed = false;
  bool aborted = false;

  releaseBrake();

  // Set opposing directions for initial sweep (Encoder Motor CW / Brake Motor CCW)
  digitalWrite(MOTOR_ENC_DIR, HIGH);
  digitalWrite(MOTOR_BRK_DIR, LOW);

  inStepLoop = true;

  while (digitalRead(LIMIT_SW_PIN) == HIGH) {
    if (homingInterrupted() || !armStepPulse()) { aborted = true; break; }
  }

  if (!aborted) {
    // Back off limit switch in opposite directions
    digitalWrite(MOTOR_ENC_DIR, LOW);
    digitalWrite(MOTOR_BRK_DIR, HIGH);
    for (int i = 0; i < HOMING_BACKOFF_STEPS; i++) {
      if (homingInterrupted() || !armStepPulse()) { aborted = true; break; }
    }
  }

  // Step loop exited: step pins are LOW. Settle, then lock.
  inStepLoop = false;
  engageBrake();
  armActivity = ACT_IDLE;

  if (aborted) {
    Serial.println("CAL_ABORTED");
    return;  // pendingCmd == CMD_STOP; loop() reports STOPPED
  }

  myEncoder.write((long)(HOME_ANGLE_DEG * ENCODER_CPR_ARM_DEG));
  isHomed = true;
  Serial.println("CAL_DONE");
}

void moveToPhysicalAngle(float targetArmDeg) {
  if (!isHomed) {
    Serial.println("ERR NOT_HOMED");
    return;
  }

  if (targetArmDeg > ARM_MAX_DEG) targetArmDeg = ARM_MAX_DEG;
  if (targetArmDeg < ARM_MIN_DEG) targetArmDeg = ARM_MIN_DEG;

  float currentDeg = getPhysicalArmDegrees();
  float deltaDeg = targetArmDeg - currentDeg;

  int stepsToMove = abs(deltaDeg) * STEPS_PER_ARM_DEG;

  if (stepsToMove == 0) {
    Serial.print("ARM_DONE ");
    Serial.println(currentDeg, 2);
    return;
  }

  armActivity = ACT_MOVING;
  releaseBrake();

  // Set inverted relative directions between the two arm motors
  if (deltaDeg < 0) {
    digitalWrite(MOTOR_ENC_DIR, LOW);  // Encoder Motor Downward
    digitalWrite(MOTOR_BRK_DIR, HIGH); // Brake Motor Opposite
  } else {
    digitalWrite(MOTOR_ENC_DIR, HIGH); // Encoder Motor Upward
    digitalWrite(MOTOR_BRK_DIR, LOW);  // Brake Motor Opposite
  }

  bool interrupted = false;
  inStepLoop = true;
  for (int i = 0; i < stepsToMove; i++) {
    if (moveInterrupted() || !armStepPulse()) { interrupted = true; break; }
  }

  // Step loop exited: step pins are LOW. Settle, then lock.
  inStepLoop = false;
  engageBrake();
  armActivity = ACT_IDLE;

  // STOP is reported by loop(); a queued ARM runs next from loop().
  if (!interrupted) {
    Serial.print("ARM_DONE ");
    Serial.println(getPhysicalArmDegrees(), 2);
  }
}

// --- DRUM CONTROL FUNCTIONS ---
void setDrumSpeed(long target_speed) {
  if (target_speed > DRUM_MAX_STEPS_S) target_speed = DRUM_MAX_STEPS_S;
  if (target_speed < -DRUM_MAX_STEPS_S) target_speed = -DRUM_MAX_STEPS_S;
  drumCommand = target_speed;

  if (target_speed != 0) {
    drum1.enableOutputs();
    drum1.setMaxSpeed(abs(target_speed));
    drum1.move((target_speed > 0) ? 2000000000 : -2000000000);
    isDrumMoving = true;
  } else {
    drum1.stop(); // Ramps down smoothly to 0 based on setAcceleration
  }
}

// --- SETUP & MAIN LOOP ---
void setup() {
  Serial.begin(115200);

  // Arm Pin Modes
  pinMode(MOTOR_ENC_STEP, OUTPUT);
  pinMode(MOTOR_ENC_DIR, OUTPUT);
  pinMode(MOTOR_BRK_STEP, OUTPUT);
  pinMode(MOTOR_BRK_DIR, OUTPUT);
  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  pinMode(BRAKE_RELAY, OUTPUT);

  // Drum Pin Modes & AccelStepper Configuration
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  drum1.setEnablePin(N17_EN);
  drum1.setPinsInverted(false, false, true); // (dir, step, enable)
  drum1.setAcceleration(1500.0); // Smooth ramp rate (lower value = gentler start)
  drum1.disableOutputs();

  engageBrake(); // Lock arm brake on boot

  Serial.println("# IPEX arm firmware ready (not homed). Send CAL.");
}

void loop() {
  pollSerial();

  // Take the queued command (if any) and run it.
  const PendingCmd cmd = pendingCmd;
  const float deg = pendingArmDeg;
  pendingCmd = CMD_NONE;

  switch (cmd) {
    case CMD_STOP:
      // Arm is idle here with brake engaged (any move already exited).
      Serial.println("STOPPED");
      break;
    case CMD_ARM:
      moveToPhysicalAngle(deg);
      break;
    case CMD_CAL:
      homeSystem();
      break;
    default:
      break;
  }

  // Non-blocking step execution for drum
  if (isDrumMoving) {
    drum1.run();
    if (drum1.distanceToGo() == 0) {
      drum1.disableOutputs();
      isDrumMoving = false;
    }
  }

  sendTelemetryIfDue();
}
