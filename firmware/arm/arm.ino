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
// Changes: step loops check the serial line before every pulse so STOP / a new
// target / a jog interrupts the move; a 50 ms settle before the brake engages;
// drums keep stepping during every arm wait; hold-to-jog with timeout.
//
// BRAKE INTERLOCK (safety-critical):
//   - armStepPulse() refuses to step unless the brake was released at least
//     BRAKE_RELEASE_MS ago.
//   - Step loops exit only between pulses, so step pins are LOW on exit.
//   - engageBrake() refuses while any step loop is running (inStepLoop), and
//     waits SETTLE_MS after the last step before engaging.
//   - Every move, jog, and homing run ends the same way: exit loop -> settle
//     -> engage. A new move/jog then starts fresh: release -> step -> ...
//
// JOG SAFETY:
//   - Jog continues only while JOG <dir> keepalives arrive; none within
//     JOG_TIMEOUT_MS -> stop and brake.
//   - Soft limits are checked BEFORE releasing the brake and before every step.
//   - A jog that is rejected or stopped at a limit / by conflict stays
//     "latched": further JOG in that direction is ignored until JOG STOP or a
//     keepalive gap longer than JOG_TIMEOUT_MS (i.e. the button was released).
//
// USB SERIAL PROTOCOL (one command per line, case-insensitive)
//
// Raspberry Pi -> Teensy:
//   CAL               Home arm to limit switch (rejected unless idle).
//   ARM <deg>         Absolute target, clamped to [ARM_MIN_DEG, ARM_MAX_DEG].
//                     Interrupts any move/jog (stop, brake), then moves.
//   STEP <deg>        Relative target (taps). Added to the in-progress target
//                     if a move is running, else to the current angle.
//   JOG UP|DOWN       Start / keep alive a jog in that direction.
//   JOG STOP          End a jog (button released).
//   DRUM <steps_s>    Drum speed, +/-DRUM_MAX_STEPS_S. Continuous (no timeout).
//                     Applied immediately, does not interrupt the arm.
//   ARM_SPEED <deg_s> Arm speed for moves and jogs, clamped to
//                     [ARM_SPEED_MIN_DEG_S, ARM_SPEED_MAX_DEG_S]; default 5.625.
//                     Applied from the next step; homing always uses the default.
//   STOP              Stop everything: arm (stop, then brake) and drums.
//
// Teensy -> Raspberry Pi:
//   CAL_START / CAL_DONE / CAL_ABORTED   Homing status (ABORTED = not homed)
//   ARM_DONE <deg>                       Move complete, brake engaged
//   JOG_DONE <reason> <deg>              Jog ended, brake engaged
//                                        reason: RELEASED|TIMEOUT|LIMIT|
//                                                CONFLICT|INTERRUPTED|FAULT
//   JOG_LIMIT <deg>                      Jog refused at limit (brake never released)
//   STOPPED                              Stop complete, brake engaged
//   ARM_SPEED <deg_s>                    Arm speed now in use (reply to ARM_SPEED)
//   STATE <homed> <arm_deg> <IDLE|MOVING|JOGGING|HOMING> <drum_steps_s>  10 Hz
//   ERR NOT_HOMED | ERR BUSY | ERR UNKNOWN_COMMAND | ERR BRAKE_NOT_RELEASED
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

// --- ARM STEP TIMING ---
// Default = the original sketch's 800 us half-period (625 steps/s = 5.625 deg/s).
// ARM_SPEED <deg_s> changes it for moves and jogs; homing always uses the
// default. The max is the original speed: raise ARM_SPEED_MAX_DEG_S only after
// checking for missed steps (encoder vs. commanded angle) at the higher speed.
const unsigned long ARM_STEP_HALF_PERIOD_US_DEFAULT = 800;
const float ARM_SPEED_DEFAULT_DEG_S =
  1000000.0 / (2.0 * ARM_STEP_HALF_PERIOD_US_DEFAULT * STEPS_PER_ARM_DEG);
const float ARM_SPEED_MIN_DEG_S = 0.5;
const float ARM_SPEED_MAX_DEG_S = ARM_SPEED_DEFAULT_DEG_S;
unsigned long armStepHalfPeriodUs = ARM_STEP_HALF_PERIOD_US_DEFAULT;

// --- JOG ---
const unsigned long JOG_TIMEOUT_MS = 250;    // No keepalive within this -> stop

// --- DRUM ---
const long DRUM_MAX_STEPS_S = 3200;
const unsigned int DRUM_MIN_PULSE_US = 10;   // TB6600 needs a few us; AccelStepper default is 1 us

const unsigned long TELEMETRY_INTERVAL_MS = 100;

// --- OBJECTS & GLOBAL STATES ---
Encoder myEncoder(ENCODER_A, ENCODER_B);
AccelStepper drum1(AccelStepper::DRIVER, N17_STEP, N17_DIR);

bool isHomed = false;
bool isDrumMoving = false;
long drumCommand = 0;

// Arm activity, for telemetry and command gating.
enum ArmActivity { ACT_IDLE, ACT_MOVING, ACT_JOGGING, ACT_HOMING };
ArmActivity armActivity = ACT_IDLE;

// Commands that run the arm are queued here and executed by loop().
// STOP, ARM and JOG also act as interrupts for an in-progress move.
enum PendingCmd { CMD_NONE, CMD_ARM, CMD_JOG, CMD_CAL, CMD_STOP };
PendingCmd pendingCmd = CMD_NONE;
float pendingArmDeg = 0.0;
int pendingJogDir = 0;

// Target of the move currently executing (for STEP accumulation).
float activeMoveTargetDeg = 0.0;

// Jog state
int jogActiveDir = 0;                  // +1 up, -1 down, 0 none
bool jogStopRequested = false;         // JOG STOP or conflict received
const char *jogStopReason = "RELEASED";
unsigned long lastJogKeepaliveMs = 0;  // Last JOG <dir> for the active jog
unsigned long lastJogRxMs = 0;         // Last JOG line of any kind
int jogLatch = 0;                      // 0 none, +1/-1 that dir, 2 both

const size_t RX_BUFFER_SIZE = 64;
char rxBuffer[RX_BUFFER_SIZE];
size_t rxIndex = 0;

unsigned long lastTelemetryMs = 0;

void setDrumSpeed(long target_speed);

// =============================================================================
// DRUM SERVICING (called during every arm wait so drums never freeze)
// =============================================================================

void serviceDrum() {
  if (isDrumMoving) {
    drum1.run();
    if (drum1.distanceToGo() == 0) {
      drum1.disableOutputs();
      isDrumMoving = false;
    }
  }
}

// Same durations as delay() / delayMicroseconds(), but keeps the drum stepping.
void waitMsServicingDrum(unsigned long ms) {
  const unsigned long start = millis();
  while (millis() - start < ms) {
    serviceDrum();
  }
}

void waitUsServicingDrum(unsigned long us) {
  const unsigned long start = micros();
  while (micros() - start < us) {
    serviceDrum();
  }
}

// =============================================================================
// BRAKE
// =============================================================================

// --- BRAKE INTERLOCK STATE ---
bool brakeReleased = false;          // Relay HIGH
unsigned long brakeReleasedAtMs = 0; // When the relay went HIGH
bool inStepLoop = false;             // True while any arm step loop is running

// Same relay action and timing as the original sketch, plus:
//   - releaseBrake() records when the brake was released.
//   - engageBrake() refuses while a step loop is running, and waits
//     SETTLE_MS after the last step before engaging.
//   - Waits keep the drum stepping instead of blocking it.
void releaseBrake() {
  digitalWrite(BRAKE_RELAY, HIGH); // Transistor ON -> IN1 Grounded -> Releases 24V Brake
  brakeReleased = true;
  brakeReleasedAtMs = millis();
  waitMsServicingDrum(BRAKE_RELEASE_MS);
}

bool engageBrake() {
  if (inStepLoop) {
    Serial.println("# REFUSED brake engage: arm step loop active");
    return false;
  }
  waitMsServicingDrum(SETTLE_MS);  // Motors stopped and settled before locking
  digitalWrite(BRAKE_RELAY, LOW);  // Transistor OFF -> Spring-locks Brake
  brakeReleased = false;
  waitMsServicingDrum(BRAKE_ENGAGE_MS);
  return true;
}

// --- CALCULATION HELPERS ---
float getPhysicalArmDegrees() {
  long rawCounts = myEncoder.read();
  return rawCounts / ENCODER_CPR_ARM_DEG;
}

float clampArmDeg(float deg) {
  if (deg > ARM_MAX_DEG) return ARM_MAX_DEG;
  if (deg < ARM_MIN_DEG) return ARM_MIN_DEG;
  return deg;
}

// Set inverted relative directions between the two arm motors.
void setArmDirection(bool up) {
  if (up) {
    digitalWrite(MOTOR_ENC_DIR, HIGH); // Encoder Motor Upward
    digitalWrite(MOTOR_BRK_DIR, LOW);  // Brake Motor Opposite
  } else {
    digitalWrite(MOTOR_ENC_DIR, LOW);  // Encoder Motor Downward
    digitalWrite(MOTOR_BRK_DIR, HIGH); // Brake Motor Opposite
  }
}

// --- ONE STEP PULSE ON BOTH ARM MOTORS (half-period = armStepHalfPeriodUs) ---
// Ends with step pins LOW. Refuses (returns false) unless the brake has been
// released for at least BRAKE_RELEASE_MS.
bool armStepPulse() {
  if (!brakeReleased || millis() - brakeReleasedAtMs < BRAKE_RELEASE_MS) {
    Serial.println("ERR BRAKE_NOT_RELEASED");
    return false;
  }

  digitalWrite(MOTOR_ENC_STEP, HIGH);
  digitalWrite(MOTOR_BRK_STEP, HIGH);
  waitUsServicingDrum(armStepHalfPeriodUs);
  digitalWrite(MOTOR_ENC_STEP, LOW);
  digitalWrite(MOTOR_BRK_STEP, LOW);
  waitUsServicingDrum(armStepHalfPeriodUs);
  return true;
}

// Arm speed for moves and jogs, clamped to [ARM_SPEED_MIN_DEG_S, ARM_SPEED_MAX_DEG_S].
// Takes effect on the next step pulse (a move in progress changes speed).
// Returns the speed actually applied.
float setArmSpeedDegS(float deg_s) {
  if (!(deg_s >= ARM_SPEED_MIN_DEG_S)) deg_s = ARM_SPEED_MIN_DEG_S;   // also catches NaN
  if (deg_s > ARM_SPEED_MAX_DEG_S) deg_s = ARM_SPEED_MAX_DEG_S;
  armStepHalfPeriodUs = (unsigned long)(1000000.0 / (2.0 * deg_s * STEPS_PER_ARM_DEG) + 0.5);
  return 1000000.0 / (2.0 * armStepHalfPeriodUs * STEPS_PER_ARM_DEG);
}

// =============================================================================
// SERIAL (non-blocking line reader, safe to call inside step loops)
// =============================================================================

const char *activityName() {
  switch (armActivity) {
    case ACT_MOVING:  return "MOVING";
    case ACT_JOGGING: return "JOGGING";
    case ACT_HOMING:  return "HOMING";
    default:          return "IDLE";
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

// Reject a jog request once, then ignore repeats until released.
void rejectJog(int dir, const char *reply) {
  if (jogLatch != dir && jogLatch != 2) {
    Serial.println(reply);
  }
  if (jogLatch == 0) {
    jogLatch = dir;
  } else if (jogLatch != dir) {
    jogLatch = 2;
  }
}

void handleJogLine(int dir) {
  const unsigned long now = millis();

  // A gap longer than the timeout means the button was released: unlatch.
  if (now - lastJogRxMs > JOG_TIMEOUT_MS) {
    jogLatch = 0;
  }
  lastJogRxMs = now;

  if (dir == 0) {                       // JOG STOP
    jogLatch = 0;
    if (armActivity == ACT_JOGGING) {
      jogStopRequested = true;
      jogStopReason = "RELEASED";
    }
    if (pendingCmd == CMD_JOG) {
      pendingCmd = CMD_NONE;
    }
    return;
  }

  if (jogLatch == dir || jogLatch == 2) {
    return;                             // Latched: ignore until released
  }

  if (pendingCmd == CMD_STOP) {
    return;
  }

  if (armActivity == ACT_JOGGING) {
    if (dir == jogActiveDir) {
      lastJogKeepaliveMs = now;         // Keepalive
    } else {
      jogStopRequested = true;          // Conflicting direction: stop
      jogStopReason = "CONFLICT";
      jogLatch = 2;
    }
    return;
  }

  if (armActivity == ACT_HOMING) {
    rejectJog(dir, "ERR BUSY");
    return;
  }
  if (!isHomed) {
    rejectJog(dir, "ERR NOT_HOMED");
    return;
  }

  if (pendingCmd == CMD_JOG && pendingJogDir != dir) {
    pendingCmd = CMD_NONE;              // Conflicting queued jog: drop both
    jogLatch = 2;
    return;
  }

  pendingCmd = CMD_JOG;                 // Start (interrupts a move if running)
  pendingJogDir = dir;
}

void handleLine(char *line) {
  for (char *p = line; *p; ++p) {
    *p = toupper(*p);
  }

  if (strcmp(line, "STOP") == 0) {
    setDrumSpeed(0);
    pendingCmd = CMD_STOP;             // Overrides any queued command
    if (armActivity == ACT_JOGGING) {
      jogStopRequested = true;
      jogStopReason = "INTERRUPTED";
    }
  }
  else if (strncmp(line, "DRUM ", 5) == 0) {
    setDrumSpeed(atol(line + 5));      // Immediate; does not touch the arm
  }
  else if (strncmp(line, "ARM_SPEED ", 10) == 0) {
    if (armActivity == ACT_HOMING) {
      Serial.println("ERR BUSY");        // Homing always runs at the default speed
    } else {
      const float applied = setArmSpeedDegS(atof(line + 10));
      Serial.print("ARM_SPEED ");
      Serial.println(applied, 3);
    }
  }
  else if (strcmp(line, "JOG UP") == 0) {
    handleJogLine(+1);
  }
  else if (strcmp(line, "JOG DOWN") == 0) {
    handleJogLine(-1);
  }
  else if (strcmp(line, "JOG STOP") == 0) {
    handleJogLine(0);
  }
  else if (strncmp(line, "ARM ", 4) == 0) {
    if (armActivity == ACT_HOMING) {
      Serial.println("ERR BUSY");
    } else if (pendingCmd != CMD_STOP) {
      pendingCmd = CMD_ARM;
      pendingArmDeg = atof(line + 4);
    }
  }
  else if (strncmp(line, "STEP ", 5) == 0) {
    if (armActivity == ACT_HOMING) {
      Serial.println("ERR BUSY");
    } else if (!isHomed) {
      Serial.println("ERR NOT_HOMED");
    } else if (pendingCmd != CMD_STOP) {
      // Accumulate onto whatever target is already in flight.
      float base;
      if (pendingCmd == CMD_ARM) {
        base = pendingArmDeg;
      } else if (armActivity == ACT_MOVING) {
        base = activeMoveTargetDeg;
      } else {
        base = getPhysicalArmDegrees();
      }
      pendingCmd = CMD_ARM;
      pendingArmDeg = clampArmDeg(base + atof(line + 5));
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
  return pendingCmd == CMD_STOP || pendingCmd == CMD_ARM || pendingCmd == CMD_JOG;
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
  setArmDirection(true);

  inStepLoop = true;

  while (digitalRead(LIMIT_SW_PIN) == HIGH) {
    if (homingInterrupted() || !armStepPulse()) { aborted = true; break; }
  }

  if (!aborted) {
    // Back off limit switch in opposite directions
    setArmDirection(false);
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

  targetArmDeg = clampArmDeg(targetArmDeg);

  float currentDeg = getPhysicalArmDegrees();
  float deltaDeg = targetArmDeg - currentDeg;

  int stepsToMove = abs(deltaDeg) * STEPS_PER_ARM_DEG;

  if (stepsToMove == 0) {
    Serial.print("ARM_DONE ");
    Serial.println(currentDeg, 2);
    return;
  }

  armActivity = ACT_MOVING;
  activeMoveTargetDeg = targetArmDeg;
  releaseBrake();

  setArmDirection(deltaDeg >= 0);

  bool interrupted = false;
  inStepLoop = true;
  for (int i = 0; i < stepsToMove; i++) {
    if (moveInterrupted() || !armStepPulse()) { interrupted = true; break; }
  }

  // Step loop exited: step pins are LOW. Settle, then lock.
  inStepLoop = false;
  engageBrake();
  armActivity = ACT_IDLE;

  // STOP is reported by loop(); a queued ARM/JOG runs next from loop().
  if (!interrupted) {
    Serial.print("ARM_DONE ");
    Serial.println(getPhysicalArmDegrees(), 2);
  }
}

bool atLimit(int dir, float deg) {
  return (dir > 0 && deg >= ARM_MAX_DEG) || (dir < 0 && deg <= ARM_MIN_DEG);
}

void jogArm(int dir) {
  if (!isHomed) {
    rejectJog(dir, "ERR NOT_HOMED");
    return;
  }

  // Limit check BEFORE releasing the brake: holding into a limit never
  // cycles the brake.
  const float startDeg = getPhysicalArmDegrees();
  if (atLimit(dir, startDeg)) {
    if (jogLatch != dir && jogLatch != 2) {
      Serial.print("JOG_LIMIT ");
      Serial.println(startDeg, 2);
    }
    jogLatch = (jogLatch == 0 || jogLatch == dir) ? dir : 2;
    return;
  }

  armActivity = ACT_JOGGING;
  jogActiveDir = dir;
  jogStopRequested = false;
  jogStopReason = "RELEASED";
  lastJogKeepaliveMs = millis();

  releaseBrake();
  setArmDirection(dir > 0);

  inStepLoop = true;
  while (true) {
    pollSerial();
    sendTelemetryIfDue();

    if (pendingCmd == CMD_STOP || pendingCmd == CMD_ARM) {
      jogStopReason = "INTERRUPTED";
      break;
    }
    if (jogStopRequested) {
      break;                                   // RELEASED or CONFLICT
    }
    if (millis() - lastJogKeepaliveMs > JOG_TIMEOUT_MS) {
      jogStopReason = "TIMEOUT";
      break;
    }
    if (atLimit(dir, getPhysicalArmDegrees())) {
      jogStopReason = "LIMIT";
      jogLatch = (jogLatch == 0 || jogLatch == dir) ? dir : 2;
      break;
    }
    if (!armStepPulse()) {
      jogStopReason = "FAULT";
      break;
    }
  }

  // Step loop exited: step pins are LOW. Settle, then lock.
  inStepLoop = false;
  engageBrake();
  armActivity = ACT_IDLE;
  jogActiveDir = 0;
  jogStopRequested = false;

  // A jog queued during the stop is stale (same button still held): drop it.
  if (pendingCmd == CMD_JOG) {
    pendingCmd = CMD_NONE;
  }

  Serial.print("JOG_DONE ");
  Serial.print(jogStopReason);
  Serial.print(' ');
  Serial.println(getPhysicalArmDegrees(), 2);
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
  drum1.setMinPulseWidth(DRUM_MIN_PULSE_US);
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
  const int jogDir = pendingJogDir;
  pendingCmd = CMD_NONE;

  switch (cmd) {
    case CMD_STOP:
      // Arm is idle here with brake engaged (any move already exited).
      Serial.println("STOPPED");
      break;
    case CMD_ARM:
      moveToPhysicalAngle(deg);
      break;
    case CMD_JOG:
      jogArm(jogDir);
      break;
    case CMD_CAL: {
      // Homing always runs at the default speed; restore the set speed after.
      const unsigned long saved = armStepHalfPeriodUs;
      armStepHalfPeriodUs = ARM_STEP_HALF_PERIOD_US_DEFAULT;
      homeSystem();
      armStepHalfPeriodUs = saved;
      break;
    }
    default:
      break;
  }

  serviceDrum();
  sendTelemetryIfDue();
}
