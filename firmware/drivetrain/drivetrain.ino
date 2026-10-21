#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <RoboClaw.h>
#include <EEPROM.h>

// Saved current-sensor zeros (see CALIBRATION STORAGE below).
struct ZeroCalibration {
  uint32_t magic;
  float zero_v[4];
  uint32_t checksum;
};

#define LED_PIN 13
#define RC_ADDRESS 0x80

// --- MOTOR DIRECTION CONFIGURATION ---
#define DIR_FRONT_RIGHT -1.0f
#define DIR_FRONT_LEFT  -1.0f
#define DIR_BACK_RIGHT  -1.0f
#define DIR_BACK_LEFT   -1.0f

// Serial1: Back Axle  (M1 = Back Left,  M2 = Back Right)
// Serial3: Front Axle (M1 = Front Right, M2 = Front Left)
RoboClaw roboclaw1(&Serial1, 10000);
RoboClaw roboclaw2(&Serial3, 10000);

// -----------------------------------------------------------------------------
// USB SERIAL PROTOCOL
//
// Raspberry Pi -> Teensy:
//   DRIVE <left_wheel_rad_s> <right_wheel_rad_s>
//   CAL_CURRENT               (zero drive current sensors; motors still; saved to EEPROM)
//   CAL_ARM_CURRENT           (zero arm current sensors; loads disconnected; saved to EEPROM)
//   CAL_SHOW                  (print the drive + arm zeros in use)
//   STOP                      (stop all motors immediately)
//
// Teensy -> Raspberry Pi:
//   STOPPED
//   VOLT <back_left> <back_right> <front_right> <front_left>
//   CURR <back_left> <back_right> <front_right> <front_left>
//   ARM_CURR <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>   (A)
//   ARM_VOLT <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>   (V)
//   BATT <drivetrain_battery_V> <pi_arm_battery_V>                       (V)
//   RC_CURR <back_left> <back_right> <front_right> <front_left>          (A, from RoboClaw)
//   SPEED <back_left> <back_right> <front_right> <front_left>            (wheel rad/s, RoboClaw encoders)
//   ENC <back_left> <back_right> <front_right> <front_left>              (wheel rad, RoboClaw encoders)
//   CAL_CURRENT START / CAL_CURRENT OK SAVED <z0..z3> / CAL_CURRENT REJECTED ...
//   CAL_ARM_CURRENT START / CAL_ARM_CURRENT OK SAVED <z0..z3> / ... REJECTED ...
//   CAL_SHOW DRIVE <z0..z3> / CAL_SHOW ARM <z0..z3>
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// CLOSED-LOOP WHEEL SPEED (RoboClaw velocity PID)
//
// DRIVE commands are sent as RoboClaw speed commands (SpeedAccelM1M2), which
// use the velocity PID tuned in Motion Studio. Units are QPPS (quadrature
// encoder counts per second).
//
// Motors: goBILDA 5304 Saturn, 71.2:1, 1993.6 counts per output-shaft rev
// (quadrature). Wheels mount directly on the output shaft.
// -----------------------------------------------------------------------------
constexpr float COUNTS_PER_WHEEL_REV = 1993.6f;
constexpr float WHEEL_RAD_S_TO_QPPS = COUNTS_PER_WHEEL_REV / (2.0f * PI);

// Max commanded speed: 260 RPM rated no-load * 1993.6 / 60 = 8639 QPPS.
constexpr float MAX_QPPS = 8640.0f;

// Ramp rate for speed changes (QPPS per second).
constexpr uint32_t ACCEL_QPPS_S = 4000;

// Fail-safe: if valid DRIVE commands stop arriving, stop all motors.
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;

// Telemetry at 10 Hz.
constexpr unsigned long TELEMETRY_INTERVAL_MS = 100;

// -----------------------------------------------------------------------------
// CURRENT SENSOR CONFIGURATION
//
// Planned wiring/order:
//   pin 20 -> Back Left
//   pin 21 -> Back Right
//   pin 22 -> Front Right
//   pin 23 -> Front Left
//
// Keep this ordering aligned with VOLT telemetry and ROS topic names.
// If the physical wiring differs, change ONLY these pin assignments.
// -----------------------------------------------------------------------------
constexpr int NUM_CURRENT_CHANNELS = 4;
const int CURRENT_SENSOR_PINS[NUM_CURRENT_CHANNELS] = {20, 21, 22, 23};

// ACS712 sensitivity by chip marking (V/A at the sensor output):
//   ELC-05B (5 A)  = 0.185     ELC-20A (20 A) = 0.100     ELC-30A (30 A) = 0.066
// The 5 A chip saturates at +/-5 A.
constexpr float ACS712_05A = 0.185f;
constexpr float ACS712_20A = 0.100f;
constexpr float ACS712_30A = 0.066f;

// ACS712 on each drive motor lead + external 10k/20k divider.
constexpr float ADC_VREF = 3.3f;
constexpr float ADC_MAX = 1023.0f;             // 10-bit ADC
constexpr float CURRENT_DIVIDER_RATIO = 1.5f; // reconstruct ACS712 output voltage

// Per channel, order: back left, back right, front right, front left.
// All four read ELC-30A (chip markings checked 2026-10-18). Original firmware
// assumed 5 A, which made every drive current read ~2.8x too low.
const float CURRENT_SENSITIVITY[NUM_CURRENT_CHANNELS] = {
  ACS712_30A, ACS712_30A, ACS712_30A, ACS712_30A};

constexpr int CURRENT_OVERSAMPLE_COUNT = 128;
// Light smoothing between 10 Hz samples (1.0 = none). Was 0.08 (~1.2 s lag),
// which hid stall spikes.
constexpr float CURRENT_ALPHA = 0.5f;
constexpr int CURRENT_CALIBRATION_SAMPLES = 500;

// Off: drive channels use the stored zeros below. A boot-time zero is wrong
// whenever the sensors' 5 V is not on yet when the Teensy boots.
constexpr bool AUTO_CALIBRATE_CURRENT_AT_BOOT = false;

// ACS712 output at 0 A, in sensor-side volts (nominal = 5 V supply / 2).
// Order: back left, back right, front right, front left (pins 20-23).
// To set: sensors powered, drive motors still, send CAL_CURRENT, paste the
// printed values here, reflash.
float current_zero_offsets[NUM_CURRENT_CHANNELS] = {2.5f, 2.5f, 2.5f, 2.5f};

// EMA state (pin volts), started at the nominal zero so readings settle at once.
float current_smoothed_pin_voltages[NUM_CURRENT_CHANNELS] = {
  2.5f / 1.5f, 2.5f / 1.5f, 2.5f / 1.5f, 2.5f / 1.5f};

// -----------------------------------------------------------------------------
// ARM POWER SENSORS (current + voltage on each arm's driver supply)
//
// Channel order (all ARM_* arrays and telemetry):
//   [0] rear drum   [1] rear shoulder   [2] front drum   [3] front shoulder
//
// Sensors: UMLIFE kit -- ACS712 30A current module + DC 0-25V voltage module.
// Teensy 4.x analog pins are 3.3 V MAX (not 5 V tolerant). Every module
// output goes through a 5 V -> 3.3 V divider (ratio 1.5):
//   - Current: ACS712 output / 1.5 -> ARM_CURRENT_DIVIDER_RATIO = 1.5.
//   - Voltage: 0-25V module (passive divider, no output cap) then the
//     5 V -> 3.3 V divider. Measured total ratio on the rover = 9.10:1
//     (27.77 V -> 3.05 V). Full 7S pack (29.4 V) -> 3.23 V at the pin;
//     full scale at 3.3 V = ~30 V. Little margin: a 15k bottom resistor
//     (instead of 20k) would give ~10.3:1.
// -----------------------------------------------------------------------------
constexpr int NUM_ARM_CHANNELS = 4;
// Wiring: rear arm on 16-19, front arm on 38-41.
//   16 rear drum I     17 rear drum V     18 rear shoulder I   19 rear shoulder V
//   38 front shldr V   39 front shldr I   40 front drum V      41 front drum I
const int ARM_CURRENT_PINS[NUM_ARM_CHANNELS] = {16, 18, 41, 39};
const int ARM_VOLTAGE_PINS[NUM_ARM_CHANNELS] = {17, 19, 40, 38};

constexpr float ARM_CURRENT_DIVIDER_RATIO = 1.5f;   // reconstruct ACS712 output voltage
// Per channel (chip markings read 2026-10-18): drums 30 A, shoulders 5 A.
// Order: rear drum, rear shoulder, front drum, front shoulder.
const float ARM_CURRENT_SENSITIVITY[NUM_ARM_CHANNELS] = {
  ACS712_30A, ACS712_05A, ACS712_30A, ACS712_05A};

// ACS712 output at 0 A, in sensor-side volts (nominal = 5 V supply / 2).
//   All channels use these stored values (no boot auto-zero; see
//     AUTO_ZERO_ARM_DRUMS_AT_BOOT). Re-run CAL_ARM_CURRENT only if a sensor or
//     the 5 V supply changes.
//   Shoulders [1],[3]: always energized (holding current), so never at 0 A
//     in operation. Zero ONCE by hand: disconnect the shoulder driver supply
//     leads downstream of the sensors, send CAL_ARM_CURRENT, paste the printed
//     values here, reflash. Redo only if the sensors or 5 V supply change.
// Measured 2026-10-15 with CAL_ARM_CURRENT (sensors powered, no load current).
float arm_current_zero_v[NUM_ARM_CHANNELS] = {2.4487f, 2.6483f, 2.4474f, 2.4952f};

// Off: all four arm channels use the stored manual zeros above. A boot-time
// zero is unreliable (sensors may not have 5 V yet, drums may be moving).
constexpr bool AUTO_ZERO_ARM_DRUMS_AT_BOOT = false;
const bool ARM_CHANNEL_IS_DRUM[NUM_ARM_CHANNELS] = {true, false, true, false};

// Measured volts per pin volt, on the Teensy (2026-10-18): 27.77 V in,
// 3.05 V at every arm voltage pin -> 9.10. Matches the circuit (module
// 30k/7.5k loaded by 10k/20k = 9.0). Full 7S pack (29.4 V) -> 3.23 V at pin.
const float ARM_VOLTAGE_DIVIDER_RATIO[NUM_ARM_CHANNELS] = {9.10f, 9.10f, 9.10f, 9.10f};

constexpr int ARM_OVERSAMPLE_COUNT = 64;
// Light smoothing between 10 Hz samples (1.0 = none). Kept low so power
// transients stay visible for characterization.
constexpr float ARM_ALPHA = 0.5f;

float arm_current_smoothed[NUM_ARM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};
float arm_voltage_smoothed[NUM_ARM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};
bool arm_sensors_initialized = false;

// -----------------------------------------------------------------------------
// BATTERY VOLTAGE SENSORS (same 0-25V module + 5V->3.3V divider as the arm
// voltage channels), inline on each battery's main leads.
//   [0] drivetrain battery   [1] Pi / arm battery
// -----------------------------------------------------------------------------
constexpr int NUM_BATTERY_CHANNELS = 2;
const int BATTERY_VOLTAGE_PINS[NUM_BATTERY_CHANNELS] = {24, 25};

// Measured volts per pin volt. Same sensor + divider as the arm voltage
// channels (9.10 measured there); VERIFY with a multimeter once wired.
const float BATTERY_VOLTAGE_DIVIDER_RATIO[NUM_BATTERY_CHANNELS] = {9.10f, 9.10f};

float battery_voltage_smoothed[NUM_BATTERY_CHANNELS] = {0.0f, 0.0f};
bool battery_sensors_initialized = false;

constexpr size_t RX_BUFFER_SIZE = 96;
char rx_buffer[RX_BUFFER_SIZE];
size_t rx_index = 0;

unsigned long last_command_time = 0;
bool have_command = false;

unsigned long last_telemetry_time = 0;


// -----------------------------------------------------------------------------
// MOTOR CONTROL
// -----------------------------------------------------------------------------

// Speed 0 under velocity PID: wheels are actively held at zero speed.
void stop_all_motors() {
  roboclaw1.SpeedM1M2(RC_ADDRESS, 0, 0);
  roboclaw2.SpeedM1M2(RC_ADDRESS, 0, 0);
}


// Wheel rad/s -> signed QPPS for one motor, clamped to +/-MAX_QPPS.
// The library takes uint32_t; RoboClaw interprets the bits as signed.
uint32_t to_qpps(float wheel_rad_s, float direction) {
  float qpps = wheel_rad_s * WHEEL_RAD_S_TO_QPPS * direction;
  if (qpps > MAX_QPPS) qpps = MAX_QPPS;
  if (qpps < -MAX_QPPS) qpps = -MAX_QPPS;
  return (uint32_t)(int32_t)lroundf(qpps);
}


void command_drive(float left_wheel_rad_s, float right_wheel_rad_s) {
  // Front Axle (RoboClaw 2): M1 = Right, M2 = Left
  roboclaw2.SpeedAccelM1M2(
    RC_ADDRESS, ACCEL_QPPS_S,
    to_qpps(right_wheel_rad_s, DIR_FRONT_RIGHT),
    to_qpps(left_wheel_rad_s,  DIR_FRONT_LEFT));

  // Back Axle (RoboClaw 1): M1 = Left, M2 = Right
  roboclaw1.SpeedAccelM1M2(
    RC_ADDRESS, ACCEL_QPPS_S,
    to_qpps(left_wheel_rad_s,  DIR_BACK_LEFT),
    to_qpps(right_wheel_rad_s, DIR_BACK_RIGHT));
}


// -----------------------------------------------------------------------------
// CURRENT SENSOR CALIBRATION / ACQUISITION
// -----------------------------------------------------------------------------

void flush_usb_rx() {
  while (Serial.available() > 0) {
    Serial.read();
  }
  rx_index = 0;
}


// -----------------------------------------------------------------------------
// CALIBRATION STORAGE (Teensy EEPROM)
//
// CAL_CURRENT / CAL_ARM_CURRENT save their zeros to EEPROM, so calibration
// survives reboots and power cycles without editing code or reflashing.
// At boot, saved values replace the compiled defaults; if nothing valid is
// saved, the compiled defaults are used.
//
// A calibration is REJECTED (nothing changed or saved) if any channel's zero
// is outside CAL_ZERO_MIN_V..CAL_ZERO_MAX_V, e.g. 0 V from an unpowered sensor.
// -----------------------------------------------------------------------------
constexpr float CAL_ZERO_MIN_V = 2.2f;   // ACS712 0 A output, sensor side
constexpr float CAL_ZERO_MAX_V = 2.8f;

constexpr uint32_t CAL_MAGIC = 0x52435831;  // "RCX1"

// (struct ZeroCalibration is defined at the top of the file so Arduino's
// auto-generated prototypes can see it.)

constexpr int EEPROM_ADDR_DRIVE_CAL = 0;
constexpr int EEPROM_ADDR_ARM_CAL = 64;

uint32_t cal_checksum(const ZeroCalibration &cal) {
  uint32_t sum = cal.magic;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(cal.zero_v);
  for (size_t i = 0; i < sizeof(cal.zero_v); ++i) {
    sum = (sum * 31u) + bytes[i];
  }
  return sum;
}

void save_zero_calibration(int addr, const float zeros[4]) {
  ZeroCalibration cal;
  cal.magic = CAL_MAGIC;
  for (int i = 0; i < 4; ++i) {
    cal.zero_v[i] = zeros[i];
  }
  cal.checksum = cal_checksum(cal);
  EEPROM.put(addr, cal);
}

// Returns true and fills zeros[] if a valid calibration is stored at addr.
bool load_zero_calibration(int addr, float zeros[4]) {
  ZeroCalibration cal;
  EEPROM.get(addr, cal);

  if (cal.magic != CAL_MAGIC || cal.checksum != cal_checksum(cal)) {
    return false;
  }
  for (int i = 0; i < 4; ++i) {
    if (!(cal.zero_v[i] >= CAL_ZERO_MIN_V && cal.zero_v[i] <= CAL_ZERO_MAX_V)) {
      return false;
    }
  }
  for (int i = 0; i < 4; ++i) {
    zeros[i] = cal.zero_v[i];
  }
  return true;
}

bool zeros_plausible(const char *name, const float zeros[4]) {
  for (int ch = 0; ch < 4; ++ch) {
    if (!(zeros[ch] >= CAL_ZERO_MIN_V && zeros[ch] <= CAL_ZERO_MAX_V)) {
      Serial.print(name);
      Serial.print(" REJECTED channel ");
      Serial.print(ch);
      Serial.print(" zero ");
      Serial.print(zeros[ch], 4);
      Serial.println(" V (sensor unpowered or current flowing?) -- nothing saved");
      return false;
    }
  }
  return true;
}

void print_zeros(const char *prefix, const float zeros[4]) {
  Serial.print(prefix);
  for (int ch = 0; ch < 4; ++ch) {
    Serial.print(zeros[ch], 4);
    Serial.print(ch < 3 ? ' ' : '\n');
  }
}


void calibrate_current_sensors() {
  // Calibration must happen at zero motor current.
  stop_all_motors();
  have_command = false;
  digitalWrite(LED_PIN, LOW);

  Serial.println("CAL_CURRENT START");

  // Let motor commands settle before measuring the zero-current baselines.
  delay(100);

  float measured[NUM_CURRENT_CHANNELS];

  for (int ch = 0; ch < NUM_CURRENT_CHANNELS; ++ch) {
    uint32_t adc_sum = 0;

    for (int i = 0; i < CURRENT_CALIBRATION_SAMPLES; ++i) {
      adc_sum += analogRead(CURRENT_SENSOR_PINS[ch]);
      delay(1);
    }

    const float avg_adc =
      adc_sum / static_cast<float>(CURRENT_CALIBRATION_SAMPLES);

    const float pin_voltage =
      (avg_adc / ADC_MAX) * ADC_VREF;

    measured[ch] = pin_voltage * CURRENT_DIVIDER_RATIO;
  }

  // If the Pi was already streaming DRIVE commands during the blocking
  // calibration window, discard those stale commands. Fresh commands arriving
  // after calibration will be processed normally.
  flush_usb_rx();

  if (!zeros_plausible("CAL_CURRENT", measured)) {
    return;
  }

  for (int ch = 0; ch < NUM_CURRENT_CHANNELS; ++ch) {
    current_zero_offsets[ch] = measured[ch];
    current_smoothed_pin_voltages[ch] = measured[ch] / CURRENT_DIVIDER_RATIO;
  }
  save_zero_calibration(EEPROM_ADDR_DRIVE_CAL, current_zero_offsets);

  print_zeros("CAL_CURRENT OK SAVED ", current_zero_offsets);
}


void read_currents(float currents[NUM_CURRENT_CHANNELS]) {
  for (int ch = 0; ch < NUM_CURRENT_CHANNELS; ++ch) {
    uint32_t adc_sum = 0;

    for (int i = 0; i < CURRENT_OVERSAMPLE_COUNT; ++i) {
      adc_sum += analogRead(CURRENT_SENSOR_PINS[ch]);
    }

    const float raw_pin_voltage =
      (adc_sum / static_cast<float>(CURRENT_OVERSAMPLE_COUNT) / ADC_MAX) *
      ADC_VREF;

    current_smoothed_pin_voltages[ch] =
      (CURRENT_ALPHA * raw_pin_voltage) +
      ((1.0f - CURRENT_ALPHA) * current_smoothed_pin_voltages[ch]);

    const float sensor_voltage =
      current_smoothed_pin_voltages[ch] * CURRENT_DIVIDER_RATIO;

    currents[ch] =
      (sensor_voltage - current_zero_offsets[ch]) /
      CURRENT_SENSITIVITY[ch];
  }
}


// -----------------------------------------------------------------------------
// ARM POWER SENSORS
// -----------------------------------------------------------------------------

// Average pin voltage over `samples` ADC reads.
float read_pin_voltage(int pin, int samples) {
  uint32_t adc_sum = 0;

  for (int i = 0; i < samples; ++i) {
    adc_sum += analogRead(pin);
  }

  return (adc_sum / static_cast<float>(samples) / ADC_MAX) * ADC_VREF;
}


// Measure the 0 A sensor-side output of one arm current channel.
float measure_arm_current_zero(int ch) {
  float pin_voltage_sum = 0.0f;

  for (int i = 0; i < CURRENT_CALIBRATION_SAMPLES; ++i) {
    pin_voltage_sum += read_pin_voltage(ARM_CURRENT_PINS[ch], 1);
    delay(1);
  }

  return (pin_voltage_sum / CURRENT_CALIBRATION_SAMPLES) * ARM_CURRENT_DIVIDER_RATIO;
}


// Boot: zero only the drum channels (drums are idle at boot). Shoulder
// channels keep their stored values.
void auto_zero_arm_drum_currents() {
  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    if (ARM_CHANNEL_IS_DRUM[ch]) {
      arm_current_zero_v[ch] = measure_arm_current_zero(ch);
    }
  }
  arm_sensors_initialized = false;
}


// Manual: zero ALL four channels. Only valid with NO current through the
// sensors (driver supply leads unplugged downstream of the sensors).
// Saves to EEPROM; rejected if any channel looks unpowered.
void calibrate_arm_current_sensors() {
  stop_all_motors();
  have_command = false;
  digitalWrite(LED_PIN, LOW);

  Serial.println("CAL_ARM_CURRENT START");

  float measured[NUM_ARM_CHANNELS];
  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    measured[ch] = measure_arm_current_zero(ch);
  }

  flush_usb_rx();

  if (!zeros_plausible("CAL_ARM_CURRENT", measured)) {
    return;
  }

  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    arm_current_zero_v[ch] = measured[ch];
  }
  arm_sensors_initialized = false;  // Restart smoothing from fresh readings
  save_zero_calibration(EEPROM_ADDR_ARM_CAL, arm_current_zero_v);

  print_zeros("CAL_ARM_CURRENT OK SAVED ", arm_current_zero_v);
}


void read_and_send_arm_sensors() {
  float currents[NUM_ARM_CHANNELS];
  float voltages[NUM_ARM_CHANNELS];

  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    const float sensor_v =
      read_pin_voltage(ARM_CURRENT_PINS[ch], ARM_OVERSAMPLE_COUNT) *
      ARM_CURRENT_DIVIDER_RATIO;
    currents[ch] = (sensor_v - arm_current_zero_v[ch]) / ARM_CURRENT_SENSITIVITY[ch];

    voltages[ch] =
      read_pin_voltage(ARM_VOLTAGE_PINS[ch], ARM_OVERSAMPLE_COUNT) *
      ARM_VOLTAGE_DIVIDER_RATIO[ch];
  }

  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    if (!arm_sensors_initialized) {
      arm_current_smoothed[ch] = currents[ch];
      arm_voltage_smoothed[ch] = voltages[ch];
    } else {
      arm_current_smoothed[ch] =
        ARM_ALPHA * currents[ch] + (1.0f - ARM_ALPHA) * arm_current_smoothed[ch];
      arm_voltage_smoothed[ch] =
        ARM_ALPHA * voltages[ch] + (1.0f - ARM_ALPHA) * arm_voltage_smoothed[ch];
    }
  }
  arm_sensors_initialized = true;

  // Order: rear drum, rear shoulder, front drum, front shoulder.
  Serial.print("ARM_CURR ");
  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    Serial.print(arm_current_smoothed[ch], 3);
    Serial.print(ch < NUM_ARM_CHANNELS - 1 ? ' ' : '\n');
  }

  Serial.print("ARM_VOLT ");
  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    Serial.print(arm_voltage_smoothed[ch], 3);
    Serial.print(ch < NUM_ARM_CHANNELS - 1 ? ' ' : '\n');
  }
}


// Order: drivetrain battery, Pi / arm battery.
void read_and_send_battery_voltages() {
  for (int ch = 0; ch < NUM_BATTERY_CHANNELS; ++ch) {
    const float volts =
      read_pin_voltage(BATTERY_VOLTAGE_PINS[ch], ARM_OVERSAMPLE_COUNT) *
      BATTERY_VOLTAGE_DIVIDER_RATIO[ch];

    battery_voltage_smoothed[ch] = battery_sensors_initialized
      ? ARM_ALPHA * volts + (1.0f - ARM_ALPHA) * battery_voltage_smoothed[ch]
      : volts;
  }
  battery_sensors_initialized = true;

  Serial.print("BATT ");
  Serial.print(battery_voltage_smoothed[0], 3);
  Serial.print(' ');
  Serial.println(battery_voltage_smoothed[1], 3);
}


// -----------------------------------------------------------------------------
// USB COMMAND INPUT
// -----------------------------------------------------------------------------

void handle_command(char *line) {
  float left_wheel_rad_s = 0.0f;
  float right_wheel_rad_s = 0.0f;

  if (sscanf(line, "DRIVE %f %f", &left_wheel_rad_s, &right_wheel_rad_s) == 2) {
    command_drive(left_wheel_rad_s, right_wheel_rad_s);
    last_command_time = millis();
    have_command = true;
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  if (strcmp(line, "CAL_CURRENT") == 0) {
    calibrate_current_sensors();
    return;
  }

  if (strcmp(line, "CAL_ARM_CURRENT") == 0) {
    calibrate_arm_current_sensors();
    return;
  }

  if (strcmp(line, "CAL_SHOW") == 0) {
    print_zeros("CAL_SHOW DRIVE ", current_zero_offsets);
    print_zeros("CAL_SHOW ARM ", arm_current_zero_v);
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    stop_all_motors();
    have_command = false;
    digitalWrite(LED_PIN, LOW);
    Serial.println("STOPPED");
    return;
  }
}


void process_usb_serial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      rx_buffer[rx_index] = '\0';

      if (rx_index > 0) {
        handle_command(rx_buffer);
      }

      rx_index = 0;
      continue;
    }

    if (rx_index < RX_BUFFER_SIZE - 1) {
      rx_buffer[rx_index++] = c;
    } else {
      // Malformed/oversized command: discard it rather than execute partial data.
      rx_index = 0;
    }
  }
}


// -----------------------------------------------------------------------------
// TELEMETRY
// -----------------------------------------------------------------------------

void read_and_send_voltages() {
  bool valid1 = false;
  bool valid2 = false;

  // RoboClaw 1 = back axle, RoboClaw 2 = front axle.
  const uint16_t raw_v1 =
    roboclaw1.ReadMainBatteryVoltage(RC_ADDRESS, &valid1);

  const uint16_t raw_v2 =
    roboclaw2.ReadMainBatteryVoltage(RC_ADDRESS, &valid2);

  const float v_main1 = valid1 ? (raw_v1 / 10.0f) : 0.0f;
  const float v_main2 = valid2 ? (raw_v2 / 10.0f) : 0.0f;

  int16_t pwm1_m1 = 0;
  int16_t pwm1_m2 = 0;
  int16_t pwm2_m1 = 0;
  int16_t pwm2_m2 = 0;

  roboclaw1.ReadPWMs(RC_ADDRESS, pwm1_m1, pwm1_m2);
  roboclaw2.ReadPWMs(RC_ADDRESS, pwm2_m1, pwm2_m2);

  const float v_back_left =
    v_main1 * (abs(pwm1_m1) / 32767.0f);

  const float v_back_right =
    v_main1 * (abs(pwm1_m2) / 32767.0f);

  const float v_front_right =
    v_main2 * (abs(pwm2_m1) / 32767.0f);

  const float v_front_left =
    v_main2 * (abs(pwm2_m2) / 32767.0f);

  Serial.print("VOLT ");
  Serial.print(v_back_left, 3);
  Serial.print(' ');
  Serial.print(v_back_right, 3);
  Serial.print(' ');
  Serial.print(v_front_right, 3);
  Serial.print(' ');
  Serial.println(v_front_left, 3);
}


// Motor currents as measured by the RoboClaws themselves (what their current
// limit acts on). RoboClaw reports signed values in 10 mA units.
// Same order as CURR: back left, back right, front right, front left.
// Prints "nan" for a RoboClaw that did not answer.
void read_and_send_roboclaw_currents() {
  int16_t back_m1 = 0, back_m2 = 0, front_m1 = 0, front_m2 = 0;

  const bool back_valid = roboclaw1.ReadCurrents(RC_ADDRESS, back_m1, back_m2);
  const bool front_valid = roboclaw2.ReadCurrents(RC_ADDRESS, front_m1, front_m2);

  // RoboClaw 1 (back): M1 = back left, M2 = back right.
  // RoboClaw 2 (front): M1 = front right, M2 = front left.
  const float values[4] = {
    back_valid ? back_m1 / 100.0f : NAN,
    back_valid ? back_m2 / 100.0f : NAN,
    front_valid ? front_m1 / 100.0f : NAN,
    front_valid ? front_m2 / 100.0f : NAN,
  };

  Serial.print("RC_CURR ");
  for (int i = 0; i < 4; ++i) {
    Serial.print(values[i], 2);
    Serial.print(i < 3 ? ' ' : '\n');
  }
}


// RoboClaw speed reading (QPPS) as a signed value. The status byte's bit 0
// marks reverse; handle both firmware styles (magnitude + flag, or signed).
float signed_qpps(uint32_t raw, uint8_t status) {
  int32_t qpps = (int32_t)raw;
  if ((status & 0x01) && qpps > 0) qpps = -qpps;
  return (float)qpps;
}


// Measured wheel motion from the RoboClaws' encoders, converted to the same
// wheel frame as DRIVE (positive = rover forward on that side).
// Same order as CURR: back left, back right, front right, front left.
//   SPEED <bl> <br> <fr> <fl>   wheel speed, rad/s
//   ENC <bl> <br> <fr> <fl>     wheel position, rad (since RoboClaw power-up)
// Prints "nan" for a RoboClaw that did not answer.
void read_and_send_wheel_feedback() {
  const float dirs[4] = {DIR_BACK_LEFT, DIR_BACK_RIGHT, DIR_FRONT_RIGHT, DIR_FRONT_LEFT};
  float speed[4];
  float position[4];

  // RoboClaw 1 (back): M1 = back left, M2 = back right.
  // RoboClaw 2 (front): M1 = front right, M2 = front left.
  uint8_t status = 0;
  bool valid = false;
  uint32_t raw = roboclaw1.ReadSpeedM1(RC_ADDRESS, &status, &valid);
  speed[0] = valid ? signed_qpps(raw, status) : NAN;
  raw = roboclaw1.ReadSpeedM2(RC_ADDRESS, &status, &valid);
  speed[1] = valid ? signed_qpps(raw, status) : NAN;
  raw = roboclaw2.ReadSpeedM1(RC_ADDRESS, &status, &valid);
  speed[2] = valid ? signed_qpps(raw, status) : NAN;
  raw = roboclaw2.ReadSpeedM2(RC_ADDRESS, &status, &valid);
  speed[3] = valid ? signed_qpps(raw, status) : NAN;

  uint32_t enc_m1 = 0, enc_m2 = 0;
  const bool back_valid = roboclaw1.ReadEncoders(RC_ADDRESS, enc_m1, enc_m2);
  position[0] = back_valid ? (float)(int32_t)enc_m1 : NAN;
  position[1] = back_valid ? (float)(int32_t)enc_m2 : NAN;
  const bool front_valid = roboclaw2.ReadEncoders(RC_ADDRESS, enc_m1, enc_m2);
  position[2] = front_valid ? (float)(int32_t)enc_m1 : NAN;
  position[3] = front_valid ? (float)(int32_t)enc_m2 : NAN;

  Serial.print("SPEED ");
  for (int i = 0; i < 4; ++i) {
    Serial.print(speed[i] / WHEEL_RAD_S_TO_QPPS * dirs[i], 4);
    Serial.print(i < 3 ? ' ' : '\n');
  }
  Serial.print("ENC ");
  for (int i = 0; i < 4; ++i) {
    Serial.print(position[i] / WHEEL_RAD_S_TO_QPPS * dirs[i], 4);
    Serial.print(i < 3 ? ' ' : '\n');
  }
}


void read_and_send_currents() {
  float currents[NUM_CURRENT_CHANNELS];
  read_currents(currents);

  // Array/pin order is:
  //   [0] pin 20 = back left
  //   [1] pin 21 = back right
  //   [2] pin 22 = front right
  //   [3] pin 23 = front left
  Serial.print("CURR ");
  Serial.print(currents[0], 3);
  Serial.print(' ');
  Serial.print(currents[1], 3);
  Serial.print(' ');
  Serial.print(currents[2], 3);
  Serial.print(' ');
  Serial.println(currents[3], 3);
}


// -----------------------------------------------------------------------------
// ARDUINO ENTRY POINTS
// -----------------------------------------------------------------------------

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (int ch = 0; ch < NUM_CURRENT_CHANNELS; ++ch) {
    pinMode(CURRENT_SENSOR_PINS[ch], INPUT);
  }

  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    pinMode(ARM_CURRENT_PINS[ch], INPUT);
    pinMode(ARM_VOLTAGE_PINS[ch], INPUT);
  }

  for (int ch = 0; ch < NUM_BATTERY_CHANNELS; ++ch) {
    pinMode(BATTERY_VOLTAGE_PINS[ch], INPUT);
  }

  // Match the 10-bit ADC assumption inherited from the current-sensor test code.
  analogReadResolution(10);

  // Teensy USB CDC serial to Raspberry Pi.
  // USB serial ignores the physical baud rate on Teensy, but this matches the
  // Pi-side configuration and documents the intended interface.
  Serial.begin(115200);

  Serial1.begin(38400); // Back Axle RoboClaw
  Serial3.begin(38400); // Front Axle RoboClaw
  roboclaw1.begin(38400);
  roboclaw2.begin(38400);

  stop_all_motors();

  // Saved calibrations (from CAL_CURRENT / CAL_ARM_CURRENT) override the
  // compiled defaults.
  if (load_zero_calibration(EEPROM_ADDR_DRIVE_CAL, current_zero_offsets)) {
    for (int ch = 0; ch < NUM_CURRENT_CHANNELS; ++ch) {
      current_smoothed_pin_voltages[ch] = current_zero_offsets[ch] / CURRENT_DIVIDER_RATIO;
    }
    print_zeros("# CAL drive zeros loaded from EEPROM: ", current_zero_offsets);
  } else {
    print_zeros("# CAL drive zeros: compiled defaults (none saved): ", current_zero_offsets);
  }

  if (load_zero_calibration(EEPROM_ADDR_ARM_CAL, arm_current_zero_v)) {
    print_zeros("# CAL arm zeros loaded from EEPROM: ", arm_current_zero_v);
  } else {
    print_zeros("# CAL arm zeros: compiled defaults (none saved): ", arm_current_zero_v);
  }

  if (AUTO_CALIBRATE_CURRENT_AT_BOOT) {
    calibrate_current_sensors();
  }

  if (AUTO_ZERO_ARM_DRUMS_AT_BOOT) {
    auto_zero_arm_drum_currents();
  }
}


void loop() {
  process_usb_serial();

  if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = millis();
    read_and_send_voltages();
    read_and_send_currents();
    read_and_send_roboclaw_currents();
    read_and_send_wheel_feedback();
    read_and_send_arm_sensors();
    read_and_send_battery_voltages();
  }

  if (have_command && (millis() - last_command_time > COMMAND_TIMEOUT_MS)) {
    stop_all_motors();
    have_command = false;
    digitalWrite(LED_PIN, LOW);
  }
}
