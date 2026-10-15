#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <RoboClaw.h>

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
//   CAL_CURRENT
//   CAL_ARM_CURRENT           (zero arm current sensors; loads disconnected)
//   STOP                      (stop all motors immediately)
//
// Teensy -> Raspberry Pi:
//   STOPPED
//   VOLT <back_left> <back_right> <front_right> <front_left>
//   CURR <back_left> <back_right> <front_right> <front_left>
//   ARM_CURR <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>   (A)
//   ARM_VOLT <rear_drum> <rear_shoulder> <front_drum> <front_shoulder>   (V)
//   CAL_ARM_CURRENT START / CAL_ARM_CURRENT OK <z0> <z1> <z2> <z3>
//   CAL_CURRENT START
//   CAL_CURRENT OK
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

// ACS712 5A sensor + external 10k/20k divider used by the EE test sketch.
constexpr float ADC_VREF = 3.3f;
constexpr float ADC_MAX = 1023.0f;             // 10-bit ADC
constexpr float CURRENT_DIVIDER_RATIO = 1.5f; // reconstruct ACS712 output voltage
constexpr float CURRENT_SENSITIVITY = 0.185f; // V/A for ACS712 5A model

constexpr int CURRENT_OVERSAMPLE_COUNT = 128;
constexpr float CURRENT_ALPHA = 0.08f;
constexpr int CURRENT_CALIBRATION_SAMPLES = 500;

// For now we calibrate once during Teensy startup while motors are stopped.
// Later the ROS startup/calibration primitive can explicitly issue CAL_CURRENT.
// Set false later if startup calibration becomes fully owned by ROS.
constexpr bool AUTO_CALIBRATE_CURRENT_AT_BOOT = true;

float current_zero_offsets[NUM_CURRENT_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};
float current_smoothed_pin_voltages[NUM_CURRENT_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};

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
//     5 V -> 3.3 V divider. Bench-measured total ratio = 11.8:1
//     (27.5 V -> 2.33 V), so a full 7S pack (29.4 V) -> ~2.5 V at the pin.
//     Full scale at 3.3 V = ~38.9 V.
// -----------------------------------------------------------------------------
constexpr int NUM_ARM_CHANNELS = 4;
// Wiring: rear arm on 16-19, front arm on 38-41.
//   16 rear drum I     17 rear drum V     18 rear shoulder I   19 rear shoulder V
//   38 front shldr V   39 front shldr I   40 front drum V      41 front drum I
const int ARM_CURRENT_PINS[NUM_ARM_CHANNELS] = {16, 18, 41, 39};
const int ARM_VOLTAGE_PINS[NUM_ARM_CHANNELS] = {17, 19, 40, 38};

constexpr float ARM_CURRENT_DIVIDER_RATIO = 1.5f;   // reconstruct ACS712 output voltage
constexpr float ARM_CURRENT_SENSITIVITY = 0.066f;   // V/A for ACS712 30A model

// ACS712 output at 0 A, in sensor-side volts (nominal = 5 V supply / 2).
//   Drums [0],[2]: auto-zeroed at boot (drum drivers are disabled until the
//     arm Teensy receives a DRUM command, so their current is ~0).
//   Shoulders [1],[3]: always energized (holding current), so never at 0 A
//     in operation. Zero ONCE by hand: disconnect the shoulder driver supply
//     leads downstream of the sensors, send CAL_ARM_CURRENT, paste the printed
//     values here, reflash. Redo only if the sensors or 5 V supply change.
// Measured 2026-10-15 with CAL_ARM_CURRENT (sensors powered, no load current).
float arm_current_zero_v[NUM_ARM_CHANNELS] = {2.4487f, 2.6483f, 2.4474f, 2.4952f};

constexpr bool AUTO_ZERO_ARM_DRUMS_AT_BOOT = true;
const bool ARM_CHANNEL_IS_DRUM[NUM_ARM_CHANNELS] = {true, false, true, false};

// Measured volts per pin volt. 11.8 from the bench test (Uno ADC); trim each
// channel on the Teensy against a multimeter: new = old * (meter / reading).
const float ARM_VOLTAGE_DIVIDER_RATIO[NUM_ARM_CHANNELS] = {11.8f, 11.8f, 11.8f, 11.8f};

constexpr int ARM_OVERSAMPLE_COUNT = 64;
// Light smoothing between 10 Hz samples (1.0 = none). Kept low so power
// transients stay visible for characterization.
constexpr float ARM_ALPHA = 0.5f;

float arm_current_smoothed[NUM_ARM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};
float arm_voltage_smoothed[NUM_ARM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};
bool arm_sensors_initialized = false;

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


void calibrate_current_sensors() {
  // Calibration must happen at zero motor current.
  stop_all_motors();
  have_command = false;
  digitalWrite(LED_PIN, LOW);

  Serial.println("CAL_CURRENT START");

  // Let motor commands settle before measuring the zero-current baselines.
  delay(100);

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

    current_zero_offsets[ch] =
      pin_voltage * CURRENT_DIVIDER_RATIO;

    // Start the EMA at the measured zero-current pin voltage.
    current_smoothed_pin_voltages[ch] = pin_voltage;
  }

  // If the Pi was already streaming DRIVE commands during the blocking
  // calibration window, discard those stale commands. Fresh commands arriving
  // after calibration will be processed normally.
  flush_usb_rx();

  Serial.println("CAL_CURRENT OK");
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
      CURRENT_SENSITIVITY;
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
// Prints the values to paste into arm_current_zero_v.
void calibrate_arm_current_sensors() {
  stop_all_motors();
  have_command = false;
  digitalWrite(LED_PIN, LOW);

  Serial.println("CAL_ARM_CURRENT START");

  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    arm_current_zero_v[ch] = measure_arm_current_zero(ch);
  }

  arm_sensors_initialized = false;  // Restart smoothing from fresh readings
  flush_usb_rx();

  Serial.print("CAL_ARM_CURRENT OK ");
  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    Serial.print(arm_current_zero_v[ch], 4);
    Serial.print(ch < NUM_ARM_CHANNELS - 1 ? ' ' : '\n');
  }
}


void read_and_send_arm_sensors() {
  float currents[NUM_ARM_CHANNELS];
  float voltages[NUM_ARM_CHANNELS];

  for (int ch = 0; ch < NUM_ARM_CHANNELS; ++ch) {
    const float sensor_v =
      read_pin_voltage(ARM_CURRENT_PINS[ch], ARM_OVERSAMPLE_COUNT) *
      ARM_CURRENT_DIVIDER_RATIO;
    currents[ch] = (sensor_v - arm_current_zero_v[ch]) / ARM_CURRENT_SENSITIVITY;

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
    read_and_send_arm_sensors();
  }

  if (have_command && (millis() - last_command_time > COMMAND_TIMEOUT_MS)) {
    stop_all_motors();
    have_command = false;
    digitalWrite(LED_PIN, LOW);
  }
}
