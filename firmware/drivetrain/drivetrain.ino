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
//   STOP                      (stop all motors immediately)
//
// Teensy -> Raspberry Pi:
//   STOPPED
//   VOLT <back_left> <back_right> <front_right> <front_left>
//   CURR <back_left> <back_right> <front_right> <front_left>
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
}


void loop() {
  process_usb_serial();

  if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = millis();
    read_and_send_voltages();
    read_and_send_currents();
  }

  if (have_command && (millis() - last_command_time > COMMAND_TIMEOUT_MS)) {
    stop_all_motors();
    have_command = false;
    digitalWrite(LED_PIN, LOW);
  }
}
