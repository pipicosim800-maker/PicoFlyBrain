// Fly Brain -- FlyWire connectome as a spiking network on a Raspberry Pi Pico.
//
// Hardware (fixed by the owner):
//   HC-SR04 Trig  -> GP2
//   HC-SR04 Echo  -> GP3 through a 1k/2k divider (5V -> 3.3V, mandatory)
//   SG90 signal   -> GP4, 5V supply, common ground
//
// Scheduler: fully non-blocking, three independent rates
//   sensor   20 Hz  (every 50 ms)
//   network   1 kHz (every 1 ms)
//   servo    50 Hz  (every 20 ms)
//
// Serial @115200 telemetry + bench commands (see README.md).

#include <Arduino.h>

#include "connectome_data.h"
#include "neural_engine.h"
#include "sensors.h"
#include "motor.h"

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
#define SENSOR_PERIOD_US 50000UL   // 20 Hz
#define SERVO_PERIOD_US 20000UL    // 50 Hz
#define TELEMETRY_PERIOD_MS 250    // 4 lines per second
#define TICK_BUDGET_US 1000UL      // one network step must fit in 1 ms

// Bench stimulus: a manual distance is held for this long, then released.
#define MANUAL_HOLD_MS 4000

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static NeuralEngine g_brain;
static ProximitySensor g_sensor;
static ServoMotor g_servo;

static uint32_t g_nextSensorUs = 0;
static uint32_t g_nextServoUs = 0;
static uint32_t g_nextTickUs = 0;
static uint32_t g_nextTelemetryMs = 0;

static int16_t g_manualDistanceMm = -1;   // >=0 while a bench distance is active
static uint32_t g_manualUntilMs = 0;
static bool g_feedMode = false;           // continuous stimulus at g_feedDistanceMm
static int16_t g_feedDistanceMm = 200;

static uint16_t g_lastRateHz = 0;
static int16_t g_lastDrive = 0;
static uint32_t g_lastTickUs = 0;
static uint32_t g_maxTickUs = 0;
static uint32_t g_ticks = 0;
static uint32_t g_loopCount = 0;

// ---------------------------------------------------------------------------
// Serial command handling
// ---------------------------------------------------------------------------
static void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  d<mm>   manual distance in mm (e.g. d250), d-1 clears"));
  Serial.println(F("  f<mm>   feed mode: repeat stimulus at <mm> mm, f0 stops"));
  Serial.println(F("  s       status line"));
  Serial.println(F("  r       reset network state"));
  Serial.println(F("  h       this help"));
}

static void printStatus() {
  Serial.print(F("STATUS dist="));
  if (g_sensor.valid()) {
    Serial.print(g_sensor.distanceMm());
  } else {
    Serial.print(F("none"));
  }
  Serial.print(F(" drive="));
  Serial.print(g_lastDrive);
  Serial.print(F(" rate="));
  Serial.print(g_lastRateHz);
  Serial.print(F(" angle="));
  Serial.print(g_servo.angle());
  Serial.print(F(" state="));
  Serial.print(g_servo.behaviourName());
  Serial.print(F(" hab="));
  Serial.print(g_brain.sensoryHabituation());
  Serial.print(F(" spikes="));
  Serial.print(g_brain.totalSpikes());
  Serial.print(F(" motor="));
  Serial.print(g_brain.motorSpikes());
  Serial.print(F(" ticks="));
  Serial.print(g_ticks);
  Serial.print(F(" maxTickUs="));
  Serial.println(g_maxTickUs);
}

static void handleCommand(char *line) {
  // Trim trailing CR/spaces.
  int len = strlen(line);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) line[--len] = '\0';
  if (len == 0) return;

  char cmd = line[0];
  long arg = 0;
  bool hasArg = false;
  if (len > 1) {
    hasArg = true;
    arg = atol(line + 1);
  }

  switch (cmd) {
    case 'd':
    case 'D':
      if (!hasArg) { printHelp(); break; }
      if (arg < 0) {
        g_manualDistanceMm = -1;
        Serial.println(F("OK manual distance cleared"));
      } else {
        if (arg < SONAR_MIN_MM) arg = SONAR_MIN_MM;
        if (arg > SONAR_MAX_MM) arg = SONAR_MAX_MM;
        g_manualDistanceMm = (int16_t)arg;
        g_manualUntilMs = millis() + MANUAL_HOLD_MS;
        Serial.print(F("OK manual distance "));
        Serial.print(arg);
        Serial.println(F(" mm"));
      }
      break;
    case 'f':
    case 'F':
      if (!hasArg) { printHelp(); break; }
      if (arg <= 0) {
        g_feedMode = false;
        Serial.println(F("OK feed mode off"));
      } else {
        if (arg < SONAR_MIN_MM) arg = SONAR_MIN_MM;
        if (arg > SONAR_MAX_MM) arg = SONAR_MAX_MM;
        g_feedDistanceMm = (int16_t)arg;
        g_feedMode = true;
        Serial.print(F("OK feed mode "));
        Serial.print(arg);
        Serial.println(F(" mm"));
      }
      break;
    case 's':
    case 'S':
      printStatus();
      break;
    case 'r':
    case 'R':
      g_brain.reset();
      g_maxTickUs = 0;
      g_ticks = 0;
      Serial.println(F("OK network reset"));
      break;
    case 'h':
    case 'H':
    case '?':
      printHelp();
      break;
    default:
      Serial.print(F("ERR unknown command: "));
      Serial.println(line);
      printHelp();
      break;
  }
}

static void pollSerial() {
  static char buf[32];
  static uint8_t pos = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos > 0) {
        buf[pos] = '\0';
        handleCommand(buf);
        pos = 0;
      }
    } else if (pos < sizeof(buf) - 1) {
      buf[pos++] = c;
    } else {
      pos = 0;  // overflow: drop the line
    }
  }
}

// ---------------------------------------------------------------------------
// Sensor scheduling
// ---------------------------------------------------------------------------
static void serviceSensor(uint32_t nowUs) {
  if ((int32_t)(nowUs - g_nextSensorUs) < 0) return;
  g_nextSensorUs = nowUs + SENSOR_PERIOD_US;

  // A bench distance overrides the real sensor so the behaviour can be tested
  // without a rangefinder.  Feed mode keeps it alive indefinitely.
  if (g_feedMode) {
    g_manualDistanceMm = g_feedDistanceMm;
    g_manualUntilMs = millis() + MANUAL_HOLD_MS;
  }
  if (g_manualDistanceMm >= 0 && (int32_t)(millis() - g_manualUntilMs) >= 0) {
    g_manualDistanceMm = -1;
  }

  if (g_manualDistanceMm >= 0) {
    g_sensor.injectDistance(g_manualDistanceMm);
    return;
  }
  g_sensor.sample(nowUs);
}

// ---------------------------------------------------------------------------
// Network scheduling: one 1 ms step per call
// ---------------------------------------------------------------------------
static void serviceBrain(uint32_t nowUs) {
  if ((int32_t)(nowUs - g_nextTickUs) < 0) return;
  g_nextTickUs = nowUs + 1000UL;

  int16_t drive = g_sensor.drive();
  g_lastDrive = drive;

  uint32_t t0 = micros();
  g_brain.step(drive);
  uint32_t dt = micros() - t0;
  g_lastTickUs = dt;
  if (dt > g_maxTickUs) g_maxTickUs = dt;
  ++g_ticks;

  g_lastRateHz = g_brain.motorRateHz();
}

// ---------------------------------------------------------------------------
// Servo scheduling: 50 Hz
// ---------------------------------------------------------------------------
static void serviceServo(uint32_t nowUs) {
  if ((int32_t)(nowUs - g_nextServoUs) < 0) return;
  g_nextServoUs = nowUs + SERVO_PERIOD_US;
  g_servo.update(g_lastRateHz, g_sensor.valid());
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
static void serviceTelemetry(uint32_t nowMs) {
  if ((int32_t)(nowMs - g_nextTelemetryMs) < 0) return;
  g_nextTelemetryMs = nowMs + TELEMETRY_PERIOD_MS;

  Serial.print(F("T dist="));
  if (g_sensor.valid()) {
    Serial.print(g_sensor.distanceMm());
  } else {
    Serial.print(F("-1"));
  }
  Serial.print(F(" rate="));
  Serial.print(g_lastRateHz);
  Serial.print(F(" drive="));
  Serial.print(g_lastDrive);
  Serial.print(F(" angle="));
  Serial.print(g_servo.angle());
  Serial.print(F(" state="));
  Serial.print(g_servo.behaviourName());
  Serial.print(F(" spikes="));
  Serial.print(g_brain.spikesThisTick());
  Serial.print(F(" total="));
  Serial.print(g_brain.totalSpikes());
  Serial.print(F(" motor="));
  Serial.print(g_brain.motorSpikes());
  Serial.print(F(" hab="));
  Serial.print(g_brain.sensoryHabituation());
  Serial.print(F(" tickUs="));
  Serial.println(g_lastTickUs);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // Give the host a moment to attach, but never block the firmware on it.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 1500) {
    delay(10);
  }

  g_sensor.begin();
  g_servo.begin();
  g_brain.begin();

  uint32_t now = micros();
  g_nextSensorUs = now;
  g_nextServoUs = now;
  g_nextTickUs = now;
  g_nextTelemetryMs = millis();

  Serial.println();
  Serial.println(F("FLY BRAIN ready -- FlyWire connectome on RP2040"));
  Serial.print(F("neurons="));
  Serial.print(FLY_NEURON_COUNT);
  Serial.print(F(" synapses="));
  Serial.print(FLY_SYNAPSE_COUNT);
  Serial.print(F(" sensory="));
  Serial.print(FLY_SENSORY_COUNT);
  Serial.print(F(" motor="));
  Serial.print(FLY_MOTOR_COUNT);
  Serial.print(F(" checksum=0x"));
  Serial.println(FLY_CHECKSUM, HEX);
  printHelp();
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  ++g_loopCount;

  pollSerial();
  serviceSensor(nowUs);
  serviceBrain(nowUs);
  serviceServo(nowUs);
  serviceTelemetry(nowMs);
}