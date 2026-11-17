// SG90 on GP4, driven at 50 Hz by the Servo library.
//
// The motor neuron rate is mapped to a behaviour: rest/scan -> alert ->
// startle reflex.  The angle is slew-rate limited so the plastic gears of a
// cheap SG90 never see a step change, and the range is clamped to 0..180.
#ifndef FLY_BRAIN_MOTOR_H
#define FLY_BRAIN_MOTOR_H

#include <Arduino.h>
#include <Servo.h>

#define SERVO_PIN 4
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180
#define SERVO_REST_ANGLE 90
#define SERVO_ALERT_ANGLE 55
#define SERVO_ESCAPE_ANGLE 20

#define SERVO_SCAN_MIN 70
#define SERVO_SCAN_MAX 110
#define SERVO_SCAN_STEP 1

// Degrees per 20 ms servo update: 3 -> 150 deg/s, 8 -> 400 deg/s.
#define SERVO_SLEW_NORMAL 3
#define SERVO_SLEW_FAST 8

// Motor population rate thresholds (Hz).
#define MOTOR_ALERT_RATE_HZ 40
#define MOTOR_STARTLE_RATE_HZ 120

enum FlyBehaviour {
  FLY_IDLE = 0,
  FLY_SCAN = 1,
  FLY_ALERT = 2,
  FLY_STARTLE = 3,
  FLY_SAFE = 4
};

class ServoMotor {
 public:
  void begin() {
    _servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    _angle = SERVO_REST_ANGLE;
    _target = SERVO_REST_ANGLE;
    _scanAngle = SERVO_SCAN_MIN;
    _scanDir = 1;
    _behaviour = FLY_IDLE;
    _servo.write(_angle);
  }

  // Called at 50 Hz.  rateHz is the motor population firing rate, valid tells
  // whether the distance sensor produced a usable reading this cycle.
  void update(uint16_t rateHz, bool valid) {
    int16_t target;
    if (!valid) {
      target = SERVO_REST_ANGLE;
      _behaviour = FLY_SAFE;
    } else if (rateHz >= MOTOR_STARTLE_RATE_HZ) {
      target = SERVO_ESCAPE_ANGLE;
      _behaviour = FLY_STARTLE;
    } else if (rateHz >= MOTOR_ALERT_RATE_HZ) {
      target = SERVO_ALERT_ANGLE;
      _behaviour = FLY_ALERT;
    } else {
      target = scanAngle();
      _behaviour = FLY_SCAN;
    }
    _target = target;
    slew();
    _servo.write(_angle);
  }

  uint8_t angle() const { return (uint8_t)_angle; }
  uint8_t targetAngle() const { return (uint8_t)_target; }
  FlyBehaviour behaviour() const { return _behaviour; }

  const char *behaviourName() const {
    switch (_behaviour) {
      case FLY_SCAN: return "SCAN";
      case FLY_ALERT: return "ALERT";
      case FLY_STARTLE: return "STARTLE";
      case FLY_SAFE: return "SAFE";
      default: return "IDLE";
    }
  }

 private:
  int16_t scanAngle() {
    _scanAngle += _scanDir * SERVO_SCAN_STEP;
    if (_scanAngle >= SERVO_SCAN_MAX) { _scanAngle = SERVO_SCAN_MAX; _scanDir = -1; }
    if (_scanAngle <= SERVO_SCAN_MIN) { _scanAngle = SERVO_SCAN_MIN; _scanDir = 1; }
    return _scanAngle;
  }

  void slew() {
    int16_t step = (_behaviour == FLY_STARTLE) ? SERVO_SLEW_FAST : SERVO_SLEW_NORMAL;
    if (_angle < _target) {
      _angle = (_angle + step > _target) ? _target : (int16_t)(_angle + step);
    } else if (_angle > _target) {
      _angle = (_angle - step < _target) ? _target : (int16_t)(_angle - step);
    }
    if (_angle < SERVO_ANGLE_MIN) _angle = SERVO_ANGLE_MIN;
    if (_angle > SERVO_ANGLE_MAX) _angle = SERVO_ANGLE_MAX;
  }

  Servo _servo;
  int16_t _angle;
  int16_t _target;
  int16_t _scanAngle;
  int8_t _scanDir;
  FlyBehaviour _behaviour;
};

#endif  // FLY_BRAIN_MOTOR_H