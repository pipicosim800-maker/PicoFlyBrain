// HC-SR04 proximity sensor: Trig = GP2, Echo = GP3 through a 5V->3.3V divider.
//
// The driver keeps a median-of-5 filter, converts distance into a sensory
// current (closer -> stronger) and estimates the approach rate from the
// distance derivative.  A missing echo never blocks the caller for longer
// than SONAR_ECHO_TIMEOUT_US and always degrades to "no stimulus".
#ifndef FLY_BRAIN_SENSORS_H
#define FLY_BRAIN_SENSORS_H

#include <Arduino.h>

#define SONAR_PIN_TRIG 2
#define SONAR_PIN_ECHO 3

// 10 ms of echo == ~1.7 m.  Anything further away is treated as "no object".
#define SONAR_ECHO_TIMEOUT_US 10000
#define SONAR_MEDIAN_N 5

#define SONAR_MIN_MM 40
#define SONAR_MAX_MM 1700

// Distance -> sensory current mapping.
#define SONAR_DRIVE_MAX 120
#define SONAR_DRIVE_NEAR_MM 120
#define SONAR_DRIVE_FAR_MM 900

// Approach-rate boost: a hand closing in fast is scarier than a static wall.
#define SONAR_APPROACH_FULL_MM_S 900
#define SONAR_APPROACH_BOOST 40

class ProximitySensor {
 public:
  void begin() {
    pinMode(SONAR_PIN_TRIG, OUTPUT);
    digitalWrite(SONAR_PIN_TRIG, LOW);
    pinMode(SONAR_PIN_ECHO, INPUT);
    _count = 0;
    _idx = 0;
    _distanceMm = -1;
    _prevMm = -1;
    _prevUs = 0;
    _rateMmS = 0;
    _valid = false;
    _timedOut = false;
  }

  // Fire one ping and update the filters.  Returns true when a valid echo was
  // received.  nowUs is the caller's micros() timestamp.
  bool sample(uint32_t nowUs) {
    digitalWrite(SONAR_PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_PIN_TRIG, LOW);

    uint32_t echoUs = pulseIn(SONAR_PIN_ECHO, HIGH, SONAR_ECHO_TIMEOUT_US);
    if (echoUs == 0) {
      // No echo: drop the history so the next real reading is not diluted by
      // stale samples, and report "nothing in range".
      _count = 0;
      _idx = 0;
      _valid = false;
      _timedOut = true;
      _rateMmS = 0;
      _prevMm = -1;
      _prevUs = nowUs;
      return false;
    }

    uint32_t mm = (echoUs * 343UL) / 2000UL;
    if (mm < SONAR_MIN_MM) mm = SONAR_MIN_MM;
    if (mm > SONAR_MAX_MM) mm = SONAR_MAX_MM;

    _ring[_idx] = (uint16_t)mm;
    _idx = (uint8_t)((_idx + 1) % SONAR_MEDIAN_N);
    if (_count < SONAR_MEDIAN_N) ++_count;

    int16_t filtered = (int16_t)median();
    if (_prevMm >= 0 && nowUs > _prevUs) {
      uint32_t dtUs = nowUs - _prevUs;
      int32_t delta = (int32_t)_prevMm - (int32_t)filtered;  // >0 == approaching
      _rateMmS = (int16_t)((delta * 1000000L) / (int32_t)dtUs);
    }
    _prevMm = filtered;
    _prevUs = nowUs;
    _distanceMm = filtered;
    _valid = true;
    _timedOut = false;
    return true;
  }

  // Bench mode: pretend the rangefinder measured mm.  The sample goes through
  // the same median filter and approach-rate estimate as a real echo, so the
  // behaviour under test is identical to the behaviour with a hand in front.
  void injectDistance(int16_t mm) {
    if (mm < SONAR_MIN_MM) mm = SONAR_MIN_MM;
    if (mm > SONAR_MAX_MM) mm = SONAR_MAX_MM;
    uint32_t nowUs = micros();
    _ring[_idx] = (uint16_t)mm;
    _idx = (uint8_t)((_idx + 1) % SONAR_MEDIAN_N);
    if (_count < SONAR_MEDIAN_N) ++_count;
    int16_t filtered = (int16_t)median();
    if (_prevMm >= 0 && nowUs > _prevUs) {
      uint32_t dtUs = nowUs - _prevUs;
      int32_t delta = (int32_t)_prevMm - (int32_t)filtered;  // >0 == approaching
      _rateMmS = (int16_t)((delta * 1000000L) / (int32_t)dtUs);
    }
    _prevMm = filtered;
    _prevUs = nowUs;
    _distanceMm = filtered;
    _valid = true;
    _timedOut = false;
  }

  bool valid() const { return _valid; }
  bool timedOut() const { return _timedOut; }
  int16_t distanceMm() const { return _distanceMm; }
  int16_t approachRateMmS() const { return _rateMmS; }

  // Sensory current for the neural engine, 0..SONAR_DRIVE_MAX.
  int16_t drive() const {
    if (!_valid || _distanceMm < 0) return 0;
    int32_t d = _distanceMm;
    int32_t base;
    if (d <= SONAR_DRIVE_NEAR_MM) {
      base = SONAR_DRIVE_MAX;
    } else if (d >= SONAR_DRIVE_FAR_MM) {
      base = 0;
    } else {
      base = ((SONAR_DRIVE_FAR_MM - d) * SONAR_DRIVE_MAX) /
             (SONAR_DRIVE_FAR_MM - SONAR_DRIVE_NEAR_MM);
    }
    if (_rateMmS > 0) {
      int32_t boost = ((int32_t)_rateMmS * SONAR_APPROACH_BOOST) / SONAR_APPROACH_FULL_MM_S;
      if (boost > SONAR_APPROACH_BOOST) boost = SONAR_APPROACH_BOOST;
      base += boost;
    }
    if (base > SONAR_DRIVE_MAX) base = SONAR_DRIVE_MAX;
    if (base < 0) base = 0;
    return (int16_t)base;
  }

 private:
  uint16_t median() {
    uint16_t tmp[SONAR_MEDIAN_N];
    uint8_t n = _count;
    for (uint8_t i = 0; i < n; ++i) tmp[i] = _ring[i];
    for (uint8_t i = 1; i < n; ++i) {  // insertion sort, n <= 5
      uint16_t key = tmp[i];
      int8_t j = (int8_t)i - 1;
      while (j >= 0 && tmp[j] > key) {
        tmp[j + 1] = tmp[j];
        --j;
      }
      tmp[j + 1] = key;
    }
    return tmp[n / 2];
  }

  uint16_t _ring[SONAR_MEDIAN_N];
  uint8_t _count;
  uint8_t _idx;
  int16_t _distanceMm;
  int16_t _prevMm;
  uint32_t _prevUs;
  int16_t _rateMmS;
  bool _valid;
  bool _timedOut;
};

#endif  // FLY_BRAIN_SENSORS_H