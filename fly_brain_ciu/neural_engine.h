// Deterministic spiking LIF engine for the FlyWire sub-network.
//
// One step() call advances the network by exactly 1 ms of simulated time.
// The hot loop is integer-only: no float, no malloc, no virtual calls.
// Synapse data lives in flash (const arrays from connectome_data.h), the
// mutable state is a handful of small static arrays.
#ifndef FLY_BRAIN_NEURAL_ENGINE_H
#define FLY_BRAIN_NEURAL_ENGINE_H

#include <stdint.h>
#include <string.h>

#include "connectome_data.h"

// Sensory drive scale: 0 = nothing in range, FLY_INPUT_MAX = object right at
// the antennae.  Same units as the int8 synapse weights.
#define FLY_INPUT_MAX 120

// Motor rate is measured over this sliding window (ms).
#define FLY_RATE_WINDOW_MS 100

// Role codes used by FLY_NEURON_ROLE.
#define FLY_ROLE_SENSORY 0
#define FLY_ROLE_INTERNEURON 1
#define FLY_ROLE_MOTOR 2

class NeuralEngine {
 public:
  void begin() { reset(); }

  void reset() {
    memset(_v, 0, sizeof(_v));
    memset(_refrac, 0, sizeof(_refrac));
    memset(_hist, 0, sizeof(_hist));
    for (uint16_t i = 0; i < FLY_NEURON_COUNT; ++i) {
      _hab[i] = LIF_HAB_MAX;
    }
    _spikeCount = 0;
    _totalSpikes = 0;
    _motorSpikes = 0;
    _histSum = 0;
    _histIdx = 0;
    _rateHz = 0;
    _tick = 0;
  }

  // Advance the network by 1 ms.  drive is the external current injected into
  // every sensory neuron (0..FLY_INPUT_MAX).
  void step(int16_t drive) {
    _spikeCount = 0;

    // 1. leak, refractory countdown, habituation recovery
    for (uint16_t i = 0; i < FLY_NEURON_COUNT; ++i) {
      int32_t v = _v[i];
      v -= (v * LIF_LEAK_NUM) >> LIF_LEAK_SHIFT;
      if (v > LIF_V_CAP) v = LIF_V_CAP;
      if (v < -LIF_V_CAP) v = -LIF_V_CAP;
      _v[i] = (int16_t)v;
      if (_refrac[i] > 0) --_refrac[i];
      if ((_tick & LIF_HAB_RECOVER_MASK) == 0 && _hab[i] < LIF_HAB_MAX) ++_hab[i];
    }

    // 2. external sensory current, attenuated by habituation
    if (drive > 0) {
      int32_t base = ((int32_t)drive * LIF_INPUT_GAIN) >> LIF_INPUT_SHIFT;
      for (uint16_t k = 0; k < FLY_SENSORY_COUNT; ++k) {
        uint16_t i = FLY_SENSORY_INDICES[k];
        int32_t d = (base * (int32_t)_hab[i]) >> 8;
        int32_t v = _v[i] + d;
        if (v > LIF_V_CAP) v = LIF_V_CAP;
        _v[i] = (int16_t)v;
      }
    }

    // 3. threshold crossing -> spike
    for (uint16_t i = 0; i < FLY_NEURON_COUNT; ++i) {
      if (_refrac[i] == 0 && _v[i] >= FLY_NEURON_THRESHOLD[i]) {
        _v[i] = 0;
        _refrac[i] = LIF_REFRACT_MS;
        if (_hab[i] > LIF_HAB_MIN) {
          _hab[i] = (_hab[i] > (uint16_t)(LIF_HAB_MIN + LIF_HAB_STEP))
                        ? (uint16_t)(_hab[i] - LIF_HAB_STEP)
                        : (uint16_t)LIF_HAB_MIN;
        }
        _spikes[_spikeCount++] = i;
      }
    }

    // 4. sparse propagation over the CSR matrix.  Spikes are delivered after
    //    the whole threshold sweep, so a tick cannot cascade into itself.
    for (uint16_t k = 0; k < _spikeCount; ++k) {
      uint16_t pre = _spikes[k];
      uint32_t end = FLY_ROW_START[pre + 1];
      for (uint32_t e = FLY_ROW_START[pre]; e < end; ++e) {
        uint16_t post = FLY_COL_IDX[e];
        int32_t d = ((int32_t)FLY_WEIGHT[e] * LIF_SYNAPSE_GAIN) >> LIF_SYNAPSE_SHIFT;
        int32_t v = _v[post] + d;
        if (v > LIF_V_CAP) v = LIF_V_CAP;
        if (v < -LIF_V_CAP) v = -LIF_V_CAP;
        _v[post] = (int16_t)v;
      }
    }

    // 5. bookkeeping: total spikes and the motor population rate
    _totalSpikes += _spikeCount;
    uint8_t motorNow = 0;
    for (uint16_t k = 0; k < _spikeCount; ++k) {
      if (FLY_NEURON_ROLE[_spikes[k]] == FLY_ROLE_MOTOR) {
        ++_motorSpikes;
        if (motorNow < 255) ++motorNow;
      }
    }
    _histSum -= _hist[_histIdx];
    _hist[_histIdx] = motorNow;
    _histSum += motorNow;
    _histIdx = (uint8_t)((_histIdx + 1) % FLY_RATE_WINDOW_MS);
    _rateHz = (uint16_t)(_histSum * (1000 / FLY_RATE_WINDOW_MS));

    ++_tick;
  }

  uint16_t spikesThisTick() const { return _spikeCount; }
  uint32_t totalSpikes() const { return _totalSpikes; }
  uint32_t motorSpikes() const { return _motorSpikes; }
  uint16_t motorRateHz() const { return _rateHz; }
  uint32_t tick() const { return _tick; }

  // Mean sensory habituation, 0..LIF_HAB_MAX (256 = fully sensitive).
  uint16_t sensoryHabituation() const {
    if (FLY_SENSORY_COUNT == 0) return LIF_HAB_MAX;
    uint32_t sum = 0;
    for (uint16_t k = 0; k < FLY_SENSORY_COUNT; ++k) sum += _hab[FLY_SENSORY_INDICES[k]];
    return (uint16_t)(sum / FLY_SENSORY_COUNT);
  }

  // Membrane potential of one neuron, for the bench diagnostics.
  int16_t membrane(uint16_t i) const { return (i < FLY_NEURON_COUNT) ? _v[i] : 0; }

 private:
  int16_t _v[FLY_NEURON_COUNT];
  uint16_t _hab[FLY_NEURON_COUNT];
  uint8_t _refrac[FLY_NEURON_COUNT];
  uint16_t _spikes[FLY_NEURON_COUNT];
  uint8_t _hist[FLY_RATE_WINDOW_MS];
  uint16_t _histSum;
  uint8_t _histIdx;
  uint16_t _spikeCount;
  uint32_t _totalSpikes;
  uint32_t _motorSpikes;
  uint16_t _rateHz;
  uint32_t _tick;
};

#endif  // FLY_BRAIN_NEURAL_ENGINE_H