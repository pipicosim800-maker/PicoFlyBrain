// fly_brain.h — симуляція ланцюга втечі мухи (FlyWire v783, 668 нейронів)
// моделлю Leaky Integrate-and-Fire з кроком 1 мс.
//
// Порт LIFSim із DesktopFly (windows/src/sim.js) у фіксовану кому Q14:
// RP2040 не має FPU, тож потенціали й ваги — цілі числа (1.0 == 16384).
// Код не залежить від Arduino — той самий файл компілюється на ПК
// (tools/host_test.cpp) для перевірки.
#pragma once
#include <stdint.h>
#include "connectome.h"

// На Pico скетч визначає FLY_FAST, щоб step() виконувався з SRAM, а не через XIP-кеш флешу.
#ifndef FLY_FAST
#define FLY_FAST
#endif

enum FlyRole : uint8_t {
  ROLE_OTHER = 0, ROLE_LOOM, ROLE_DNA, ROLE_MDN, ROLE_FWD, ROLE_GROOM,
  ROLE_ESCW, ROLE_GF, ROLE_SENSORY, ROLE_ASCEND
};

enum FlySide : uint8_t { SIDE_LEFT = 0, SIDE_RIGHT = 1, SIDE_ANY = 2 };

class FlyBrain {
public:
  static const int32_t ONE = 1 << FLY_Q;

  // --- Параметри моделі (як у sim.js) ---
  static const int32_t DECAY      = 15585;              // exp(-1/20) · 2^14: τ мембрани 20 мс
  static const int32_t THRESHOLD  = ONE;                // поріг спайку 1.0
  static const int32_t V_MIN      = -2 * ONE;           // нижня межа потенціалу
  static const int32_t V_MAX      = 8 * ONE;            // верхня межа (захист від переповнення)
  static const uint8_t REFRACT_MS = 2;                  // рефрактерний період
  static const uint8_t INH_DELAY  = 4;                  // затримка гальмівних (GABA/Glu) синапсів, мс
  static const uint8_t INH_SLOTS  = 5;
  static const uint32_t P_NOISE   = 9448928u;           // 0.0022 · 2^32 — ймовірність шумового поштовху
  static const int32_t NOISE_KICK = 6881;               // 0.42
  static const int32_t LOOM_GAIN  = 4915;               // 0.30
  static const int32_t PUFF_GAIN  = 1966;               // 0.12

  // --- Входи (Q14, 0..ONE) — виставляються ззовні ---
  int32_t loomL = 0, loomR = 0;   // «насування» об'єкта на ліве/праве око
  int32_t airPuff = 0;            // потік повітря -> сенсорні нейрони

  // --- Виходи ---
  float rateLoom = 0, rateDNaL = 0, rateDNaR = 0, rateMDN = 0;
  float rateFwd = 0, rateGroom = 0, rateEscW = 0, ratePop = 0;   // Гц на нейрон (EMA)
  uint32_t gfSpikes = 0;          // лічильник спайків гігантського волокна
  uint32_t totalSpikes = 0;
  uint32_t simMs = 0;
  int lastSpikeCount = 0;

  void begin(uint32_t seed) {
    rng_ = seed ? seed : 0x9E3779B9u;
    for (int i = 0; i < FLY_N; i++) { v_[i] = 0; refr_[i] = 0; }
    for (int s = 0; s < INH_SLOTS; s++)
      for (int i = 0; i < FLY_N; i++) inhQ_[s][i] = 0;
    qHead_ = 0;
    simMs = 0;
    burstUntil_ = 0;
    burstNext_ = 12000;
    stimUntil_ = 0;
    for (int r = 0; r < 10; r++) groupSize_[r][0] = groupSize_[r][1] = 0;
    for (int i = 0; i < FLY_N; i++) {
      groupSize_[fly_role[i]][fly_side[i]]++;
      role_[i] = fly_role[i];        // копії з флешу в SRAM: їх читає кожен крок
      side_[i] = fly_side[i];
      base_[i] = fly_baseline[i];
    }
    // Кожен рядок CSR: спершу збуджувальні синапси, потім гальмівні (стабільно).
    // Тоді в step() два тісні цикли без перевірки знаку. Порядок додавань цілих чисел
    // не впливає на результат, тож динаміка бітово та сама.
    for (int i = 0; i < FLY_N; i++) {
      const int b = fly_row_start[i], e = fly_row_start[i + 1];
      rowStart_[i] = b;
      int m = b;
      for (int k = b; k < e; k++) {
        if (fly_w[k] < 0) continue;
        const uint16_t c = fly_col[k];
        const int16_t w = fly_w[k];
        for (int t = k; t > m; t--) { fly_col[t] = fly_col[t - 1]; fly_w[t] = fly_w[t - 1]; }
        fly_col[m] = c;
        fly_w[m] = w;
        m++;
      }
      rowInh_[i] = m;
    }
    rowStart_[FLY_N] = fly_row_start[FLY_N];
  }

  // «Оптогенетична» стимуляція групи нейронів: strength (Q14) щомілісекунди протягом durationMs.
  void stimulate(FlyRole role, FlySide side, int32_t strength, uint32_t durationMs) {
    stimRole_ = role;
    stimSide_ = side;
    stimStrength_ = strength;
    stimUntil_ = simMs + durationMs;
  }

  int groupSize(FlyRole role) const { return groupSize_[role][0] + groupSize_[role][1]; }

  // Один крок симуляції = 1 мс біологічного часу.
  void FLY_FAST step() {
    simMs++;
    if (simMs >= burstNext_) {    // рідкісні сплески «збудження»
      burstUntil_ = simMs + 400;
      burstNext_ = simMs + 15000 + rand() % 25001;
    }
    const uint32_t p = simMs < burstUntil_ ? P_NOISE * 6 : P_NOISE;
    const int32_t dL = (loomL * LOOM_GAIN) >> FLY_Q;
    const int32_t dR = (loomR * LOOM_GAIN) >> FLY_Q;
    const int32_t dP = (airPuff * PUFF_GAIN) >> FLY_Q;
    const bool stim = simMs < stimUntil_;
    int32_t* q = inhQ_[qHead_];

    // 1–4 за один прохід (нейрони до поширення спайків незалежні, тож порядок той самий, що в sim.js):
    // витік + базовий струм + шум, сенсорні входи, затримане гальмування, поріг -> спайк
    int nSpiked = 0;
    for (int i = 0; i < FLY_N; i++) {
      int32_t vi = v_[i];
      if (vi > V_MAX) vi = V_MAX;
      vi = (vi * DECAY) >> FLY_Q;
      uint8_t rf = refr_[i];
      if (rf) {
        refr_[i] = --rf;
      } else {
        vi += base_[i];
        if (rand() < p) vi += NOISE_KICK;
      }
      const uint8_t r = role_[i];
      if (r == ROLE_LOOM) vi += side_[i] == SIDE_LEFT ? dL : dR;
      else if (r == ROLE_SENSORY) vi += dP;
      if (stim && r == stimRole_ && (stimSide_ == SIDE_ANY || stimSide_ == side_[i]))
        vi += stimStrength_;
      const int32_t qi = q[i];
      if (qi) {
        vi += qi;
        if (vi < V_MIN) vi = V_MIN;
        q[i] = 0;
      }
      if (!rf && vi >= THRESHOLD) {
        vi = 0;
        refr_[i] = REFRACT_MS;
        spiked_[nSpiked++] = (uint16_t)i;
      }
      v_[i] = vi;
    }
    totalSpikes += nSpiked;
    lastSpikeCount = nSpiked;

    // 5. Поширення спайків: збудження миттєве, гальмування — через INH_DELAY мс.
    // Верхня межа V_MAX застосовується на початку наступного кроку, до множення на DECAY:
    // тут додаються лише додатні ваги, тож min(v + Σw, V_MAX) == послідовним min(v + w, V_MAX).
    int32_t* inh = inhQ_[(qHead_ + INH_DELAY) % INH_SLOTS];
    for (int s = 0; s < nSpiked; s++) {
      const int i = spiked_[s];
      int k = rowStart_[i];
      const int mid = rowInh_[i], end = rowStart_[i + 1];
      for (; k < mid; k++) v_[fly_col[k]] += fly_w[k];
      for (; k < end; k++) inh[fly_col[k]] += fly_w[k];
    }
    qHead_ = (qHead_ + 1) % INH_SLOTS;

    // 6. Частоти груп (Гц на нейрон, експоненційне згладжування ~120 мс)
    int cLoom = 0, cDL = 0, cDR = 0, cM = 0, cF = 0, cG = 0, cW = 0;
    for (int s = 0; s < nSpiked; s++) {
      const int i = spiked_[s];
      switch (role_[i]) {
        case ROLE_LOOM: cLoom++; break;
        case ROLE_DNA: if (side_[i] == SIDE_LEFT) cDL++; else cDR++; break;
        case ROLE_MDN: cM++; break;
        case ROLE_FWD: cF++; break;
        case ROLE_GROOM: cG++; break;
        case ROLE_ESCW: cW++; break;
        case ROLE_GF: gfSpikes++; break;
        default: break;
      }
    }
    ema(rateLoom, cLoom, groupSize(ROLE_LOOM));
    ema(rateDNaL, cDL, groupSize_[ROLE_DNA][SIDE_LEFT]);
    ema(rateDNaR, cDR, groupSize_[ROLE_DNA][SIDE_RIGHT]);
    ema(rateMDN, cM, groupSize(ROLE_MDN));
    ema(rateFwd, cF, groupSize(ROLE_FWD));
    ema(rateGroom, cG, groupSize(ROLE_GROOM));
    ema(rateEscW, cW, groupSize(ROLE_ESCW));
    ema(ratePop, nSpiked, FLY_N);
  }

private:
  int32_t v_[FLY_N];
  uint8_t refr_[FLY_N];
  uint8_t role_[FLY_N], side_[FLY_N];
  int16_t base_[FLY_N];
  uint16_t rowStart_[FLY_N + 1], rowInh_[FLY_N];
  int32_t inhQ_[INH_SLOTS][FLY_N];
  uint8_t qHead_ = 0;
  uint16_t spiked_[FLY_N];
  uint16_t groupSize_[10][2];
  uint32_t rng_ = 1;
  uint32_t burstUntil_ = 0, burstNext_ = 12000;
  FlyRole stimRole_ = ROLE_OTHER;
  FlySide stimSide_ = SIDE_ANY;
  int32_t stimStrength_ = 0;
  uint32_t stimUntil_ = 0;

  uint32_t rand() {   // xorshift32
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
  }

  static void ema(float& rate, int count, int groupN) {
    if (groupN < 1) groupN = 1;
    rate += (count * 1000.0f / groupN - rate) * (1.0f / 120.0f);
  }
};
