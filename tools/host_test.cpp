// Перевірка мозку мухи на ПК (той самий fly_brain.h, що й на Pico).
//   g++ -O2 -std=c++17 -I../FlyBrain host_test.cpp -o host_test && ./host_test
// Повторює ключові інваріанти з desktop-fly (windows/test/simtest.js):
//   GF мовчить 4 с у спокої; GF стріляє ≤ ~10 мс після різкого насування.
#include <chrono>
#include <cstdio>
#include "fly_brain.h"

static FlyBrain brain;

static bool consumeGF(uint32_t& seen) {
  bool fired = brain.gfSpikes != seen;
  seen = brain.gfSpikes;
  return fired;
}

int main() {
  int fails = 0;
  uint32_t seen = 0;
  brain.begin(42);
  std::printf("нейронів %d, синапсів %d, loom-нейронів %d, GF %d\n",
              FLY_N, FLY_E, brain.groupSize(ROLE_LOOM), brain.groupSize(ROLE_GF));

  // 1. Спокій 4 с
  auto t0 = std::chrono::steady_clock::now();
  int gfRest = 0;
  for (int ms = 0; ms < 4000; ms++) { brain.step(); if (consumeGF(seen)) gfRest++; }
  double usPerStep = std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - t0).count() / 4000;
  std::printf("спокій 4 с: популяція %.2f Гц/нейрон, LC %.1f Гц, DNa L/R %.1f/%.1f, MDN %.1f, GF спайків %d\n",
              brain.ratePop, brain.rateLoom, brain.rateDNaL, brain.rateDNaR, brain.rateMDN, gfRest);
  if (gfRest > 0) { std::puts("  FAIL: GF має мовчати у спокої"); fails++; }

  // 2. Різке насування (як ривок курсора): loomL = 1.0, loomR = 0.5
  int gfLoom = 0, latency = -1;
  brain.loomL = FlyBrain::ONE;
  brain.loomR = FlyBrain::ONE / 2;
  for (int ms = 0; ms < 400; ms++) {
    brain.step();
    if (consumeGF(seen)) { gfLoom++; if (latency < 0) latency = ms; }
  }
  brain.loomL = brain.loomR = 0;
  std::printf("насування 0.4 с: LC %.1f Гц, GF спайків %d, перший через %d мс\n",
              brain.rateLoom, gfLoom, latency);
  if (latency < 0 || latency > 15) { std::puts("  FAIL: GF має вистрілити швидко"); fails++; }

  // 3. Поріг: який рівень насування викликає втечу
  for (int lvl = 1; lvl <= 10; lvl++) {
    for (int ms = 0; ms < 1500; ms++) { brain.step(); consumeGF(seen); }
    int n = 0;
    brain.loomL = brain.loomR = FlyBrain::ONE * lvl / 10;
    for (int ms = 0; ms < 300; ms++) { brain.step(); if (consumeGF(seen)) n++; }
    brain.loomL = brain.loomR = 0;
    std::printf("  loom %.1f -> GF спайків за 300 мс: %d\n", lvl / 10.0, n);
  }

  // 4. Оптогенетика: GF 0.5 протягом 40 мс
  for (int ms = 0; ms < 1000; ms++) { brain.step(); consumeGF(seen); }
  brain.stimulate(ROLE_GF, SIDE_ANY, FlyBrain::ONE / 2, 40);
  int gfStim = 0;
  for (int ms = 0; ms < 60; ms++) { brain.step(); if (consumeGF(seen)) gfStim++; }
  std::printf("стимуляція GF: %s\n", gfStim ? "спрацювала" : "НЕ спрацювала");
  if (!gfStim) fails++;

  // 5. Стимуляція командних нейронів
  struct { FlyRole r; const char* name; float* rate; } groups[] = {
    {ROLE_MDN, "MDN (задкування)", &brain.rateMDN},
    {ROLE_GROOM, "DNg11 (грумінг)", &brain.rateGroom},
    {ROLE_FWD, "DNp09 (ходьба)", &brain.rateFwd},
  };
  for (auto& g : groups) {
    for (int ms = 0; ms < 1000; ms++) { brain.step(); consumeGF(seen); }
    float before = *g.rate;
    brain.stimulate(g.r, SIDE_ANY, FlyBrain::ONE / 10, 1000);
    for (int ms = 0; ms < 1000; ms++) { brain.step(); consumeGF(seen); }
    std::printf("стимуляція %s: %.1f -> %.1f Гц\n", g.name, before, *g.rate);
  }

  std::printf("час кроку на ПК: %.1f мкс\n", usPerStep);
  std::puts(fails ? "ЄСТЬ ПОМИЛКИ" : "OK");
  return fails ? 1 : 0;
}
