// FlyBrain — фізичний робот-муха на Raspberry Pi Pico (RP2040).
//
// Поведінкою керує симуляція реального ланцюга мозку дрозофіли
// (FlyWire FAFB v783, 668 нейронів, 18 968 синапсів), а не if (dist < 20).
//   HC-SR04 (око)  ->  LC4/LPLC2 (детектори насування)  ->  GF (гігантське волокно)  ->  серво (втеча)
//
// Ядро 0: мозок, рівно 1000 кроків LIF на секунду.
// Ядро 1: далекомір, сервопривід, світлодіод, Serial-монітор і команди.
//
// Плата: "Raspberry Pi Pico" з ядра Earle Philhower (arduino-pico).
// Serial Monitor: 115200, введіть h + Enter для списку команд.

#include <Servo.h>

// Граф синапсів (~74 КБ) — у SRAM, а не у флеші: випадковий доступ із флешу через XIP-кеш повільний
#define FLY_RAM __not_in_flash("fly")
// step() — теж із SRAM: ядро 1 (Serial, printf) інакше витісняє його з XIP-кешу -> піки часу кроку
#define FLY_FAST __attribute__((noinline, section(".time_critical.fly_step")))
#include "fly_brain.h"

// ---------------- Піни ----------------
const int TRIG_PIN  = 2;   // GP2 (фізичний пін 4)
const int ECHO_PIN  = 3;   // GP3 (пін 5) — через дільник 1 кОм / 2 кОм!
const int SERVO_PIN = 4;   // GP4 (пін 6)

// ---------------- Тіло (сервопривід) ----------------
const int REST_ANGLE   = 90;    // спокійна поза
const int ESCAPE_ANGLE = 170;   // ривок при втечі
const uint32_t ESCAPE_HOLD_MS     = 150;   // скільки тримати ривок
const uint32_t ESCAPE_COOLDOWN_MS = 800;   // «відновлення» після стрибка

// ---------------- Око (HC-SR04 -> насування) ----------------
const uint32_t SONAR_PERIOD_MS = 60;     // опитування датчика
const float LOOM_RANGE_CM  = 100.0f;     // далі цього об'єкти не лякають
const float LOOM_REL_FULL  = 2.0f;       // швидкість наближення / відстань (1/с) для повного насування
const float HOVER_NEAR_CM  = 15.0f;      // рука, що просто зависла поруч, тривожить
const float HOVER_MAX      = 0.25f;      // ...але нижче порогу втечі (~0.3)
const float LOOM_DECAY_TAU = 0.15f;      // с, згасання сигналу насування

// ---------------- Пороги поведінки (як у SignalBuilder з desktop-fly) ----------------
const float MDN_BACKWARD_HZ = 8.0f;   // MDN > 8 Гц -> задкування
const float GROOM_HZ        = 8.0f;   // DNg11 > 8 Гц -> грумінг
const float WALK_HZ         = 3.0f;   // DNp09 > 3 Гц -> ходьба

// ======================================================================
// Спільні дані між ядрами
// ======================================================================
struct Shared {
  // ядро 1 -> ядро 0
  volatile int32_t loomQ14;
  volatile int32_t puffQ14;
  volatile uint32_t stimSeq;
  volatile uint8_t stimRole, stimSide;
  volatile int32_t stimStrength;
  volatile uint32_t stimMs;
  // ядро 0 -> ядро 1
  volatile uint32_t gfSpikes, simMs, stepUs, stepUsMax, overruns;
  volatile float rateLoom, rateDNaL, rateDNaR, rateMDN, rateFwd, rateGroom, rateEscW, ratePop;
};
Shared sh;

FlyBrain brain;

// ======================================================================
// ЯДРО 0 — мозок
// ======================================================================
static uint32_t nextUs;
static uint32_t lastStimSeq;

void setup() {
  brain.begin(rp2040.hwrand32());
  nextUs = micros();
}

void loop() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextUs) < 0) return;
  nextUs += 1000;
  if ((int32_t)(now - nextUs) > 50000) {   // відстали на >50 мс — не наздоганяти
    nextUs = now + 1000;
    sh.overruns = sh.overruns + 1;
  }

  // входи від ядра 1
  brain.loomL = brain.loomR = sh.loomQ14;   // один далекомір = обидва ока
  brain.airPuff = sh.puffQ14;
  if (sh.stimSeq != lastStimSeq) {
    lastStimSeq = sh.stimSeq;
    __sync_synchronize();
    brain.stimulate((FlyRole)sh.stimRole, (FlySide)sh.stimSide, sh.stimStrength, sh.stimMs);
  }

  const uint32_t t0 = micros();
  brain.step();
  const uint32_t dt = micros() - t0;

  // виходи для ядра 1
  sh.stepUs = dt;
  if (dt > sh.stepUsMax) sh.stepUsMax = dt;
  sh.rateLoom = brain.rateLoom;
  sh.rateDNaL = brain.rateDNaL;
  sh.rateDNaR = brain.rateDNaR;
  sh.rateMDN = brain.rateMDN;
  sh.rateFwd = brain.rateFwd;
  sh.rateGroom = brain.rateGroom;
  sh.rateEscW = brain.rateEscW;
  sh.ratePop = brain.ratePop;
  sh.simMs = brain.simMs;
  sh.gfSpikes = brain.gfSpikes;
}

// ======================================================================
// ЯДРО 1 — тіло
// ======================================================================
Servo leg;

// --- HC-SR04 на перериваннях (не блокує, на відміну від pulseIn) ---
volatile uint32_t echoRiseUs, echoWidthUs;
volatile bool echoDone;

void echoISR() {
  const uint32_t t = micros();
  if (digitalRead(ECHO_PIN)) {
    echoRiseUs = t;
  } else {
    echoWidthUs = t - echoRiseUs;
    echoDone = true;
  }
}

void sonarTrigger() {
  echoDone = false;
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

// --- стан ока ---
float distCm = -1;              // відфільтрована відстань, -1 = немає даних
float hist[3] = {400, 400, 400};
float loom = 0;                 // 0..1
float loomOverride = 0;         // команда 'l'
float puffLeft = 0;             // с, команда 'p'
uint32_t lastSonarMs;

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  return a > b ? a : b;
}

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Відстань -> сигнал насування, як computeLoom() у desktop-fly:
// швидкість кутового розширення об'єкта + «великий об'єкт впритул».
void updateEye(uint32_t nowMs) {
  if (nowMs - lastSonarMs < SONAR_PERIOD_MS) return;
  const float dtS = (nowMs - lastSonarMs) / 1000.0f;
  lastSonarMs = nowMs;

  float raw = -1;
  if (echoDone) {
    raw = echoWidthUs / 58.0f;
    if (raw < 2 || raw > 400) raw = -1;
  }
  sonarTrigger();
  if (raw < 0) raw = 400;   // нічого не видно = далеко

  hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = raw;
  const float d = median3(hist[0], hist[1], hist[2]);   // прибирає поодинокі «стрибки» датчика

  float target = 0;
  if (distCm > 0) {
    const float approach = (distCm - d) / dtS;           // см/с, >0 — наближається
    const float rel = approach / (d > 1 ? d : 1);        // 1/с
    target = clampf(rel / LOOM_REL_FULL, 0, 1) * clampf(1 - d / LOOM_RANGE_CM, 0, 1);
    target += clampf((HOVER_NEAR_CM - d) / HOVER_NEAR_CM, 0, 1) * HOVER_MAX;
  }
  distCm = d;

  loom *= expf(-dtS / LOOM_DECAY_TAU);
  if (target > loom) loom = target;
}

// --- поведінка ---
enum Behavior { IDLE, WALK, GROOM, BACKWARD, ESCAPE };
const char* const BEHAVIOR_NAME[] = {"спокій", "ходьба", "грумінг", "задкування", "ВТЕЧА"};
Behavior behavior = IDLE;
uint32_t escapeStartMs = 0, escapeCount = 0;
uint32_t seenGf = 0;
float phase = 0;

void updateBody(uint32_t nowMs, float dtS) {
  // GF вистрілив -> миттєвий стрибок (якщо не у відновленні)
  const uint32_t gf = sh.gfSpikes;
  if (gf != seenGf) {
    seenGf = gf;
    if (behavior != ESCAPE && nowMs - escapeStartMs > ESCAPE_COOLDOWN_MS) {
      behavior = ESCAPE;
      escapeStartMs = nowMs;
      escapeCount++;
      leg.write(ESCAPE_ANGLE);
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.printf("[%lu мс] GF СПАЙК -> ВТЕЧА #%lu (відстань %.0f см, насування %.2f)\n",
                    (unsigned long)sh.simMs, (unsigned long)escapeCount, distCm, loom);
    }
  }

  if (behavior == ESCAPE) {
    if (nowMs - escapeStartMs < ESCAPE_HOLD_MS) return;
    digitalWrite(LED_BUILTIN, LOW);
    behavior = IDLE;
  }

  // інші поведінки — з частот командних нейронів
  if (sh.rateMDN > MDN_BACKWARD_HZ) behavior = BACKWARD;
  else if (sh.rateGroom > GROOM_HZ) behavior = GROOM;
  else if (sh.rateFwd > WALK_HZ) behavior = WALK;
  else behavior = IDLE;

  float angle = REST_ANGLE;
  switch (behavior) {
    case BACKWARD:   // лапка відведена назад і ритмічно гребе
      phase += dtS * 1.5f;
      angle = REST_ANGLE - 30 + 15 * sinf(phase * TWO_PI);
      break;
    case GROOM:      // дрібне швидке «тертя лапок»
      phase += dtS * 6.0f;
      angle = REST_ANGLE + 12 * sinf(phase * TWO_PI);
      break;
    case WALK: {     // крок; амплітуда ∝ walkDrive = rateFwd / 10
      const float drive = clampf(sh.rateFwd / 10.0f, 0, 1.3f);
      phase += dtS * 2.0f;
      angle = REST_ANGLE + 25 * drive * sinf(phase * TWO_PI);
      break;
    }
    default: {       // тремтіння ∝ nervous = rateLoom / 80
      const float nervous = clampf(sh.rateLoom / 80.0f, 0, 1);
      phase += dtS * 8.0f;
      angle = REST_ANGLE + 8 * nervous * sinf(phase * TWO_PI);
      break;
    }
  }
  if (phase > 1000) phase = 0;
  leg.write((int)angle);
}

// --- Serial: статус і команди «оптогенетики» ---
bool streamStatus = true;
uint32_t lastStatusMs;

void requestStim(FlyRole role, FlySide side, float strength, uint32_t ms, const char* what) {
  sh.stimRole = role;
  sh.stimSide = side;
  sh.stimStrength = (int32_t)(strength * FlyBrain::ONE);
  sh.stimMs = ms;
  __sync_synchronize();
  sh.stimSeq = sh.stimSeq + 1;
  Serial.printf("Стимуляція: %s\n", what);
}

void printHelp() {
  Serial.println(F(
    "\nКоманди (символ + Enter):\n"
    "  g  стимулювати GF (гігантське волокно) -> втеча\n"
    "  m  MDN 1 с -> задкування (moonwalker)\n"
    "  c  DNg11 1 с -> грумінг\n"
    "  w  DNp09 1 с -> ходьба\n"
    "  a / d  DNa01/02 зліва / справа 1 с (кермування)\n"
    "  l  тест насування 0.6 (як 'escapeTest' у DesktopFly)\n"
    "  p  потік повітря 1 с -> сенсорні нейрони\n"
    "  s  увімкнути/вимкнути потік статусу\n"
    "  h  ця довідка\n"));
}

void handleSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'g': requestStim(ROLE_GF, SIDE_ANY, 0.5f, 40, "GF (DNp01)"); break;
      case 'm': requestStim(ROLE_MDN, SIDE_ANY, 0.1f, 1000, "MDN"); break;
      case 'c': requestStim(ROLE_GROOM, SIDE_ANY, 0.1f, 1000, "DNg11"); break;
      case 'w': requestStim(ROLE_FWD, SIDE_ANY, 0.1f, 1000, "DNp09"); break;
      case 'a': requestStim(ROLE_DNA, SIDE_LEFT, 0.1f, 1000, "DNa ліві"); break;
      case 'd': requestStim(ROLE_DNA, SIDE_RIGHT, 0.1f, 1000, "DNa праві"); break;
      case 'l': loomOverride = 0.6f; Serial.println("Насування 0.6"); break;
      case 'p': puffLeft = 1.0f; Serial.println("Потік повітря"); break;
      case 's': streamStatus = !streamStatus; break;
      case 'h': case '?': printHelp(); break;
      default: break;
    }
  }
}

void printStatus() {
  Serial.printf("t=%6.1fс  d=%5.1fсм  loom=%.2f | LC %5.1f  DNa %4.1f/%4.1f  MDN %4.1f  "
                "DNg11 %4.1f  DNp09 %4.1f  escW %4.1f  pop %4.1f Гц | GF %lu | крок %lu/%lu мкс | %s\n",
                sh.simMs / 1000.0f, distCm, loom,
                sh.rateLoom, sh.rateDNaL, sh.rateDNaR, sh.rateMDN,
                sh.rateGroom, sh.rateFwd, sh.rateEscW, sh.ratePop,
                (unsigned long)sh.gfSpikes, (unsigned long)sh.stepUs, (unsigned long)sh.stepUsMax,
                BEHAVIOR_NAME[behavior]);
  sh.stepUsMax = 0;
}

uint32_t lastBodyMs;

void setup1() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);
  leg.attach(SERVO_PIN);
  leg.write(REST_ANGLE);

  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.printf("\nFlyBrain: %d нейронів, %d синапсів (FlyWire v783)\n", FLY_N, FLY_E);
  printHelp();
  lastSonarMs = lastBodyMs = lastStatusMs = millis();
  sonarTrigger();
}

void loop1() {
  const uint32_t now = millis();
  handleSerial();
  updateEye(now);

  if (now - lastBodyMs >= 20) {   // 50 Гц — частота керування серво
    const float dtS = (now - lastBodyMs) / 1000.0f;
    lastBodyMs = now;

    loomOverride = loomOverride > 0 ? loomOverride - dtS * 1.2f : 0;
    const float l = loom > loomOverride ? loom : loomOverride;
    sh.loomQ14 = (int32_t)(clampf(l, 0, 1) * FlyBrain::ONE);
    if (puffLeft > 0) puffLeft -= dtS;
    sh.puffQ14 = puffLeft > 0 ? FlyBrain::ONE : 0;

    updateBody(now, dtS);
  }

  if (streamStatus && now - lastStatusMs >= 250) {
    lastStatusMs = now;
    printStatus();
  }
}
