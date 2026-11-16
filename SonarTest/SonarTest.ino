// SonarTest — пошук, куди насправді приходить Echo HC-SR04 на Pico.
// Trig імпульс на GP2, далі 40 мс стежимо за ВСІМА GPIO (тільки читання) —
// показуємо, на якому піні з'явився імпульс і якої ширини.
// Також друкує VBUS (GP24): чи є 5 В на піні 40.
// Serial 115200. Після перевірки залити назад FlyBrain.

const uint8_t TRIGS[] = {2, 4};   // по черзі пробуємо обидва піни як Trig
int ti = 0;

// GP0..GP22, GP26..GP28 (GP23..25 — внутрішні на Pico)
const uint8_t PINS[] = {0,1,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,26,27,28};
const int NP = sizeof(PINS);

uint32_t hits[30];

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NP; i++) pinMode(PINS[i], INPUT_PULLDOWN);
}

void loop() {
  const int trig = TRIGS[ti];
  ti = (ti + 1) % 2;
  for (int i = 0; i < NP; i++) pinMode(PINS[i], INPUT_PULLDOWN);
  delay(5);
  uint32_t before = sio_hw->gpio_in;
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  uint32_t rise[30] = {0}, width[30] = {0};
  uint32_t prev = sio_hw->gpio_in;
  const uint32_t t0 = micros();
  while (micros() - t0 < 40000) {
    const uint32_t now = sio_hw->gpio_in, t = micros();
    const uint32_t ch = now ^ prev;
    if (ch) {
      for (int i = 0; i < NP; i++) {
        const int p = PINS[i];
        if (ch & (1u << p)) {
          if (now & (1u << p)) rise[p] = t;
          else if (rise[p]) width[p] = t - rise[p];
        }
      }
      prev = now;
    }
  }

  Serial.printf("Trig=GP%d | ", trig);
  bool any = false;
  for (int i = 0; i < NP; i++) {
    const int p = PINS[i];
    if (p == trig) continue;
    if (width[p]) {
      hits[p]++;
      any = true;
      Serial.printf("ІМПУЛЬС на GP%d: %lu мкс (%.1f см), всього %lu  ", p,
                    (unsigned long)width[p], width[p] / 58.0f, (unsigned long)hits[p]);
    }
  }
  if (!any) Serial.print("імпульсів ні на одному піні");
  Serial.print(" | постійно HIGH:");
  for (int i = 0; i < NP; i++)
    if (PINS[i] != trig && (before & prev & (1u << PINS[i]))) Serial.printf(" GP%d", PINS[i]);
  Serial.println();
  delay(150);
}
