// Durum LED'i (GPIO2 -> 1k -> LED -> GND, aktif yuksek; DevKitC klonlarindaki kart ustu LED de IO2'de).
//   hata: 5 Hz | calisiyor: 1 s'de 80 ms | durdu + BLE bagli: 2 s'de 30 ms | durdu: 4 s'de 30 ms
#pragma once
#include <Arduino.h>

constexpr int PIN_LED = 2;  // acilis pini: LED yere cektigi icin acilista LOW kalir (dogru seviye)

static bool g_led_en = true;

static inline void ledInit() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

static inline bool ledPattern(uint32_t now, bool fault, bool run, bool conn) {
  if (fault) return (now / 100) % 2 == 0;
  if (run)   return now % 1000 < 80;
  return now % (conn ? 2000 : 4000) < 30;
}

static inline void ledUpdate(bool fault, bool run, bool conn) {
  digitalWrite(PIN_LED, g_led_en && ledPattern(millis(), fault, run, conn) ? HIGH : LOW);
}
