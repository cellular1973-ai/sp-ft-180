// Tam kopru: 2 x IR2104 yarim kopru surucusu (IN = PWM, ortak SD), 4 x N-MOSFET; akim olcumu ve periyot basina
// genlik duzeltmesi (cwft180_surucu/kanal.h ile ayni yasa). Tek kutuplu (unipolar) surus:
//   v > 0: kol A PWM, kol B alt MOSFET acik | v < 0: tersi | v = 0: iki alt MOSFET acik (fren / serbest donus)
// Her kolun alt MOSFET'i her PWM periyodunda iletime girdigi icin bootstrap kondansatoru surekli dolar.
#pragma once
#include <Arduino.h>
#include <math.h>

constexpr uint32_t PWM_FREQ = 40000;      // Hz (47 uH seri sokla akim dalgalanmasi ~0.25 A t-t)
constexpr uint8_t  PWM_BITS = 10;         // 80 MHz / 40 kHz = 2000 -> 10 bit
constexpr uint32_t PWM_MAX  = (1u << PWM_BITS) - 1;
constexpr float SENSE_V_PER_A = 0.010f * 20.0f;  // 10 mOhm x INA240A1 (20) = 0.2 V/A, REF ortada -> +-8 A
constexpr float D_MAX    = 0.95f;         // bootstrap icin her periyotta en az %5 alt MOSFET
constexpr float KI       = 0.25f;         // periyot basina genlik duzeltme kazanci
constexpr float STEP_MAX = 0.05f;         // periyot basina en fazla %5 degisim
constexpr float I_TRIP   = 4.5f;          // A anlik - aninda kapat

struct Kopru {
  int pin_a, pin_b, pin_is;               // IR2104 IN (kol A), IN (kol B), INA240 cikisi
  float ioff_mV = 1650.0f;                // sifir akim ofseti
  float ampV = 0;                         // kopru gerilim genligi komutu (V)
  float imeas = 0;                        // olculen tepe akimi (A)
  uint16_t sat = 0;                       // ust uste doyum sayaci (acik devre algisi)

  void begin() {
    if (!ledcAttach(pin_a, PWM_FREQ, PWM_BITS) || !ledcAttach(pin_b, PWM_FREQ, PWM_BITS))
      Serial.println("LEDC baslatilamadi");
    off();
    analogSetPinAttenuation(pin_is, ADC_11db);
  }

  void off() {                            // iki alt MOSFET acik; SD ayrica kapatir
    ledcWrite(pin_a, 0);
    ledcWrite(pin_b, 0);
  }

  float readA() const { return (analogReadMilliVolts(pin_is) - ioff_mV) / (SENSE_V_PER_A * 1000.0f); }

  void calibrate() {                      // kopru kapaliyken cagrilmali
    uint32_t acc = 0;
    for (int k = 0; k < 64; k++) acc += analogReadMilliVolts(pin_is);
    ioff_mV = acc / 64.0f;
  }

  void write(float v, float vbus) {
    uint32_t d = (uint32_t)lroundf(fminf(fabsf(v) / vbus, D_MAX) * PWM_MAX);
    ledcWrite(pin_a, v > 0 ? d : 0);
    ledcWrite(pin_b, v < 0 ? d : 0);
  }

  // periyot sonu: carpimsal, sinirli genlik duzeltmesi. true -> doyumda ve akim cok dusuk (bobin bagli degil)
  bool update(float im, float iset, float vmax) {
    imeas = im;
    if (iset <= 0) { ampV = 0; sat = 0; return false; }
    float err = (iset - im) / fmaxf(iset, 0.05f);
    ampV *= 1.0f + fminf(fmaxf(KI * err, -STEP_MAX), STEP_MAX);
    if (ampV > vmax) {
      ampV = vmax;
      sat = (im < 0.3f * iset) ? sat + 1 : 0;
      return sat > 50;
    }
    sat = 0;
    return false;
  }
};
