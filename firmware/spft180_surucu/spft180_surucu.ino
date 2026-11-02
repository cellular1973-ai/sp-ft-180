/*
  SP-FT-180 Yildiz-Poligon Tam Toroid - harici surucu kutusu - ESP32 DevKitC (WROOM-32, Arduino-ESP32 core 3.x)
  TASLAK, test edilmedi. CW-FT-180 kaide surucusunun (cwft180_surucu) tek kanalli, 12 V adaptorlu, ayrik
  MOSFET koprulu surumu. Bobin kaidedeki iki banana soketine kabloyla baglanir.

  Bobin (Biot-Savart / Neumann, girdap_bobin/dogrulama/uc_bobin_180.py rodin_full, 1 mm Cu, N=37 M=15):
    10.63 m, 233 mOhm, 42.3 uH, tau 0.18 ms | merkez 136 uT/A, eksende +50 mm 77 uT/A, tup ici 143 uT/A, 1 m 0.03 uT/A
  Donanim (on tasarim):
    12 V adaptor (DC jak) -> 2 x IR2104 + 4 x IRF540N tam kopru -> 47 uH seri sok -> 10 mOhm sont
    -> klemens -> banana soketler -> bobin (kart: pcb_spft/). Sont uzerinde INA240A1 (G = 20, REF ortada: 1.65 V + 0.2 V/A).
    12 V -> buck 5 V -> DevKitC 5V pini; 12 V ayrica IR2104 VCC. VIN: 100k / 22k bolucu.
    12 V / (0.23 + 0.05 sok + ~0.03 kopru/sont) ~ 40 A olacagindan akim yalniz PWM ile sinirlanir: 3 A icin ~%8 doluluk.
  Zamanlama: 10 kHz adim, 32 bit faz biriktirici; sinus 256 noktali tablo + lock-in genlik olcumu,
    darbe faz tasinca baslar (kenar 0.1 ms, tepede I*R, kenarda L*dI/dt).
  Program: en fazla 12 adim (f / dalga / dakika) sirayla calisir; oturum suresi adimlarin toplamidir.

  Komutlar (Seri 115200 ve BLE ayni):
    on | off | a <mA> | t <dakika> | f <Hz> | m sinus | m darbe | d <yuzde> | w <us> (0: doluluk kullan)
    schumann | p <f>/<s|dNN|wNNN>/<dk>,... | p (programi sil) | led on | led off | s (durum) | cal
    ornek program: p 7.83/w500/5,10/s/10,100/d25/5
*/
#include <Arduino.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "durum_led.h"
#include "kopru.h"

// ---------------- pinler (DevKitC; 0, 2, 5, 12, 15 acilis pinleri, 6-11 flash - PWM/SD icin kullanma) ----------------
constexpr int PIN_SD  = 27;   // iki IR2104'un SD girisi (aktif yuksek = calis), 10k pull-down: acilista kopru kapali
constexpr int PIN_VIN = 35;   // 12 V / 5.55 (ADC1, yalniz giris)
static Kopru K{25, 26, 34};   // IN kol A, IN kol B, INA240 cikisi (ADC1)

// ---------------- zamanlama ----------------
constexpr uint32_t TICK_US = 100;               // 10 kHz
constexpr int      N_TAB = 256;                 // sinus tablosu; indeks = faz >> 24
constexpr int      ADC_EVERY = 4;               // sinusta her 4 adimda bir akim ornegi
constexpr float    F_MIN = 0.5f, F_MAX = 200.0f;

// ---------------- sinirlar / ayar ----------------
constexpr float I_SET_MAX = 3.0f;     // A tepe (merkezde ~410 uT)
constexpr float VIN_MIN = 10.0f, VIN_MAX = 15.0f, VIN_DIV = (100.0f + 22.0f) / 22.0f;
constexpr float R_INIT  = 0.31f;      // Ohm: bobin 0.233 + sok ~0.05 + kopru/sont (ilk tahmin)
constexpr float L_TOP   = 47e-6f + 42.3e-6f;   // seri sok (L1) + bobin
constexpr float T_RISE  = TICK_US * 1e-6f;     // darbe kenari: 1 adim = 0.1 ms
constexpr int   DUTY_MIN = 10, DUTY_MAX = 50, N_ON_MIN = 4;
constexpr float UT_PER_A = 136.3f;             // bobin merkezi, 1 A
constexpr int   N_PROG = 12;

enum Fault : uint8_t { F_NONE, F_OC, F_VIN_LOW, F_VIN_HIGH, F_OPEN };
const char *FAULT_TXT[] = {"yok", "asiri akim", "besleme dusuk", "besleme yuksek", "bobin bagli degil"};
enum Mode : uint8_t { M_SINUS, M_DARBE };

struct Adim { float f; Mode m; uint8_t duty; uint16_t pw_us; uint16_t sn; };

static float SINE[N_TAB];
static volatile bool  g_run = false;
static volatile Mode  g_mode = M_SINUS;       // istenen (on / yeniden baslatmada etkin olur)
static volatile float g_freq = 100.0f;
static volatile int   g_duty_pct = 25;
static volatile uint32_t g_pw_us = 0;         // 0 -> doluluk
static volatile float g_iset = 1.0f;
static volatile uint32_t g_session_s = 20 * 60;
static volatile Fault g_fault = F_NONE;
// calisan ayar (startOutput'ta kopyalanir; surucu gorevi yalniz bunlari okur)
static Mode  r_mode = M_SINUS;
static uint32_t r_step = 0;                   // faz adimi (2^32 = bir periyot)
static int   r_n_on = 25;
static volatile bool g_reset = false;
static float g_vin = 12.0f, g_phase_deg = 0;
static uint32_t g_run_started_ms = 0;
static Adim g_prog[N_PROG];
static int g_nprog = 0, g_pstep = -1;         // g_pstep: calisan adim, -1 program yok/durdu
static uint32_t g_step_started_ms = 0;
static TaskHandle_t g_task = nullptr;
static hw_timer_t *g_timer = nullptr;
static BLECharacteristic *g_status_ch = nullptr;
static volatile bool g_ble_conn = false;

static float readVin() { return analogReadMilliVolts(PIN_VIN) * VIN_DIV / 1000.0f; }

static void outputOff() {
  digitalWrite(PIN_SD, LOW);
  K.off();
}

static void trip(Fault f) {
  g_run = false;
  if (g_timer) timerStop(g_timer);
  outputOff();
  g_fault = f;
}

static bool vinOk() {
  if (g_vin < VIN_MIN) { trip(F_VIN_LOW); return false; }
  if (g_vin > VIN_MAX) { trip(F_VIN_HIGH); return false; }
  return true;
}

static void startOutput() {
  if (g_fault != F_NONE) return;
  g_vin = readVin();
  if (!vinOk()) return;
  r_mode = g_mode;
  r_step = (uint32_t)((double)g_freq * TICK_US * 1e-6 * 4294967296.0);
  uint32_t period_us = (uint32_t)lroundf(1e6f / g_freq);
  uint32_t pw = g_pw_us ? g_pw_us : period_us * g_duty_pct / 100;
  r_n_on = constrain((int)(pw / TICK_US), N_ON_MIN, (int)(period_us / TICK_US / 2));
  K.ampV = g_iset * R_INIT;                   // ileri besleme
  K.sat = 0;
  g_reset = true;
  K.off();
  digitalWrite(PIN_SD, HIGH);
  delayMicroseconds(200);                     // alt MOSFET'ler acik: bootstrap kondansatorleri dolsun
  g_run = true;
  timerRestart(g_timer);
  timerStart(g_timer);
}

static String restartIfRunning(String reply) {   // mod/frekans degisikligi: oturum suresi korunur
  if (g_run) {
    uint32_t t0 = g_run_started_ms;
    trip(F_NONE);
    startOutput();
    g_run_started_ms = t0;
  }
  return reply;
}

// ---------------- surucu gorevi (10 kHz) ----------------
void IRAM_ATTR onTick() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(g_task, &woken);
  portYIELD_FROM_ISR(woken);
}

// darbe gerilimi: k = 0 yukselen kenar, 1..n-2 tepe, n-1 dusen kenar, sonrasi 0
static inline float pulseV(int k, int n, float ampV) {
  float vl = L_TOP * g_iset / T_RISE;
  if (k == 0)     return  vl + 0.5f * ampV;
  if (k < n - 1)  return ampV;
  if (k == n - 1) return -vl + 0.5f * ampV;
  return 0.0f;
}

static void periodDone(float im) {
  float vmax = D_MAX * g_vin - (r_mode == M_DARBE ? L_TOP * g_iset / T_RISE : 0.0f);
  if (K.update(im, g_iset, vmax)) trip(F_OPEN);
}

static void driverTask(void *) {
  uint32_t ph = 0;
  int tick = 0, n_adc = 0, k = 0, n_pl = 0;
  float aS = 0, aC = 0, acc = 0;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!g_run) continue;
    if (g_reset) { g_reset = false; ph = 0; tick = 0; n_adc = 0; aS = aC = 0; k = 0; n_pl = 0; acc = 0; }
    uint32_t prev = ph;

    if (r_mode == M_DARBE) {
      int n = r_n_on;
      if (k <= n) {
        K.write(pulseV(k, n, K.ampV), g_vin);
        bool win = k >= n / 2 && k <= n - 2;  // tepenin ikinci yarisi
        if (win || k == n) {
          float i = K.readA();
          if (fabsf(i) > I_TRIP) { trip(F_OC); continue; }
          if (win) { acc += i; n_pl++; }
        }
        if (k == n) periodDone(n_pl ? acc / n_pl : 0.0f);
        k++;
      }
      ph += r_step;
      if (ph < prev) { k = 0; n_pl = 0; acc = 0; }  // yeni periyot
      continue;
    }

    int ia = ph >> 24;
    float sa = SINE[ia];
    K.write(K.ampV * sa, g_vin);
    if (tick++ % ADC_EVERY == 0) {
      float i = K.readA();
      if (fabsf(i) > I_TRIP) { trip(F_OC); continue; }
      aS += i * sa;
      aC += i * SINE[(ia + N_TAB / 4) & (N_TAB - 1)];
      n_adc++;
    }
    ph += r_step;
    if (ph < prev && n_adc > 0) {             // periyot tamamlandi: lock-in genlik I = (2/N) sqrt(Ss^2 + Sc^2)
      float is = 2.0f * aS / n_adc, ic = 2.0f * aC / n_adc;
      g_phase_deg = atan2f(-ic, is) * 57.2958f;
      periodDone(sqrtf(is * is + ic * ic));
      aS = aC = 0; n_adc = 0;
    }
  }
}

// ---------------- program ----------------
static void applyStep(int k) {
  const Adim &s = g_prog[k];
  g_freq = s.f; g_mode = s.m; g_duty_pct = s.duty; g_pw_us = s.pw_us;
  g_pstep = k;
  g_step_started_ms = millis();
}

static String parseProgram(String arg) {       // "7.83/w500/5,10/s/10,100/d25/5"
  arg.trim();
  g_nprog = 0; g_pstep = -1;
  if (arg.length() == 0 || arg == "sil") return "program=yok";
  int start = 0;
  while (start < (int)arg.length() && g_nprog < N_PROG) {
    int end = arg.indexOf(',', start);
    if (end < 0) end = arg.length();
    char tok[40], md[12];
    arg.substring(start, end).toCharArray(tok, sizeof tok);
    float f = 0, dk = 0;
    if (sscanf(tok, "%f/%11[^/]/%f", &f, md, &dk) != 3 || dk <= 0) { g_nprog = 0; return String("program hatasi: ") + tok; }
    Adim s{fminf(fmaxf(f, F_MIN), F_MAX), M_SINUS, 25, 0, (uint16_t)fminf(dk * 60.0f, 65000.0f)};
    if (md[0] == 'd') { s.m = M_DARBE; s.duty = constrain(atoi(md + 1), DUTY_MIN, DUTY_MAX); }
    else if (md[0] == 'w') { s.m = M_DARBE; s.pw_us = (uint16_t)max(atoi(md + 1), N_ON_MIN * (int)TICK_US); }
    else if (md[0] != 's') { g_nprog = 0; return String("program hatasi: ") + tok; }
    g_prog[g_nprog++] = s;
    start = end + 1;
  }
  uint32_t tot = 0;
  for (int k = 0; k < g_nprog; k++) tot += g_prog[k].sn;
  return "program=" + String(g_nprog) + " adim, " + String(tot / 60.0f, 1) + "dk";
}

static void programTick() {                    // 1 s'de bir: adim suresi doldu mu
  if (!g_run || g_pstep < 0) return;
  if ((millis() - g_step_started_ms) / 1000 < g_prog[g_pstep].sn) return;
  if (g_pstep + 1 >= g_nprog) { trip(F_NONE); g_pstep = -1; return; }
  applyStep(g_pstep + 1);
  restartIfRunning("");
}

// ---------------- komutlar (Seri + BLE) ----------------
static String statusText() {
  char b[300], mod[20], prog[16];
  if (g_mode == M_SINUS) strcpy(mod, "sinus");
  else if (g_pw_us)      snprintf(mod, sizeof mod, "darbe/%luus", (unsigned long)g_pw_us);
  else                   snprintf(mod, sizeof mod, "darbe/%d", g_duty_pct);
  if (g_nprog == 0)      strcpy(prog, "yok");
  else                   snprintf(prog, sizeof prog, "%d/%d", g_pstep + 1, g_nprog);
  uint32_t ses = g_session_s;
  snprintf(b, sizeof b,
           "%s mod=%s f=%.2fHz I_set=%.0fmA I=%.0fmA V=%.0fmV faz=%.0f Vin=%.2fV Bm~%.0fuT kalan=%lus prog=%s "
           "led=%s hata=%s",
           g_run ? "ACIK" : "KAPALI", mod, (float)g_freq, g_iset * 1000, K.imeas * 1000, K.ampV * 1000,
           g_phase_deg, g_vin, g_run ? UT_PER_A * K.imeas : 0.0f,
           g_run ? (unsigned long)(ses - min(ses, (uint32_t)((millis() - g_run_started_ms) / 1000))) : 0UL,
           prog, g_led_en ? "acik" : "kapali", FAULT_TXT[g_fault]);
  return String(b);
}

static String handleCommand(String c) {
  c.trim();
  if (c == "on") {
    g_fault = F_NONE;
    if (g_nprog > 0) {
      uint32_t tot = 0;
      for (int k = 0; k < g_nprog; k++) tot += g_prog[k].sn;
      g_session_s = tot;
      applyStep(0);
    }
    if (g_run) trip(F_NONE);
    startOutput();
    g_run_started_ms = millis();
    return statusText();
  }
  if (c == "off") { trip(F_NONE); g_pstep = -1; return statusText(); }
  if (c == "s")   return statusText();
  if (c == "cal") {
    bool was = g_run; trip(F_NONE); delay(5);
    K.calibrate();
    if (was) restartIfRunning("");
    return "ofset=" + String(K.ioff_mV, 1) + "mV";
  }
  if (c.startsWith("a ")) {
    g_iset = fminf(fmaxf(c.substring(2).toFloat() / 1000.0f, 0.0f), I_SET_MAX);
    return "I_set=" + String(g_iset * 1000, 0) + "mA";
  }
  if (c.startsWith("t ")) {
    g_session_s = (uint32_t)(fmaxf(c.substring(2).toFloat(), 1.0f) * 60);
    return "oturum=" + String(g_session_s / 60) + "dk";
  }
  if (c.startsWith("f ")) {
    g_freq = fminf(fmaxf(c.substring(2).toFloat(), F_MIN), F_MAX);
    return restartIfRunning("f=" + String(g_freq, 2) + "Hz");
  }
  if (c == "m sinus") { g_mode = M_SINUS; return restartIfRunning("mod=sinus"); }
  if (c == "m darbe") { g_mode = M_DARBE; return restartIfRunning("mod=darbe"); }
  if (c.startsWith("d ")) {
    g_duty_pct = constrain(c.substring(2).toInt(), DUTY_MIN, DUTY_MAX);
    g_pw_us = 0;
    return restartIfRunning("doluluk=%" + String(g_duty_pct));
  }
  if (c.startsWith("w ")) {
    long w = c.substring(2).toInt();
    g_pw_us = w <= 0 ? 0 : (uint32_t)max(w, (long)(N_ON_MIN * TICK_US));
    return restartIfRunning(g_pw_us ? "genislik=" + String(g_pw_us) + "us" : String("genislik=doluluk"));
  }
  if (c == "schumann") {
    g_freq = 7.83f; g_mode = M_DARBE; g_pw_us = 500;
    return restartIfRunning("schumann: 7.83Hz darbe 500us");
  }
  if (c == "p" || c.startsWith("p ")) {
    bool was = g_run && g_pstep >= 0;
    if (was) { trip(F_NONE); g_pstep = -1; }
    return parseProgram(c.length() > 2 ? c.substring(2) : String(""));
  }
  if (c == "led on" || c == "led off") { g_led_en = c == "led on"; return g_led_en ? "led=acik" : "led=kapali"; }
  return "komutlar: on | off | a <mA> | t <dk> | f <Hz> | m sinus|darbe | d <yuzde> | w <us> | schumann | "
         "p <f>/<s|dNN|wNNN>/<dk>,... | led on|off | s | cal";
}

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *ch) override {
    String reply = handleCommand(String(ch->getValue().c_str()));
    if (g_status_ch) { g_status_ch->setValue(reply.c_str()); g_status_ch->notify(); }
  }
};

class SrvCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { g_ble_conn = true; }
  void onDisconnect(BLEServer *) override {   // yayini yeniden baslat: yoksa cihaz bir daha bulunamaz
    g_ble_conn = false;
    BLEDevice::startAdvertising();
  }
};

static void setupBLE() {
  BLEDevice::init("SPFT180");
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new SrvCallbacks());
  BLEService *svc = srv->createService("7a1c0201-5b4e-4f6a-9c1d-2e3f4a5b6c7d");
  BLECharacteristic *cmd = svc->createCharacteristic("7a1c0202-5b4e-4f6a-9c1d-2e3f4a5b6c7d",
                                                     BLECharacteristic::PROPERTY_WRITE);
  cmd->setCallbacks(new CmdCallbacks());
  g_status_ch = svc->createCharacteristic("7a1c0203-5b4e-4f6a-9c1d-2e3f4a5b6c7d",
                                          BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  g_status_ch->addDescriptor(new BLE2902());   // NimBLE bunu kendisi ekler
#endif
  svc->start();
  BLEDevice::getAdvertising()->addServiceUUID(svc->getUUID());
  BLEDevice::startAdvertising();
}

// ---------------- kurulum ----------------
void setup() {
  pinMode(PIN_SD, OUTPUT);
  digitalWrite(PIN_SD, LOW);
  ledInit();
  Serial.begin(115200);
  for (int k = 0; k < N_TAB; k++) SINE[k] = sinf(2.0f * PI * k / N_TAB);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VIN, ADC_11db);
  K.begin();
  delay(20);
  K.calibrate();

  xTaskCreatePinnedToCore(driverTask, "drv", 4096, nullptr, configMAX_PRIORITIES - 1, &g_task, 1);
  g_timer = timerBegin(1000000);              // 1 MHz zaman tabani
  timerAttachInterrupt(g_timer, &onTick);
  timerAlarm(g_timer, TICK_US, true, 0);
  timerStop(g_timer);                         // yalniz surus sirasinda calisir

  setupBLE();                                 // BLE cekirdek 0'da, surucu gorevi cekirdek 1'de
  Serial.println("SPFT180 hazir. " + handleCommand("?"));
}

void loop() {
  static uint32_t last = 0;
  if (Serial.available()) Serial.println(handleCommand(Serial.readStringUntil('\n')));

  if (millis() - last >= 1000) {               // 1 s: besleme, oturum/program, durum bildirimi
    last = millis();
    g_vin = 0.7f * g_vin + 0.3f * readVin();
    if (g_run) vinOk();
    if (g_run && (millis() - g_run_started_ms) / 1000 >= g_session_s) { trip(F_NONE); g_pstep = -1; }
    programTick();
    if (g_status_ch) { String st = statusText(); g_status_ch->setValue(st.c_str()); g_status_ch->notify(); }
  }
  ledUpdate(g_fault != F_NONE, g_run, g_ble_conn);
  delay(20);
}
