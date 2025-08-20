#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "INA226.h"

// -------- Pins (ESP32-C6) --------
const uint8_t I2C_SDA_PIN = 6;
const uint8_t I2C_SCL_PIN = 7;
const uint8_t ESC_PIN     = 10;   // PWM out
const int     POT_PIN     = 4;    // ADC1, Poti-Schleifer
const int ESC_CHANNEL = 0;  // frei wählbar, 0–15

// -------- ESC / PWM --------
const uint32_t PWM_FREQ_HZ  = 50;
const uint8_t  PWM_RES_BITS = 16;
const int MIN_US = 1000;
const int MAX_US = 2000;

// -------- Akku/Timing --------
const float  LOW_VOLTAGE_V = 7.5f;    // 3S cutoff
bool fanAllowed = false;

// -------- Poti-Kalibrierung --------
const int POT_RAW_MIN = 0;     // gemessen
const int POT_RAW_MAX = 3310;  // dein 100%-Wert

// ---- Sanfte Regelung ----
const uint32_t UPDATE_INTERVAL_MS = 15;  // häufiger updaten
const float    INPUT_ALPHA        = 0.2f; // Poti-Glättung (0..1, größer = schneller)
const float    RAMP_PCT_PER_SEC   = 10.0f; // max. %/Sekunde Änderung am ESC

float potFiltPct = 0.0f;      // gefilterter Potiwert [%]
float outPct     = 0.0f;      // tatsächlich ausgegebener Wert [%]
uint32_t lastUpdateMs = 0;
float outUs      = (float)MIN_US; // [µs] tatsächlich ausgegebener Puls

// zusätzlich: Rampe in µs
const float RAMP_US_PER_SEC = (MAX_US - MIN_US) * (RAMP_PCT_PER_SEC / 100.0f);

inline int pctFromRaw(int raw) {
  raw = constrain(raw, POT_RAW_MIN, POT_RAW_MAX);
  return (int) lroundf((raw - POT_RAW_MIN) * 100.0f / (POT_RAW_MAX - POT_RAW_MIN));
}

// -------- INA226 (I2C @0x44) --------
INA226 ina(0x44);

// -------- Display SH1106 --------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// -------- Helpers --------
const float PERIOD_US = 1e6f / PWM_FREQ_HZ;
const uint32_t DUTY_MAX = (1u << PWM_RES_BITS) - 1u;

// Begrenzt die Änderung von current->target auf maxDelta (in µs)
static inline float slewTowardsUs(float current, float target, float maxDeltaUs) {
  if (target > current + maxDeltaUs) return current + maxDeltaUs;
  if (target < current - maxDeltaUs) return current - maxDeltaUs;
  return target;
}

uint32_t usToDuty(float us){
  float d = (us / PERIOD_US) * DUTY_MAX;
  if (d < 0) d = 0; if (d > DUTY_MAX) d = DUTY_MAX;
  return (uint32_t)(d + 0.5f);
}

// ESC direkt in µs schreiben (kein Prozent-Runden)
void writeUs(float us){
  us = constrain(us, (float)MIN_US, (float)MAX_US);
  ledcWrite(ESC_PIN, usToDuty(us));
}

void writePercent(int pct){
  pct = constrain(pct, 0, 100);
  int us = MIN_US + (int)((MAX_US - MIN_US) * (pct / 100.0f));
  ledcWrite(ESC_PIN, usToDuty(us));
}

inline float pctToUs(float pct){
  pct = constrain(pct, 0.0f, 100.0f);
  return MIN_US + (MAX_US - MIN_US) * (pct / 100.0f);
}

float readPackVoltage(){
  return ina.isConnected() ? ina.getBusVoltage() : NAN;
}

int voltageToSOC3S(float v) {
  if (isnan(v)) return 0;
  if (v <= 7.5f) return 0;
  if (v >= 12.6f) return 100;
  return (int)((v - 7.5f) * 100.0f / (12.6f - 7.5f) + 0.5f);
}

// ---- Batterie mit Prozent (immer lesbar) ----
void drawBatteryWithPct(int x, int y, int w, int h, int soc) {
  u8g2.drawFrame(x, y, w, h);
  int nubW = max(2, w/8);
  u8g2.drawBox(x + w, y + (h/4), nubW, h/2);
  int innerW = w - 2, innerH = h - 2;
  int fillW  = (innerW * soc) / 100;
  if (fillW > 0) u8g2.drawBox(x+1, y+1, fillW, innerH);
  u8g2.setDrawColor(1);  // weiße Box für Text
  u8g2.drawBox(x+1, y+1, innerW, innerH);
  char buf[6]; snprintf(buf, sizeof(buf), "%d%%", soc);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_5x8_tf);
  int tw = u8g2.getStrWidth(buf);
  u8g2.drawStr(x + (w - tw)/2, y + h - 3, buf);
  u8g2.setDrawColor(1);
}

// ---- kleiner animierter Lüfter ----
void drawFan(int cx, int cy, int r, int angleDeg) {
  u8g2.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
  for (int i=0;i<3;i++){
    float ang = (angleDeg + i*120) * PI / 180.0f;
    int x2 = cx + cosf(ang) * (r-2);
    int y2 = cy + sinf(ang) * (r-2);
    u8g2.drawLine(cx, cy, x2, y2);
  }
}

// ---- UI (inkl. Low-Battery Screen) ----
void drawUI(int pct, float v) {
  u8g2.clearBuffer();

  // --- Low Battery Alarm ---
  if (!isnan(v) && v <= LOW_VOLTAGE_V) {
    writePercent(0);
    fanAllowed = false;

    // Hintergrund dauerhaft weiß
    u8g2.setDrawColor(1);
    u8g2.drawBox(0,0,128,64);

    // Text blinkt (ein/aus)
    bool vis = (millis()/500) % 2 == 0;
    if (vis) {
      u8g2.setFont(u8g2_font_7x13_tf);
      const char* msg = "LOW BATTERY!";
      int w = u8g2.getStrWidth(msg);
      u8g2.setDrawColor(0);  // Schwarz
      u8g2.drawStr((128 - w)/2, 36, msg);
    }

    u8g2.sendBuffer();
    return;
  }

  // großer Gaswert
  char pctStr[8]; snprintf(pctStr, sizeof(pctStr), "%d%%", pct);
  u8g2.setFont(u8g2_font_logisoso24_tf);
  int w = u8g2.getStrWidth(pctStr);
  u8g2.drawStr((128 - w)/2, 38, pctStr);

  // Spannung + Batterie unten
  int soc = voltageToSOC3S(v);
  char line[16];
  if (isnan(v)) snprintf(line, sizeof(line), "--.--V");
  else          snprintf(line, sizeof(line), "%.2fV", v);
  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.drawStr(0, 63, line);

  drawBatteryWithPct(90, 50, 36, 14, soc);

  // Lüfter oben rechts (Speed ∝ Gas, doppelt)
  static int fanAngle = 0;
  if (pct > 0) {
    fanAngle = (fanAngle + (pct/5) * 2) % 360;  // keine Mindestdrehung mehr
  }
  drawFan(115, 15, 10, fanAngle);


  u8g2.sendBuffer();
}

// ---- Splash ----
void showSplash() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso28_tf);
  int w = u8g2.getStrWidth("smart");
  u8g2.drawStr((128 - w)/2, 30, "smart");
  w = u8g2.getStrWidth("Lufter");
  u8g2.drawStr((128 - w)/2, 62, "Lufter");
  u8g2.sendBuffer();
  delay(1200);
}

// ----------------- SETUP -----------------
void setup() {
  // Serial (optional zur Diagnose)
  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  scanI2C();

  // Display
  u8g2.setI2CAddress(0x3C*2);
  u8g2.begin();
  u8g2.setBusClock(100000);
  u8g2.setContrast(160);
  u8g2.setFlipMode(0);

  showSplash();

  // INA226
  if (!ina.begin()) {
    Serial.println("INA226 not found");
  }

  // ADC Poti
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(POT_PIN, ADC_11db);

  // ESC PWM
  if (!ledcAttach(ESC_PIN, PWM_FREQ_HZ, PWM_RES_BITS)) {
    Serial.println("LEDC attach failed");
    while(true) delay(1000);
  }

  outUs = MIN_US;
  writeUs(outUs);

  fanAllowed = true;
}

// ----------------- LOOP -----------------
void loop() {
  // Akku prüfen
  float v = readPackVoltage();
  if (!isnan(v) && v <= LOW_VOLTAGE_V) {
    outUs = MIN_US;
    writeUs(outUs);
    fanAllowed = false;
    drawUI(0, v);
    delay(100);
    return;
  }

  // Zeitgesteuertes Update
  uint32_t now = millis();
  uint32_t dt  = now - lastUpdateMs;
  if (dt < UPDATE_INTERVAL_MS) {
    int dispPct = (int)lroundf( (outUs - MIN_US) * 100.0f / (MAX_US - MIN_US) );
    drawUI(fanAllowed ? dispPct : 0, v);
    return;
  }
  lastUpdateMs = now;

  // Poti lesen & glätten (weiterhin in %)
  int raw = analogRead(POT_PIN);
  int pct = pctFromRaw(raw);                 // 0..100 (int aus Kalibrierung)
  potFiltPct = INPUT_ALPHA * pct + (1.0f - INPUT_ALPHA) * potFiltPct;

  // Ziel in µs
  float targetUs = fanAllowed ? pctToUs(potFiltPct) : (float)MIN_US;

  // erlaubter Schritt (in µs) aus dt
  float maxDeltaUs = RAMP_US_PER_SEC * (dt / 1000.0f);

  // sanft auf Ziel rampen (in µs)
  outUs = slewTowardsUs(outUs, targetUs, maxDeltaUs);

  // zum ESC ausgeben (feinauflösend)
  writeUs(outUs);

  // Anzeige
  int dispPct = (int)lroundf( (outUs - MIN_US) * 100.0f / (MAX_US - MIN_US) );
  drawUI(dispPct, v);
}


