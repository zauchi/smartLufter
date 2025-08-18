#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "INA226.h"

// ---------- Pins ----------
const uint8_t I2C_SDA_PIN = 21;   // <- bestätigt
const uint8_t I2C_SCL_PIN = 19;   // <- bestätigt

const uint8_t ESC_PIN       = 22; // weg von I2C-Pins
const uint32_t PWM_FREQ_HZ  = 50;
const uint8_t  PWM_RES_BITS = 16;

// ---------- ESC-Endpunkte ----------
const int MIN_US = 1000;
const int MAX_US = 2000;

// ---------- Testprofil ----------
const uint8_t  START_PCT = 10;
const uint8_t  STOP_PCT  = 60;
const uint8_t  STEP_PCT  = 10;
const uint32_t STEP_INTERVAL_MS = 5000;
const uint32_t ARMING_MS = 3000;

// ---------- INA226 ----------
INA226 ina(0x40);  // 0x40 laut Scan

// ---------- SH1106 (U8g2, 128x64) ----------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ---------- Helpers ----------
const float PERIOD_US = 1e6f / PWM_FREQ_HZ;
const uint32_t DUTY_MAX = (1u << PWM_RES_BITS) - 1u;

int percentToUs(int pct){
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return MIN_US + (int)((MAX_US - MIN_US) * (pct / 100.0f));
}
uint32_t usToDuty(int us){
  float d = (us / PERIOD_US) * DUTY_MAX;
  if (d < 0) d = 0; if (d > DUTY_MAX) d = DUTY_MAX;
  return (uint32_t)(d + 0.5f);
}
void writePercent(int pct){
  ledcWrite(ESC_PIN, usToDuty(percentToUs(pct))); // ESP32 Core v3: pin-basiert
}


float readPackVoltage(){
  return ina.isConnected() ? ina.getBusVoltage() : NAN; // Volt
}

// Splash-Screen
void showSplash() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso28_tf);
  uint16_t w = u8g2.getStrWidth("smart");
  u8g2.drawStr((128 - w)/2, 30, "smart");
  w = u8g2.getStrWidth("Lufter");
  u8g2.drawStr((128 - w)/2, 62, "Lufter");
  u8g2.sendBuffer();
  delay(2000);
}

// Neue SoC-Berechnung (linear 7.5–12.6 V)
int voltageToSOC3S(float v) {
  if (isnan(v)) return 0;
  if (v <= 7.5f) return 0;
  if (v >= 12.6f) return 100;
  return (int)((v - 7.5f) * 100.0f / (12.6f - 7.5f) + 0.5f);
}

void drawBatteryWithPct(int x, int y, int w, int h, int soc) {
  // Gehäuse
  u8g2.drawFrame(x, y, w, h);

  // Nase
  int nubW = max(2, w/8);
  u8g2.drawBox(x + w, y + (h/4), nubW, h/2);

  // Füllung nach SoC
  int innerW = w - 2;
  int innerH = h - 2;
  int fillW = (innerW * soc) / 100;
  if (fillW > 0) {
    u8g2.drawBox(x+1, y+1, fillW, innerH);
  }

  // Weiße Fläche für Text (über gesamte Batterie-Innenfläche)
  u8g2.setDrawColor(1);
  u8g2.drawBox(x+1, y+1, innerW, innerH);

  // Schwarze Zahl oben drauf
  char buf[6];
  snprintf(buf, sizeof(buf), "%d%%", soc);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_5x8_tf);
  uint16_t tw = u8g2.getStrWidth(buf);
  int tx = x + (w - tw)/2;
  int ty = y + h - 3;  // etwas höher, damit es mittig wirkt
  u8g2.drawStr(tx, ty, buf);

  // Zeichenfarbe zurücksetzen
  u8g2.setDrawColor(1);
}


// Hauptanzeige
void drawUI(int pct, float v) {
  u8g2.clearBuffer();

  // Großer Wert (Gas)
  char pctStr[8];
  snprintf(pctStr, sizeof(pctStr), "%d%%", pct);
  u8g2.setFont(u8g2_font_logisoso24_tf);
  uint16_t w = u8g2.getStrWidth(pctStr);
  u8g2.drawStr((128 - w)/2, 38, pctStr);

  // Spannung links unten
  int soc = voltageToSOC3S(v);
  char line[16];
  if (isnan(v)) snprintf(line, sizeof(line), "--.--V");
  else          snprintf(line, sizeof(line), "%.2fV", v);
  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.drawStr(0, 63, line);

  // Batterie rechts unten
  drawBatteryWithPct(90, 50, 36, 14, soc);

  // Low-Batt-Warnung
  if (!isnan(v) && v <= 7.5f) {
    if ((millis()/500) % 2 == 0) { // blinkt alle 0.5s
      u8g2.setFont(u8g2_font_7x13B_tf);
      u8g2.drawStr(20, 20, "LOW BAT!");
    }
  }

  u8g2.sendBuffer();
}




void setup(){
  Serial.begin(115200);
  delay(300);
  Serial.println("Start...");

  // I2C auf 21/19
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // SH1106 @ 0x3C (7-bit) -> U8g2 erwartet 8-bit-Adresse
  u8g2.setI2CAddress(0x3C * 2);
  u8g2.begin();
  u8g2.setBusClock(100000);  // I2C auf 100 kHz -> stabiler
  u8g2.setContrast(160);     // 0..255, 150–190 meist gut
  u8g2.setFlipMode(0);       // bei Bedarf 1

  showSplash();

  // INA226
  if (!ina.begin()){
    Serial.println("INA226 nicht gefunden (0x40)?");
  } else {
    Serial.println("INA226 OK");
  }

  // ESC PWM
  if (!ledcAttach(ESC_PIN, PWM_FREQ_HZ, PWM_RES_BITS)){
    Serial.println("LEDC attach failed");
    while(true) delay(1000);
  }

  // Arming
  writePercent(0);
  drawUI(0, NAN);
  Serial.println("Arming...");
  delay(ARMING_MS);
  Serial.println("Run...");

}

void loop(){
  // Hoch
  for (int p = START_PCT; p <= STOP_PCT; p += STEP_PCT){
    writePercent(p);
    uint32_t t0 = millis();
    while (millis() - t0 < STEP_INTERVAL_MS){
      float v = readPackVoltage();
      drawUI(p, v);
      delay(200);
    }
  }
  // Runter
  for (int p = STOP_PCT - STEP_PCT; p >= START_PCT; p -= STEP_PCT){
    writePercent(p);
    uint32_t t0 = millis();
    while (millis() - t0 < STEP_INTERVAL_MS){
      float v = readPackVoltage();
      drawUI(p, v);
      delay(200);
    }
  }
}