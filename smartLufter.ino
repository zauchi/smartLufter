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

// sehr grobe SoC-Schätzung (3S, Ruhespannung!)
int voltageToSOC3S(float v){
  struct P { float v; int soc; };
  const P t[] = {{9.60f,0},{10.80f,10},{11.40f,30},{11.70f,50},{12.00f,70},{12.30f,90},{12.60f,100}};
  if (v <= t[0].v) return 0;
  if (v >= t[6].v) return 100;
  for (int i=0;i<6;i++){
    if (v < t[i+1].v){
      float x = (v - t[i].v) / (t[i+1].v - t[i].v);
      return (int)(t[i].soc + x * (t[i+1].soc - t[i].soc) + 0.5f);
    }
  }
  return 0;
}

float readPackVoltage(){
  return ina.isConnected() ? ina.getBusVoltage() : NAN; // Volt
}

void drawUI(int pct, float v){
  u8g2.clearBuffer();

  // Titel (gut lesbar, nicht zu klein)
  u8g2.setFont(u8g2_font_7x13B_tf);       // 13 px hoch, fett
  u8g2.drawStr(0,13, "ESC Test");

  // GROSSER Prozentwert mittig
  char pctStr[8];
  snprintf(pctStr, sizeof(pctStr), "%d%%", pct);
  u8g2.setFont(u8g2_font_logisoso24_tf);  // ~24 px hoch
  uint16_t w = u8g2.getStrWidth(pctStr);
  int x = (128 - w) / 2;
  u8g2.drawStr(x, 38, pctStr);            // Baseline bei y=38 passt sicher

  // Infozeile: Spannung + SoC
  int soc = isnan(v) ? 0 : voltageToSOC3S(v);
  char line[32];
  if (isnan(v)) snprintf(line, sizeof(line), "U=--.--V   SoC~%d%%", soc);
  else          snprintf(line, sizeof(line), "U=%.2fV   SoC~%d%%", v, soc);
  u8g2.setFont(u8g2_font_7x13_tf);        // etwas größer als 6x10
  u8g2.drawStr(0, 53, line);

  // SoC-Balken unten
  const int barX=0, barY=56, barW=128, barH=8;
  u8g2.drawFrame(barX, barY, barW, barH);
  int fillW = (barW-2) * soc / 100;
  if (fillW > 0) u8g2.drawBox(barX+1, barY+1, fillW, barH-2);

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
