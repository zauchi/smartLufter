#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "INA226.h"

// -------- Pins (ESP32-C6) --------
const uint8_t I2C_SDA_PIN = 6;
const uint8_t I2C_SCL_PIN = 7;
const uint8_t ESC_PIN     = 10;   // PWM output to ESC
const int     POT_PIN     = 4;    // ADC input from potentiometer

// -------- ESC / PWM --------
const uint32_t PWM_FREQ_HZ  = 50;     // Standard RC PWM frequency
const uint8_t  PWM_RES_BITS = 16;     // 16-bit resolution
const int MIN_US = 1000;              // ESC min pulse width [µs]
const int MAX_US = 2000;              // ESC max pulse width [µs]

// -------- Battery / Safety --------
const float  LOW_VOLTAGE_V = 7.5f;    // 3S LiPo cutoff voltage
bool fanAllowed = false;              // Global flag: ESC enabled

// -------- Potentiometer Calibration --------
const int POT_RAW_MIN = 0;            // Minimum raw ADC value
const int POT_RAW_MAX = 3310;         // Maximum raw ADC value

// -------- Input Filtering & Ramp --------
const uint32_t UPDATE_INTERVAL_MS = 15;   // Control update interval
const float    INPUT_ALPHA        = 0.2f; // Pot smoothing factor (0..1)
const float    RAMP_PCT_PER_SEC   = 10.0f;// Max % change per second

float potFiltPct = 0.0f;      // Filtered potentiometer [%]
float outPct     = 0.0f;      // Output in percent [%]
uint32_t lastUpdateMs = 0;
float outUs = (float)MIN_US;  // Actual PWM output [µs]

// Ramp slope in µs/sec (derived from RAMP_PCT_PER_SEC)
const float RAMP_US_PER_SEC = (MAX_US - MIN_US) * (RAMP_PCT_PER_SEC / 100.0f);

// -------- Helpers --------
inline int pctFromRaw(int raw) {
  raw = constrain(raw, POT_RAW_MIN, POT_RAW_MAX);
  return (int) lroundf((raw - POT_RAW_MIN) * 100.0f / (POT_RAW_MAX - POT_RAW_MIN));
}

// -------- INA226 (I2C @0x44) --------
INA226 ina(0x44);

// -------- Display SH1106 --------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// -------- ESC Helpers --------
const float PERIOD_US = 1e6f / PWM_FREQ_HZ;
const uint32_t DUTY_MAX = (1u << PWM_RES_BITS) - 1u;

// Limit the change of current -> target by maxDeltaUs
static inline float slewTowardsUs(float current, float target, float maxDeltaUs) {
  if (target > current + maxDeltaUs) return current + maxDeltaUs;
  if (target < current - maxDeltaUs) return current - maxDeltaUs;
  return target;
}

// Convert pulse width [µs] into duty cycle for LEDC
uint32_t usToDuty(float us){
  float d = (us / PERIOD_US) * DUTY_MAX;
  d = constrain(d, 0.0f, (float)DUTY_MAX);
  return (uint32_t)(d + 0.5f);
}

// Write exact pulse width to ESC
void writeUs(float us){
  us = constrain(us, (float)MIN_US, (float)MAX_US);
  ledcWrite(ESC_PIN, usToDuty(us));
}

// Write percentage (0–100) to ESC
void writePercent(int pct){
  pct = constrain(pct, 0, 100);
  int us = MIN_US + (int)((MAX_US - MIN_US) * (pct / 100.0f));
  ledcWrite(ESC_PIN, usToDuty(us));
}

// Convert percentage to pulse width [µs]
inline float pctToUs(float pct){
  pct = constrain(pct, 0.0f, 100.0f);
  return MIN_US + (MAX_US - MIN_US) * (pct / 100.0f);
}

// -------- Battery Helpers --------
float readPackVoltage(){
  return ina.isConnected() ? ina.getBusVoltage() : NAN;
}

int voltageToSOC3S(float v) {
  if (isnan(v)) return 0;
  if (v <= 7.5f) return 0;
  if (v >= 12.6f) return 100;
  return (int)((v - 7.5f) * 100.0f / (12.6f - 7.5f) + 0.5f);
}

// -------- UI: Battery --------
void drawBatteryWithPct(int x, int y, int w, int h, int soc) {
  u8g2.drawFrame(x, y, w, h);
  int nubW = max(2, w/8);
  u8g2.drawBox(x + w, y + (h/4), nubW, h/2);

  int innerW = w - 2, innerH = h - 2;
  int fillW  = (innerW * soc) / 100;
  if (fillW > 0) u8g2.drawBox(x+1, y+1, fillW, innerH);

  // White box for text
  u8g2.setDrawColor(1);
  u8g2.drawBox(x+1, y+1, innerW, innerH);

  // Overlay percentage text
  char buf[6]; snprintf(buf, sizeof(buf), "%d%%", soc);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_5x8_tf);
  int tw = u8g2.getStrWidth(buf);
  u8g2.drawStr(x + (w - tw)/2, y + h - 3, buf);

  u8g2.setDrawColor(1); // reset
}

// -------- UI: Fan Animation --------
void drawFan(int cx, int cy, int r, int angleDeg) {
  u8g2.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
  for (int i=0;i<3;i++){
    float ang = (angleDeg + i*120) * PI / 180.0f;
    int x2 = cx + cosf(ang) * (r-2);
    int y2 = cy + sinf(ang) * (r-2);
    u8g2.drawLine(cx, cy, x2, y2);
  }
}

// -------- UI: Main --------
void drawUI(int pct, float v) {
  u8g2.clearBuffer();

  // --- Low Battery Warning --- //
  if (!isnan(v) && v <= LOW_VOLTAGE_V) {
    writePercent(0);
    fanAllowed = false;

    // White background
    u8g2.setDrawColor(1);
    u8g2.drawBox(0,0,128,64);

    // Blinking text
    if ((millis()/500) % 2 == 0) {
      u8g2.setFont(u8g2_font_7x13_tf);
      const char* msg = "LOW BATTERY!";
      int w = u8g2.getStrWidth(msg);
      u8g2.setDrawColor(0);
      u8g2.drawStr((128 - w)/2, 36, msg);
    }

    u8g2.sendBuffer();
    return;
  }

  // --- Throttle display (large number) --- //
  char pctStr[8]; snprintf(pctStr, sizeof(pctStr), "%d%%", pct);
  u8g2.setFont(u8g2_font_logisoso24_tf);
  int w = u8g2.getStrWidth(pctStr);
  u8g2.drawStr((128 - w)/2, 38, pctStr);

  // --- Voltage + battery icon --- //
  int soc = voltageToSOC3S(v);
  char line[16];
  if (isnan(v)) snprintf(line, sizeof(line), "--.--V");
  else          snprintf(line, sizeof(line), "%.2fV", v);
  u8g2.setFont(u8g2_font_7x13_tf);
  u8g2.drawStr(0, 63, line);

  drawBatteryWithPct(90, 50, 36, 14, soc);

  // --- Fan animation (stops at 0%) --- //
  static int fanAngle = 0;
  if (pct > 0) {
    fanAngle = (fanAngle + (pct/5) * 2) % 360;
  }
  drawFan(115, 15, 10, fanAngle);

  u8g2.sendBuffer();
}

// -------- UI: Splash Screen --------
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

// ----------------- SETUP ----------------- //
void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Display init
  u8g2.setI2CAddress(0x3C*2);
  u8g2.begin();
  u8g2.setBusClock(100000);
  u8g2.setContrast(160);
  u8g2.setFlipMode(0);

  showSplash();

  // INA226 init
  if (!ina.begin()) {
    Serial.println("INA226 not found");
  }

  // ADC (potentiometer)
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

// ----------------- LOOP ----------------- //
void loop() {
  // --- Battery check --- //
  float v = readPackVoltage()-0.64;
  if (!isnan(v) && v <= LOW_VOLTAGE_V) {
    outUs = MIN_US;
    writeUs(outUs);
    fanAllowed = false;
    drawUI(0, v);
    delay(100);
    return;
  }

  // --- Timed update loop --- //
  uint32_t now = millis();
  uint32_t dt  = now - lastUpdateMs;
  if (dt < UPDATE_INTERVAL_MS) {
    int dispPct = (int)lroundf((outUs - MIN_US) * 100.0f / (MAX_US - MIN_US));
    drawUI(fanAllowed ? dispPct : 0, v);
    return;
  }
  lastUpdateMs = now;

  // --- Read potentiometer & filter --- //
  int raw = analogRead(POT_PIN);
  int pct = pctFromRaw(raw);
  potFiltPct = INPUT_ALPHA * pct + (1.0f - INPUT_ALPHA) * potFiltPct;

  // --- Target in µs --- //
  float targetUs = fanAllowed ? pctToUs(potFiltPct) : (float)MIN_US;

  // --- Apply ramp (limit slew rate) --- //
  float maxDeltaUs = RAMP_US_PER_SEC * (dt / 1000.0f);
  outUs = slewTowardsUs(outUs, targetUs, maxDeltaUs);

  // --- Write to ESC --- //
  writeUs(outUs);

  // --- UI update --- //
  int dispPct = (int)lroundf((outUs - MIN_US) * 100.0f / (MAX_US - MIN_US));
  drawUI(dispPct, v);
}