#include <Arduino.h>
#include <LovyanGFX.hpp>

#include "LGFX_Config.h"

LGFX tft;

namespace ui {
constexpr int kScreenW = 320;
constexpr int kScreenH = 240;

constexpr int kBatPin = 35;
constexpr float kBatFull = 8.40f;
constexpr float kBatEmpty = 6.00f;
constexpr unsigned long kBatUpdateIntervalMs = 30000;  // 30 seconds
constexpr float kBatDividerCoeff = 3.162f;

unsigned long lastBatUpdate = 0;
unsigned long lastTouchDraw = 0;
bool showVoltageInsteadOfPercent = false;

bool isBalancing = false;
int statusAngle = 0;
unsigned long lastStatusUpdate = 0;

constexpr int kStatusIconX = 150;
constexpr int kStatusIconY = 12;
constexpr int kStatusIconRadius = 10;
constexpr int kStatusDotRadius = 2;
constexpr int kStatusTextX = 165;
constexpr int kStatusTextY = 6;

float readBatteryVoltage() {
  int raw = analogRead(kBatPin);
  float v_adc = raw * (3.3f / 4095.0f);
  float v_bat = v_adc * kBatDividerCoeff;
  return v_bat;
}

void drawBatteryIcon(float vbat, int percent) {
  const int x = 290;
  const int y = 2;
  const int w = 20;
  const int h = 10;

  uint16_t fillColor = TFT_GREEN;
  if (percent <= 65 && percent > 40) fillColor = TFT_YELLOW;
  if (percent <= 40 && percent > 15) fillColor = TFT_ORANGE;
  if (percent <= 15) fillColor = TFT_RED;

  tft.fillRect(x - 5, y - 5, w + 60, h + 25, TFT_BLACK);
  tft.drawRoundRect(x, y, w, h, 2, TFT_WHITE);
  tft.fillRect(x + w, y + 3, 3, h - 6, TFT_WHITE);

  int fillW = (percent * (w - 2)) / 100;
  if (fillW < 0) fillW = 0;
  if (fillW > (w - 2)) fillW = (w - 2);
  tft.fillRoundRect(x + 1, y + 1, fillW, h - 2, 1, fillColor);

  tft.setFont(&fonts::lgfxJapanGothic_12);
  tft.setTextColor(TFT_WHITE, TFT_TRANSPARENT);

  char buf[12];
  if (showVoltageInsteadOfPercent) {
    sprintf(buf, "%.2fV", vbat);
  } else {
    sprintf(buf, "%d%%", percent);
  }

  int textW = tft.textWidth(buf);
  tft.setCursor(x + (w / 2) - (textW / 2), y + h + 2);
  tft.print(buf);
}

void initStatusIcon() {
  tft.fillRect(kStatusIconX - 15, kStatusIconY - 15, 40, 40, TFT_BLACK);
  tft.drawCircle(kStatusIconX, kStatusIconY, kStatusIconRadius, TFT_WHITE);
  tft.fillCircle(kStatusIconX, kStatusIconY, kStatusIconRadius - 1, TFT_BLACK);
  tft.fillCircle(kStatusIconX, kStatusIconY, 1, TFT_WHITE);
}

void updateStatusIcon() {
  static int lastPx = -1;
  static int lastPy = -1;
  unsigned long now = millis();

  if (!isBalancing) {
    float rad = 0;
    int px = kStatusIconX + cos(rad) * (kStatusIconRadius - 3);
    int py = kStatusIconY + sin(rad) * (kStatusIconRadius - 3);

    if (lastPx != -1) tft.fillCircle(lastPx, lastPy, kStatusDotRadius, TFT_BLACK);
    tft.fillCircle(px, py, kStatusDotRadius, TFT_RED);
    lastPx = px;
    lastPy = py;
    return;
  }

  if (now - lastStatusUpdate > 80) {
    lastStatusUpdate = now;

    if (lastPx != -1) tft.fillCircle(lastPx, lastPy, kStatusDotRadius, TFT_BLACK);

    statusAngle += 15;
    if (statusAngle >= 360) statusAngle = 0;

    float rad = statusAngle * 0.0174533f;
    int px = kStatusIconX + cos(rad) * (kStatusIconRadius - 3);
    int py = kStatusIconY + sin(rad) * (kStatusIconRadius - 3);

    tft.fillCircle(px, py, kStatusDotRadius, TFT_RED);

    lastPx = px;
    lastPy = py;
  }
}

void drawStatusText() {
  static bool lastState = false;
  static bool firstDraw = true;

  if (!firstDraw && lastState == isBalancing) return;

  firstDraw = false;
  lastState = isBalancing;

  tft.setFont(&fonts::efontCN_12);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  const char* txt = isBalancing ? "Баланс" : "Изчакване";
  int w = tft.textWidth(txt);
  tft.fillRect(kStatusTextX - 2, kStatusTextY - 2, w + 6, 18, TFT_BLACK);
  tft.setCursor(kStatusTextX, kStatusTextY);
  tft.print(txt);
}

void drawNoSensorsWarning() {
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setCursor(10, 35);
  tft.println("Няма свързани сензори!");
}

void drawStartupScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(top_left);
  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Кубол Балансьор", 8, 8);

  drawBatteryIcon(kBatFull, 100);
  initStatusIcon();
  drawStatusText();
  drawNoSensorsWarning();
}

void handleBatteryTouchToggle() {
  uint16_t x = 0;
  uint16_t y = 0;
  if (tft.getTouch(&x, &y)) {
    unsigned long now = millis();
    if (now - lastTouchDraw > 350) {
      if (x >= 285 && x <= 320 && y >= 0 && y <= 22) {
        showVoltageInsteadOfPercent = !showVoltageInsteadOfPercent;
        float currentV = readBatteryVoltage();
        int currentP = ((currentV - kBatEmpty) / (kBatFull - kBatEmpty)) * 100.0f;
        currentP = constrain(currentP, 0, 100);
        drawBatteryIcon(currentV, currentP);
      }
      lastTouchDraw = now;
    }
  }
}

void updateBatteryPeriodic() {
  unsigned long now = millis();
  if (now - lastBatUpdate >= kBatUpdateIntervalMs) {
    lastBatUpdate = now;
    float vbat = readBatteryVoltage();
    vbat = constrain(vbat, kBatEmpty, kBatFull);
    int percent = ((vbat - kBatEmpty) / (kBatFull - kBatEmpty)) * 100.0f;
    percent = constrain(percent, 0, 100);
    drawBatteryIcon(vbat, percent);
  }
}
}  // namespace ui

void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(3);  // Landscape: 320x240

  ui::drawStartupScreen();
}

void loop() {
  ui::updateStatusIcon();
  ui::drawStatusText();
  ui::handleBatteryTouchToggle();
  ui::updateBatteryPeriodic();
  delay(10);
}
