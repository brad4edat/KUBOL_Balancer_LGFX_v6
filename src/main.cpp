#include <Arduino.h>
#include <LovyanGFX.hpp>

#include "LGFX_Config.h"

LGFX tft;

namespace ui {
constexpr int kScreenW = 320;
constexpr int kScreenH = 240;

void drawBatteryIcon(int x, int y, int bodyW, int bodyH, int color) {
  // Battery body (horizontal icon) with terminal nub on the right.
  const int stroke = 2;
  const int nubW = 4;
  const int nubH = bodyH / 2;
  const int nubY = y + (bodyH - nubH) / 2;

  tft.drawRect(x, y, bodyW, bodyH, color);
  tft.drawRect(x + stroke, y + stroke, bodyW - (2 * stroke), bodyH - (2 * stroke), color);
  tft.fillRect(x + bodyW, nubY, nubW, nubH, color);
}

void drawStartupScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(top_left);
  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Кубол Балансьор", 8, 8);

  const int batteryW = 26;
  const int batteryH = 12;
  const int margin = 8;
  const int batteryX = kScreenW - batteryW - 4 - margin;
  const int batteryY = 8;
  drawBatteryIcon(batteryX, batteryY, batteryW, batteryH, TFT_WHITE);
}
}  // namespace ui

void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(3);  // Landscape: 320x240

  ui::drawStartupScreen();
}

void loop() {
  delay(10);
}
