#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Wire.h>

#include "LGFX_Config.h"

LGFX tft;

namespace ui {
constexpr int kScreenW = 320;
constexpr int kScreenH = 240;

constexpr int kBatPin = 35;
constexpr int kHwStartStopPin = 25;
constexpr int kBuzzerPin = 33;
constexpr int kBuzzerChannel = 3;
constexpr int kOpticalPin = 26;
constexpr int kI2cSdaPin = 21;
constexpr int kI2cSclPin = 22;
constexpr uint8_t kMpuLeftAddr = 0x69;
constexpr uint8_t kMpuRightAddr = 0x68;
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

enum class ScreenMode : uint8_t {
  Home = 0,
  Oscilloscope = 1,
  CalibLayoutSelect = 2,
  CalibConfirmStart = 3,
  CalibCountdown = 4,
  CalibMeasuring = 5,
  CalibSavePrompt = 6,
  CalibDone = 7,
};

ScreenMode currentScreen = ScreenMode::Home;

struct Rect {
  int x;
  int y;
  int w;
  int h;
};

enum class SensorLayout : uint8_t {
  SameDirection = 0,
  OppositeFacing = 1,
};

Rect btnCalibRect = {0, 0, 0, 0};
Rect btnMeasureRect = {0, 0, 0, 0};
Rect btnSettingsRect = {0, 0, 0, 0};
Rect btnScopeRect = {0, 0, 0, 0};
Rect btnScopeBackRect = {8, 32, 72, 24};
Rect scopeSliderRect = {122, 52, 186, 4};
int scopeSliderValue = 30;
bool scopePaused = false;

Rect btnLayoutSameRect = {18, 92, 136, 96};
Rect btnLayoutOppRect = {166, 92, 136, 96};
Rect btnCalibYesRect = {38, 168, 110, 46};
Rect btnCalibNoRect = {172, 168, 110, 46};
Rect btnCalibBackRect = {8, 28, 72, 24};

SensorLayout selectedLayout = SensorLayout::SameDirection;
bool hasLayoutSelection = false;

unsigned long calibPhaseStartMs = 0;
unsigned long lastCalibBlinkMs = 0;
bool calibBlinkOn = false;
int lastCountdownBeepSecond = -1;

float zeroLxSum = 0.0f;
float zeroLySum = 0.0f;
float zeroRxSum = 0.0f;
float zeroRySum = 0.0f;
uint32_t zeroSamples = 0;

struct ZeroBaselineData {
  SensorLayout layout;
  float lX;
  float lY;
  float rX;
  float rY;
  uint32_t sampleCount;
  bool valid;
};

ZeroBaselineData zeroBaseline = {SensorLayout::SameDirection, 0, 0, 0, 0, 0, false};

bool mpuLeftOk = false;
bool mpuRightOk = false;
unsigned long lastMpuRetryMs = 0;

constexpr int kScopeLeft = 8;
constexpr int kScopeTop = 60;
constexpr int kScopeW = 304;
constexpr int kScopeH = 146;
constexpr int kScopeSamples = 304;
constexpr int kScopeSampleIntervalMs = 2;
constexpr int kScopeRawMaxSamples = 4000;
Rect scopePlotRect = {kScopeLeft, kScopeTop, kScopeW, kScopeH};
int16_t scopeSigLeft[kScopeSamples] = {0};
int16_t scopeSigRight[kScopeSamples] = {0};
int16_t scopeSigOpt[kScopeSamples] = {0};
bool scopeLastOptState = false;
unsigned long lastRpmEdgeMs = 0;
unsigned long lastRpmPeriodMs = 0;
float rpmDisplay = 0.0f;
int scopeWindowMsFiltered = 1600;
int rawWriteIndex = 0;
int rawCount = 0;
int16_t rawLeft[kScopeRawMaxSamples] = {0};
int16_t rawRight[kScopeRawMaxSamples] = {0};
int16_t rawOpt[kScopeRawMaxSamples] = {0};
unsigned long lastScopeDrawMs = 0;
unsigned long lastScopeSampleMs = 0;
LGFX_Sprite scopeSprite(&tft);
bool scopeSpriteReady = false;

constexpr int kStatusIconX = 150;
constexpr int kStatusIconY = 12;
constexpr int kStatusIconRadius = 10;
constexpr int kStatusDotRadius = 2;
constexpr int kStatusTextX = 165;
constexpr int kStatusTextY = 6;

void drawStartupScreen();
unsigned long scopeSampleIntervalMs();
int scopeWindowMs();
unsigned long scopeDrawIntervalMs();
void drawScopeTimeSlider();
void rebuildScopeFromRawWindow();
void drawScopeRpmValue();
void drawCalibrationLayoutScreen();
void drawCalibrationConfirmScreen();
void drawCalibrationCountdownScreen();
void drawCalibrationMeasuringScreen();
void drawCalibrationSavePromptScreen();
void drawCalibrationDoneScreen(bool saved);
void updateCalibrationFlow();
void enterCalibrationLayoutScreen();
void enterCalibrationConfirmScreen();
void startCalibrationCountdown();
void startCalibrationMeasuring();
void enterCalibrationSavePrompt();
void resetZeroAccum();
void saveZeroAccumToBaseline();
void buzzerBeep(uint16_t freq, uint16_t durationMs);

void buzzerBeep(uint16_t freq, uint16_t durationMs) {
  ledcWriteTone(kBuzzerChannel, freq);
  delay(durationMs);
  ledcWriteTone(kBuzzerChannel, 0);
}

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
  tft.setTextColor(TFT_WHITE);

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
  tft.setTextColor(TFT_WHITE);

  const char* txt = isBalancing ? "Баланс" : "Изчакване";
  int w = tft.textWidth(txt);
  tft.fillRect(kStatusTextX - 2, kStatusTextY - 2, w + 6, 18, TFT_BLACK);
  tft.setCursor(kStatusTextX, kStatusTextY);
  tft.print(txt);
}

void drawNoSensorsWarning() {
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_RED);
  tft.setCursor(10, 35);
  tft.println("Няма свързани сензори!");
}

bool pointInRect(uint16_t px, uint16_t py, const Rect& r) {
  return (px >= r.x && px < (r.x + r.w) && py >= r.y && py < (r.y + r.h));
}

void drawTwoLineCentered(const char* l1, const char* l2, const Rect& r) {
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);

  int tw1 = tft.textWidth(l1);
  int tw2 = tft.textWidth(l2);
  tft.drawString(l1, r.x + (r.w - tw1) / 2, r.y + 8);
  tft.drawString(l2, r.x + (r.w - tw2) / 2, r.y + 28);
}

void drawSettingsIcon(int x, int y) {
  const uint16_t gearColor = tft.color565(255, 176, 40);
  const uint16_t wrenchColor = tft.color565(70, 216, 245);

  // Gear (orange)
  tft.fillCircle(x, y, 10, gearColor);
  tft.fillCircle(x, y, 4, TFT_BLACK);
  tft.fillRect(x - 2, y - 15, 4, 5, gearColor);
  tft.fillRect(x - 2, y + 10, 4, 5, gearColor);
  tft.fillRect(x - 15, y - 2, 5, 4, gearColor);
  tft.fillRect(x + 10, y - 2, 5, 4, gearColor);
  tft.fillRect(x - 11, y - 11, 4, 4, gearColor);
  tft.fillRect(x + 7, y - 11, 4, 4, gearColor);
  tft.fillRect(x - 11, y + 7, 4, 4, gearColor);
  tft.fillRect(x + 7, y + 7, 4, 4, gearColor);

  // Wrench (cyan)
  const int wx = x + 20;
  const int wy = y + 3;
  tft.drawLine(wx - 8, wy + 8, wx + 7, wy - 7, wrenchColor);
  tft.drawLine(wx - 7, wy + 8, wx + 8, wy - 7, wrenchColor);
  tft.drawCircle(wx + 9, wy - 8, 4, wrenchColor);
  tft.fillCircle(wx - 9, wy + 9, 2, wrenchColor);
}

void drawMainMenuButtons() {
  const int marginX = 8;
  const int gapX = 8;
  const int gapY = 8;
  const int topY = 120;
  const int h = 50;
  const int w = (kScreenW - (2 * marginX) - gapX) / 2;

  const int x1 = marginX;
  const int x2 = marginX + w + gapX;
  const int y1 = topY;
  const int y2 = topY + h + gapY;

  const uint16_t c1 = tft.color565(20, 78, 110);
  const uint16_t c2 = tft.color565(18, 120, 96);
  const uint16_t c3 = tft.color565(90, 72, 18);
  const uint16_t c4 = tft.color565(76, 32, 98);

  btnCalibRect = {x1, y1, w, h};
  btnMeasureRect = {x2, y1, w, h};
  btnSettingsRect = {x1, y2, w, h};
  btnScopeRect = {x2, y2, w, h};

  tft.fillRoundRect(x1, y1, w, h, 8, c1);
  tft.drawRoundRect(x1, y1, w, h, 8, TFT_WHITE);
  tft.fillRoundRect(x2, y1, w, h, 8, c2);
  tft.drawRoundRect(x2, y1, w, h, 8, TFT_WHITE);
  tft.fillRoundRect(x1, y2, w, h, 8, c3);
  tft.drawRoundRect(x1, y2, w, h, 8, TFT_WHITE);
  tft.fillRoundRect(x2, y2, w, h, 8, c4);
  tft.drawRoundRect(x2, y2, w, h, 8, TFT_WHITE);

  drawTwoLineCentered("Калибрация", "в покой", btnCalibRect);
  drawTwoLineCentered("Премини към", "измерване", btnMeasureRect);

  drawSettingsIcon(x1 + 24, y2 + 24);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  const char* btn3 = "Настройки";
  int tw3 = tft.textWidth(btn3);
  tft.drawString(btn3, x1 + 52 + ((w - 54 - tw3) / 2), y2 + 16);

  drawTwoLineCentered("Осцило", "скоп", btnScopeRect);
}

void drawTopBar() {
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(top_left);
  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Кубол Балансьор", 8, 8);

  drawBatteryIcon(kBatFull, 100);
  initStatusIcon();
  drawStatusText();
}

bool initMpu(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(addr);
  Wire.write(0x1C);
  Wire.write(0x10);  // +/-8g range
  return (Wire.endTransmission() == 0);
}

int16_t readMpuAxisX(uint8_t addr, bool* ok) {
  *ok = false;

  Wire.beginTransmission(addr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission((uint8_t)false) != 0) return 0;

  size_t got = Wire.requestFrom((uint8_t)addr, (size_t)2, (bool)true);
  if (got != 2) return 0;

  int16_t v = ((int16_t)Wire.read() << 8) | Wire.read();
  *ok = true;
  return v;
}

void readMpuAxesXY(uint8_t addr, bool* ok, int16_t* ax, int16_t* ay) {
  *ok = false;
  *ax = 0;
  *ay = 0;

  Wire.beginTransmission(addr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission((uint8_t)false) != 0) return;

  size_t got = Wire.requestFrom((uint8_t)addr, (size_t)4, (bool)true);
  if (got != 4) return;

  *ax = ((int16_t)Wire.read() << 8) | Wire.read();
  *ay = ((int16_t)Wire.read() << 8) | Wire.read();
  *ok = true;
}

int16_t synthSignal(float phase, float amp1, float amp2, float tMul) {
  float t = millis() * tMul;
  float v = sinf(t + phase) * amp1 + cosf(t * 0.45f + phase * 0.6f) * amp2;
  return (int16_t)v;
}

void scopePushSample() {
  unsigned long now = millis();
  if (now - lastScopeSampleMs < scopeSampleIntervalMs()) return;
  lastScopeSampleMs = now;

  bool okL = false;
  bool okR = false;
  int16_t leftRaw = readMpuAxisX(kMpuLeftAddr, &okL);
  int16_t rightRaw = readMpuAxisX(kMpuRightAddr, &okR);

  if (!okL) leftRaw = synthSignal(0.4f, 1100.0f, 420.0f, 0.018f);
  if (!okR) rightRaw = synthSignal(1.2f, 950.0f, 500.0f, 0.020f);

  int optLevel = digitalRead(kOpticalPin) ? 220 : -220;
  static int optFiltered = 0;
  optFiltered = (optFiltered * 7 + optLevel) / 8;

  bool optState = (optLevel > 0);
  bool risingEdge = (optState && !scopeLastOptState);
  scopeLastOptState = optState;

  if (risingEdge) {
    if (lastRpmEdgeMs != 0) {
      unsigned long periodMs = now - lastRpmEdgeMs;
      // Accept only realistic RPM periods and smooth output.
      if (periodMs >= 8 && periodMs <= 10000) {
        float rpmInstant = 60000.0f / periodMs;
        if (rpmInstant > 6000.0f) rpmInstant = 6000.0f;
        if (rpmDisplay <= 0.1f) {
          rpmDisplay = rpmInstant;
        } else {
          rpmDisplay = rpmDisplay * 0.88f + rpmInstant * 0.12f;
        }
        lastRpmPeriodMs = periodMs;
      }
    }
    // Always move edge marker forward to avoid getting stuck on stale period.
    lastRpmEdgeMs = now;
  }

  rawLeft[rawWriteIndex] = leftRaw;
  rawRight[rawWriteIndex] = rightRaw;
  rawOpt[rawWriteIndex] = optFiltered;
  rawWriteIndex = (rawWriteIndex + 1) % kScopeRawMaxSamples;
  if (rawCount < kScopeRawMaxSamples) rawCount++;

  unsigned long rpmTimeoutMs = max(1500UL, lastRpmPeriodMs * 3);
  if ((now - lastRpmEdgeMs) > rpmTimeoutMs) {
    rpmDisplay *= 0.90f;
    if (rpmDisplay < 1.0f) rpmDisplay = 0.0f;
  }
}

int scopeWindowMs() {
  // Slider controls visible span around 1..2 revolutions.
  if (rpmDisplay < 1.0f) {
    int stillMs = 900 + (scopeSliderValue * 16);  // 0.9..2.5 sec at zero RPM
    return constrain(stillMs, 600, 3000);
  }

  float oneRevMs = 60000.0f / rpmDisplay;
  float revFactor = 0.95f + (scopeSliderValue / 100.0f) * 1.10f;  // ~1 to ~2 rev
  int winMs = (int)(oneRevMs * revFactor);
  winMs = constrain(winMs, 120, 7000);

  // Smooth window changes to avoid visual jerks.
  scopeWindowMsFiltered = (scopeWindowMsFiltered * 8 + winMs * 2) / 10;
  return scopeWindowMsFiltered;
}

unsigned long scopeDrawIntervalMs() {
  // Slider also controls sweep speed: right = slower movement.
  unsigned long interval = 32 + (scopeSliderValue * 3);  // 32..332ms
  if (rpmDisplay < 5.0f) interval = min(420UL, interval + 80);
  return interval;
}

void rebuildScopeFromRawWindow() {
  int wantedSamples = scopeWindowMs() / (int)kScopeSampleIntervalMs;
  wantedSamples = constrain(wantedSamples, 20, kScopeRawMaxSamples - 1);
  int available = min(rawCount, wantedSamples);
  if (available < 8) return;

  int startIdx = rawWriteIndex - available;
  if (startIdx < 0) startIdx += kScopeRawMaxSamples;

  auto rawSampleAt = [&](const int16_t* arr, int idxFromStart) {
    int idx = startIdx + idxFromStart;
    if (idx >= kScopeRawMaxSamples) idx -= kScopeRawMaxSamples;
    return arr[idx];
  };

  for (int x = 0; x < kScopeSamples; ++x) {
    int idx = ((available - 1) * x) / (kScopeSamples - 1);
    int i0 = constrain(idx - 1, 0, available - 1);
    int i1 = constrain(idx, 0, available - 1);
    int i2 = constrain(idx + 1, 0, available - 1);

    scopeSigLeft[x] = (rawSampleAt(rawLeft, i0) + rawSampleAt(rawLeft, i1) + rawSampleAt(rawLeft, i2)) / 3;
    scopeSigRight[x] = (rawSampleAt(rawRight, i0) + rawSampleAt(rawRight, i1) + rawSampleAt(rawRight, i2)) / 3;
    scopeSigOpt[x] = (rawSampleAt(rawOpt, i0) + rawSampleAt(rawOpt, i1) + rawSampleAt(rawOpt, i2)) / 3;
  }
}

void drawScopeRpmValue() {
  char rpmBuf[20];
  int rpmInt = (int)(rpmDisplay + 0.5f);
  sprintf(rpmBuf, "RPM:%5d", rpmInt);

  tft.fillRect(206, 212, 108, 20, TFT_BLACK);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(rpmBuf, 206, 212);
}

void drawScopeTimeSlider() {
  tft.drawFastHLine(scopeSliderRect.x, scopeSliderRect.y + (scopeSliderRect.h / 2),
                    scopeSliderRect.w, TFT_LIGHTGREY);

  int knobX = scopeSliderRect.x + (scopeSliderValue * (scopeSliderRect.w - 1)) / 100;
  tft.fillCircle(knobX, scopeSliderRect.y + (scopeSliderRect.h / 2), 6,
                 tft.color565(60, 180, 255));
}

int mapScopeY(int16_t v) {
  const int16_t clampV = constrain(v, (int16_t)-2000, (int16_t)2000);
  return kScopeTop + (kScopeH / 2) - ((clampV * (kScopeH / 2 - 2)) / 2000);
}

void drawOscilloscopeScreenStatic() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  btnScopeBackRect = {8, 28, 72, 24};
  tft.fillRoundRect(btnScopeBackRect.x, btnScopeBackRect.y, btnScopeBackRect.w,
                    btnScopeBackRect.h, 6, tft.color565(30, 72, 112));
  tft.drawRoundRect(btnScopeBackRect.x, btnScopeBackRect.y, btnScopeBackRect.w,
                    btnScopeBackRect.h, 6, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Назад", btnScopeBackRect.x + 10, btnScopeBackRect.y + 4);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Осцилоскоп - LIVE", 98, 32);
  drawScopeTimeSlider();

  tft.drawRect(kScopeLeft - 1, kScopeTop - 1, kScopeW + 2, kScopeH + 2, TFT_DARKGREY);
  tft.drawFastHLine(kScopeLeft, kScopeTop + kScopeH / 2, kScopeW, TFT_DARKGREY);

  if (!scopeSpriteReady) {
    scopeSprite.setColorDepth(16);
    scopeSprite.createSprite(kScopeW, kScopeH);
    scopeSpriteReady = true;
  }

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("ЛЯВ сензор", 8, 212);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("ДЕСЕН сензор", 104, 212);
  drawScopeRpmValue();
}

void drawOscilloscopeWaveforms() {
  unsigned long now = millis();
  unsigned long drawIntervalMs = scopeDrawIntervalMs();
  if (now - lastScopeDrawMs < drawIntervalMs) return;
  lastScopeDrawMs = now;

  if (!scopeSpriteReady) return;

  rebuildScopeFromRawWindow();

  scopeSprite.fillScreen(TFT_BLACK);
  scopeSprite.drawFastHLine(0, kScopeH / 2, kScopeW, TFT_DARKGREY);

  bool prevOptHi = false;
  for (int x = 1; x < kScopeSamples; ++x) {
    int px0 = x - 1;
    int px1 = x;

    int yL0 = mapScopeY(scopeSigLeft[x - 1]) - kScopeTop;
    int yL1 = mapScopeY(scopeSigLeft[x]) - kScopeTop;
    int yR0 = mapScopeY(scopeSigRight[x - 1]) - kScopeTop;
    int yR1 = mapScopeY(scopeSigRight[x]) - kScopeTop;
    int yO0 = mapScopeY(scopeSigOpt[x - 1]) - kScopeTop + 8;
    int yO1 = mapScopeY(scopeSigOpt[x]) - kScopeTop + 8;

    scopeSprite.drawLine(px0, yL0, px1, yL1, TFT_GREEN);
    scopeSprite.drawLine(px0, yR0, px1, yR1, TFT_YELLOW);
    // Thicker RPM timing trace for better visibility.
    scopeSprite.drawLine(px0, yO0, px1, yO1, TFT_CYAN);
    scopeSprite.drawLine(px0, yO0 + 1, px1, yO1 + 1, TFT_CYAN);

    bool optHi = scopeSigOpt[x] > 55;
    if (optHi && !prevOptHi) {
      scopeSprite.drawFastVLine(px1, (kScopeH / 2) - 40, 80, TFT_CYAN);
      scopeSprite.drawFastVLine(px1 + 1, (kScopeH / 2) - 40, 80, TFT_CYAN);
    }
    prevOptHi = optHi;
  }

  scopeSprite.pushSprite(kScopeLeft, kScopeTop);
  drawScopeRpmValue();

  uint16_t dotL = mpuLeftOk ? TFT_GREEN : TFT_RED;
  uint16_t dotR = mpuRightOk ? TFT_YELLOW : TFT_RED;
  uint16_t dotO = digitalRead(kOpticalPin) ? TFT_CYAN : TFT_DARKGREY;
  tft.fillCircle(248, 35, 3, dotL);
  tft.fillCircle(258, 35, 3, dotR);
  tft.fillCircle(268, 35, 3, dotO);
}

void scopeTryRecoverSensors() {
  unsigned long now = millis();
  if (now - lastMpuRetryMs < 2000) return;
  lastMpuRetryMs = now;

  if (!mpuLeftOk) mpuLeftOk = initMpu(kMpuLeftAddr);
  if (!mpuRightOk) mpuRightOk = initMpu(kMpuRightAddr);
}

void resetZeroAccum() {
  zeroLxSum = 0.0f;
  zeroLySum = 0.0f;
  zeroRxSum = 0.0f;
  zeroRySum = 0.0f;
  zeroSamples = 0;
}

void saveZeroAccumToBaseline() {
  if (zeroSamples == 0) return;
  zeroBaseline.layout = selectedLayout;
  zeroBaseline.lX = zeroLxSum / zeroSamples;
  zeroBaseline.lY = zeroLySum / zeroSamples;
  zeroBaseline.rX = zeroRxSum / zeroSamples;
  zeroBaseline.rY = zeroRySum / zeroSamples;
  zeroBaseline.sampleCount = zeroSamples;
  zeroBaseline.valid = true;
}

void drawLayoutSensorIcon(const Rect& r, bool oppositeMode) {
  uint16_t bg = oppositeMode ? tft.color565(46, 64, 88) : tft.color565(34, 84, 56);
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 10, TFT_WHITE);

  int ly = r.y + 18;
  int lw = 36;
  int lh = 26;
  int leftX = r.x + 16;
  int rightX = r.x + r.w - 16 - lw;

  // Left sensor (green)
  tft.fillRoundRect(leftX, ly, lw, lh, 4, TFT_GREEN);
  tft.fillRect(leftX + 6, ly + lh, 2, 12, TFT_LIGHTGREY);
  tft.fillRect(leftX + 10, ly + lh, 2, 12, TFT_LIGHTGREY);

  // Right sensor (yellow) with orientation difference.
  tft.fillRoundRect(rightX, ly, lw, lh, 4, TFT_YELLOW);
  tft.fillRect(rightX + 24, ly + lh, 2, 12, TFT_LIGHTGREY);
  tft.fillRect(rightX + 28, ly + lh, 2, 12, TFT_LIGHTGREY);

  if (oppositeMode) {
    tft.drawLine(rightX + lw - 4, ly + 4, rightX + 4, ly + lh - 4, TFT_BLACK);
    tft.drawLine(rightX + lw - 4, ly + lh - 4, rightX + 4, ly + 4, TFT_BLACK);
  } else {
    tft.drawFastHLine(rightX + 5, ly + lh / 2, lw - 10, TFT_BLACK);
  }
}

void drawCalibrationLayoutScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.fillRoundRect(btnCalibBackRect.x, btnCalibBackRect.y, btnCalibBackRect.w,
                    btnCalibBackRect.h, 6, tft.color565(30, 72, 112));
  tft.drawRoundRect(btnCalibBackRect.x, btnCalibBackRect.y, btnCalibBackRect.w,
                    btnCalibBackRect.h, 6, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Назад", btnCalibBackRect.x + 10, btnCalibBackRect.y + 4);

  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Изберете разположение", 36, 58);
  tft.drawString("на сензорите", 96, 76);

  drawLayoutSensorIcon(btnLayoutSameRect, false);
  drawLayoutSensorIcon(btnLayoutOppRect, true);

  if (hasLayoutSelection) {
    Rect selectedRect = (selectedLayout == SensorLayout::SameDirection) ? btnLayoutSameRect : btnLayoutOppRect;
    tft.drawRoundRect(selectedRect.x - 2, selectedRect.y - 2, selectedRect.w + 4, selectedRect.h + 4, 10, TFT_CYAN);
  }

  tft.setFont(&fonts::efontCN_12);
  tft.drawString("Еднакво", btnLayoutSameRect.x + 38, btnLayoutSameRect.y + 70);
  tft.drawString("Срещуположно", btnLayoutOppRect.x + 22, btnLayoutOppRect.y + 70);
}

void drawCalibrationConfirmScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Започване нулево", 68, 66);
  tft.drawString("измерване?", 102, 86);

  tft.setFont(&fonts::efontCN_12);
  tft.drawString("Машината трябва да е в покой.", 54, 112);

  tft.fillRoundRect(btnCalibYesRect.x, btnCalibYesRect.y, btnCalibYesRect.w, btnCalibYesRect.h, 8, tft.color565(28, 112, 58));
  tft.drawRoundRect(btnCalibYesRect.x, btnCalibYesRect.y, btnCalibYesRect.w, btnCalibYesRect.h, 8, TFT_WHITE);
  tft.fillRoundRect(btnCalibNoRect.x, btnCalibNoRect.y, btnCalibNoRect.w, btnCalibNoRect.h, 8, tft.color565(92, 46, 46));
  tft.drawRoundRect(btnCalibNoRect.x, btnCalibNoRect.y, btnCalibNoRect.w, btnCalibNoRect.h, 8, TFT_WHITE);

  tft.setFont(&fonts::efontCN_16);
  tft.drawString("ДА", btnCalibYesRect.x + 38, btnCalibYesRect.y + 12);
  tft.drawString("НЕ", btnCalibNoRect.x + 38, btnCalibNoRect.y + 12);
}

void drawCalibrationCountdownScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("Отдалечете се", 90, 82);
  tft.drawString("Всичко в покой", 84, 102);
}

void drawCalibrationMeasuringScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_CYAN);
  tft.drawString("Измерване в покой", 66, 86);
}

void drawCalibrationSavePromptScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Да запишем", 112, 72);
  tft.drawString("нулевите данни?", 88, 92);

  tft.fillRoundRect(btnCalibYesRect.x, btnCalibYesRect.y, btnCalibYesRect.w, btnCalibYesRect.h, 8, tft.color565(28, 112, 58));
  tft.drawRoundRect(btnCalibYesRect.x, btnCalibYesRect.y, btnCalibYesRect.w, btnCalibYesRect.h, 8, TFT_WHITE);
  tft.fillRoundRect(btnCalibNoRect.x, btnCalibNoRect.y, btnCalibNoRect.w, btnCalibNoRect.h, 8, tft.color565(92, 46, 46));
  tft.drawRoundRect(btnCalibNoRect.x, btnCalibNoRect.y, btnCalibNoRect.w, btnCalibNoRect.h, 8, TFT_WHITE);

  tft.setFont(&fonts::efontCN_16);
  tft.drawString("ДА", btnCalibYesRect.x + 38, btnCalibYesRect.y + 12);
  tft.drawString("НЕ", btnCalibNoRect.x + 38, btnCalibNoRect.y + 12);
}

void drawCalibrationDoneScreen(bool saved) {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();
  tft.setFont(&fonts::efontCN_16);
  if (saved) {
    tft.setTextColor(TFT_GREEN);
    tft.drawString("Zero data saved", 72, 84);
    tft.drawString("OK", 146, 104);
  } else {
    tft.setTextColor(TFT_YELLOW);
    tft.drawString("Process finished", 72, 84);
    tft.drawString("Save skipped", 98, 104);
  }
}

void enterCalibrationLayoutScreen() {
  currentScreen = ScreenMode::CalibLayoutSelect;
  drawCalibrationLayoutScreen();
}

void enterCalibrationConfirmScreen() {
  currentScreen = ScreenMode::CalibConfirmStart;
  drawCalibrationConfirmScreen();
}

void startCalibrationCountdown() {
  currentScreen = ScreenMode::CalibCountdown;
  calibPhaseStartMs = millis();
  lastCalibBlinkMs = 0;
  calibBlinkOn = false;
  lastCountdownBeepSecond = -1;
  drawCalibrationCountdownScreen();
}

void startCalibrationMeasuring() {
  currentScreen = ScreenMode::CalibMeasuring;
  calibPhaseStartMs = millis();
  resetZeroAccum();
  drawCalibrationMeasuringScreen();
}

void enterCalibrationSavePrompt() {
  currentScreen = ScreenMode::CalibSavePrompt;
  drawCalibrationSavePromptScreen();
}

void updateCalibrationFlow() {
  unsigned long now = millis();

  if (currentScreen == ScreenMode::CalibCountdown) {
    unsigned long elapsed = now - calibPhaseStartMs;
    int remain = 10 - (int)(elapsed / 1000);
    if (remain < 0) remain = 0;

    if (remain != lastCountdownBeepSecond) {
      lastCountdownBeepSecond = remain;
      if (remain > 0) {
        buzzerBeep(1800, 40);
      }
    }

    if (now - lastCalibBlinkMs >= 500) {
      lastCalibBlinkMs = now;
      calibBlinkOn = !calibBlinkOn;
      tft.fillRect(0, 138, kScreenW, 28, calibBlinkOn ? tft.color565(52, 20, 20) : TFT_BLACK);
      tft.setFont(&fonts::efontCN_16);
      tft.setTextColor(TFT_WHITE);
      char buf[24];
      sprintf(buf, "Старт след: %d", remain);
      tft.drawString(buf, 96, 142);
    }

    if (elapsed >= 10000) {
      buzzerBeep(2400, 120);
      startCalibrationMeasuring();
    }
    return;
  }

  if (currentScreen == ScreenMode::CalibMeasuring) {
    bool okL = false;
    bool okR = false;
    int16_t lx = 0, ly = 0, rx = 0, ry = 0;
    readMpuAxesXY(kMpuLeftAddr, &okL, &lx, &ly);
    readMpuAxesXY(kMpuRightAddr, &okR, &rx, &ry);

    if (okL && okR) {
      zeroLxSum += lx;
      zeroLySum += ly;
      zeroRxSum += rx;
      zeroRySum += ry;
      zeroSamples++;
    }

    unsigned long elapsed = now - calibPhaseStartMs;
    int remain = 10 - (int)(elapsed / 1000);
    if (remain < 0) remain = 0;

    tft.fillRect(78, 132, 168, 24, TFT_BLACK);
    tft.setFont(&fonts::efontCN_16);
    tft.setTextColor(TFT_WHITE);
    char buf[24];
    sprintf(buf, "Остават: %d", remain);
    tft.drawString(buf, 104, 136);

    if (elapsed >= 10000) {
      buzzerBeep(2600, 140);
      enterCalibrationSavePrompt();
    }
    return;
  }
}

unsigned long scopeSampleIntervalMs() {
  return kScopeSampleIntervalMs;
}

void enterHomeScreen() {
  currentScreen = ScreenMode::Home;
  drawStartupScreen();
}

void enterOscilloscopeScreen() {
  currentScreen = ScreenMode::Oscilloscope;
  drawOscilloscopeScreenStatic();
}

void drawStartupScreen() {
  tft.fillScreen(TFT_BLACK);

  drawTopBar();
  drawNoSensorsWarning();
  drawMainMenuButtons();
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

void handleUiTouch() {
  uint16_t x = 0;
  uint16_t y = 0;
  if (!tft.getTouch(&x, &y)) return;

  static unsigned long lastUiTouchMs = 0;
  unsigned long now = millis();
  if (now - lastUiTouchMs < 220) return;
  lastUiTouchMs = now;

  if (currentScreen == ScreenMode::Home) {
    if (pointInRect(x, y, btnCalibRect)) {
      enterCalibrationLayoutScreen();
      return;
    }
    if (pointInRect(x, y, btnScopeRect)) {
      enterOscilloscopeScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::Oscilloscope) {
    if (pointInRect(x, y, btnScopeBackRect)) {
      enterHomeScreen();
      return;
    }
    if (pointInRect(x, y, scopePlotRect)) {
      scopePaused = !scopePaused;
      return;
    }
    Rect sliderTouch = {scopeSliderRect.x - 4, scopeSliderRect.y - 8,
                        scopeSliderRect.w + 8, scopeSliderRect.h + 16};
    if (pointInRect(x, y, sliderTouch)) {
      int relX = (int)x - scopeSliderRect.x;
      relX = constrain(relX, 0, scopeSliderRect.w - 1);
      scopeSliderValue = (relX * 100) / (scopeSliderRect.w - 1);

      tft.fillRect(scopeSliderRect.x - 6, scopeSliderRect.y - 6,
                   scopeSliderRect.w + 12, 14, TFT_BLACK);
      drawScopeTimeSlider();
      return;
    }
  } else if (currentScreen == ScreenMode::CalibLayoutSelect) {
    if (pointInRect(x, y, btnCalibBackRect)) {
      enterHomeScreen();
      return;
    }
    if (pointInRect(x, y, btnLayoutSameRect)) {
      selectedLayout = SensorLayout::SameDirection;
      hasLayoutSelection = true;
      enterCalibrationConfirmScreen();
      return;
    }
    if (pointInRect(x, y, btnLayoutOppRect)) {
      selectedLayout = SensorLayout::OppositeFacing;
      hasLayoutSelection = true;
      enterCalibrationConfirmScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::CalibConfirmStart) {
    if (pointInRect(x, y, btnCalibYesRect)) {
      startCalibrationCountdown();
      return;
    }
    if (pointInRect(x, y, btnCalibNoRect)) {
      enterCalibrationLayoutScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::CalibSavePrompt) {
    if (pointInRect(x, y, btnCalibYesRect)) {
      saveZeroAccumToBaseline();
      currentScreen = ScreenMode::CalibDone;
      calibPhaseStartMs = millis();
      drawCalibrationDoneScreen(true);
      return;
    }
    if (pointInRect(x, y, btnCalibNoRect)) {
      currentScreen = ScreenMode::CalibDone;
      calibPhaseStartMs = millis();
      drawCalibrationDoneScreen(false);
      return;
    }
  } else if (currentScreen == ScreenMode::CalibDone) {
    enterHomeScreen();
    return;
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

  pinMode(ui::kHwStartStopPin, INPUT_PULLUP);

  ledcSetup(ui::kBuzzerChannel, 2000, 8);
  ledcAttachPin(ui::kBuzzerPin, ui::kBuzzerChannel);
  ledcWriteTone(ui::kBuzzerChannel, 0);

  pinMode(ui::kOpticalPin, INPUT_PULLUP);
  Wire.begin(ui::kI2cSdaPin, ui::kI2cSclPin, 400000);
  ui::mpuLeftOk = ui::initMpu(ui::kMpuLeftAddr);
  ui::mpuRightOk = ui::initMpu(ui::kMpuRightAddr);

  tft.init();
  tft.setRotation(3);  // Landscape: 320x240

  ui::drawStartupScreen();
}

void loop() {
  ui::handleUiTouch();
  ui::updateStatusIcon();
  ui::drawStatusText();
  ui::handleBatteryTouchToggle();
  ui::updateBatteryPeriodic();

  if (ui::currentScreen == ui::ScreenMode::Oscilloscope) {
    if (!ui::scopePaused) {
      ui::scopeTryRecoverSensors();
      ui::scopePushSample();
      ui::drawOscilloscopeWaveforms();
    }
  }

  ui::updateCalibrationFlow();

  delay(10);
}
