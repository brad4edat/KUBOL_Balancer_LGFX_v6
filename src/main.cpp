#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <time.h>
#include <Preferences.h>

#include "LGFX_Config.h"

LGFX tft;

static QueueHandle_t gSaveQueue = nullptr;

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
constexpr uint32_t kI2cFastHz = 400000;
constexpr uint32_t kI2cSafeHz = 100000;
constexpr unsigned long kSensorDiagIntervalMs = 2000;
constexpr uint8_t kMpuLeftAddr = 0x69;
constexpr uint8_t kMpuRightAddr = 0x68;
constexpr float kBatFull = 8.40f;
constexpr float kBatEmpty = 6.00f;
constexpr unsigned long kBatUpdateIntervalMs = 30000;  // 30 seconds
constexpr float kBatDividerCoeff = 3.162f;

unsigned long lastBatUpdate = 0;
unsigned long lastTouchDraw = 0;
bool showVoltageInsteadOfPercent = false;
float currentBatteryVoltage = kBatFull;
int currentBatteryPercent = 100;
bool batteryUiDirty = true;

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
  CalibRecordName = 8,
  CalibRecordNameKeyboard = 9,
  CalibProceedToBalance = 10,
  RealBalanceStub = 11,
  SettingsMenu = 12,
  SettingsDateTime = 13,
  OscLayoutSelect = 14,
};

ScreenMode currentScreen = ScreenMode::Home;

struct Rect {
  int x;
  int y;
  int w;
  int h;
};

struct DateTimeData {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
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
Rect sensorStatusRect = {10, 30, 300, 60};  // За касанието на съобщението със сензорите
Rect scopeSliderRect = {122, 52, 186, 4};
int scopeSliderValue = 30;
bool scopePaused = false;

Rect btnLayoutSameRect = {18, 92, 136, 120};
Rect btnLayoutOppRect = {166, 92, 136, 120};
Rect btnOscLayoutSameRect = {18, 92, 136, 120};
Rect btnOscLayoutOppRect = {166, 92, 136, 120};
Rect btnCalibYesRect = {38, 168, 110, 46};
Rect btnCalibNoRect = {172, 168, 110, 46};
Rect btnCalibBackRect = {8, 28, 72, 24};
Rect btnNameManualRect = {18, 138, 136, 46};
Rect btnNameDefaultRect = {166, 138, 136, 46};
Rect btnNameContinueRect = {38, 192, 244, 40};
Rect btnKeyDoneRect = {254, 32, 58, 24};
Rect btnRealBalBackHomeRect = {78, 186, 164, 40};
Rect btnSettingsBackRect = {8, 28, 72, 24};
Rect btnSettingsDateTimeRect = {18, 76, 284, 52};
Rect btnSettingsBuzzerRect = {18, 136, 284, 52};
Rect btnBuzzerMinusRect = {214, 148, 36, 28};
Rect btnBuzzerPlusRect = {258, 148, 36, 28};
Rect btnDateTimeBackRect = {8, 28, 72, 24};
Rect btnDateTimeSaveRect = {236, 28, 76, 24};

Rect btnYearMinusRect = {14, 78, 34, 30};
Rect btnYearPlusRect = {272, 78, 34, 30};
Rect btnMonthMinusRect = {14, 112, 34, 30};
Rect btnMonthPlusRect = {272, 112, 34, 30};
Rect btnDayMinusRect = {14, 146, 34, 30};
Rect btnDayPlusRect = {272, 146, 34, 30};
Rect btnHourMinusRect = {14, 180, 34, 30};
Rect btnHourPlusRect = {272, 180, 34, 30};
Rect btnMinuteMinusRect = {14, 214, 34, 24};
Rect btnMinutePlusRect = {272, 214, 34, 24};

DateTimeData manualDateTime = {2026, 3, 11, 12, 0, 0};
unsigned long lastDateTickMs = 0;
uint8_t buzzerVolumePercent = 80;

constexpr int kMaxRecordNameLen = 31;
char defaultRecordName[kMaxRecordNameLen + 1] = {0};
char currentRecordName[kMaxRecordNameLen + 1] = {0};

constexpr int kKbRows = 3;
constexpr int kKbCols = 10;
Rect kbKeyRects[kKbRows][kKbCols] = {};
const char* kbKeyLabels[kKbRows][kKbCols] = {
  {"А", "Б", "В", "Г", "Д", "Е", "Ж", "З", "И", "Й"},
  {"К", "Л", "М", "Н", "О", "П", "Р", "С", "Т", "У"},
  {"Ф", "Х", "Ц", "Ч", "Ш", "Щ", "Ъ", "Ь", "Ю", "Я"},
};

struct SaveRequest {
  SensorLayout layout;
  float lX;
  float lY;
  float rX;
  float rY;
  uint32_t sampleCount;
  bool valid;
  char name[kMaxRecordNameLen + 1];
};

Rect kbBackspaceRect = {10, 200, 94, 30};
Rect kbSpaceRect = {110, 200, 100, 30};
Rect kbOkRect = {216, 200, 94, 30};

SensorLayout selectedLayout = SensorLayout::SameDirection;
bool hasLayoutSelection = false;
SensorLayout selectedLayoutForCalib = SensorLayout::SameDirection;
bool hasLayoutSelectionForCalib = false;
SensorLayout selectedLayoutForOsc = SensorLayout::SameDirection;
bool hasLayoutSelectionForOsc = false;

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
bool opticalSensorOk = true;
bool sensorDiagnosticsDone = false;
bool sensorsAllOk = false;
String sensorErrorMessage = "";
unsigned long lastSensorDiagnosticMs = 0;
uint32_t currentI2cHz = kI2cFastHz;
int currentI2cSdaPin = kI2cSdaPin;
int currentI2cSclPin = kI2cSclPin;
unsigned long lastMpuRetryMs = 0;

constexpr int kScopeLeft = 8;
constexpr int kScopeTop = 60;
constexpr int kScopeW = 304;
constexpr int kScopeH = 146;
constexpr int kScopeSamples = 304;
constexpr int kScopeSampleIntervalMs = 2;
constexpr int kScopeRawMaxSamples = 4000;
// Scope-only tuning constants (UI visualization path).
// These must not be reused for calibration/balance math pipelines.
constexpr int16_t kScopeHpClamp = 3200;
constexpr int16_t kScopeDrawRange = 2800;
constexpr int16_t kScopeNoiseDeadband = 10;
constexpr int32_t kScopeMotionDeltaThresh = 28;
constexpr unsigned long kScopeMotionHoldMs = 360;
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
int16_t lastLeftAx = 0;
int16_t lastLeftAy = 0;
int16_t lastLeftAz = 0;
int16_t lastRightAx = 0;
int16_t lastRightAy = 0;
int16_t lastRightAz = 0;
float leftMixLp = 0.0f;
float rightMixLp = 0.0f;
unsigned long leftMotionUntilMs = 0;
unsigned long rightMotionUntilMs = 0;
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
int16_t applyDeadband(int16_t v, int16_t deadband);
void drawCalibrationLayoutScreen();
void drawOscLayoutSelectScreen();
void drawCalibrationConfirmScreen();
void drawCalibrationCountdownScreen();
void drawCalibrationMeasuringScreen();
void drawCalibrationSavePromptScreen();
void drawCalibrationDoneScreen(bool saved);
void drawRecordNameScreen();
void drawRecordNameKeyboardScreen();
bool isDiagnosticsScreen(ScreenMode m);
void drawKeyboardNameField();
void drawRecordNameValueBox(int x, int y, int w, int h);
void drawSettingsMenuScreen();
void drawSettingsDateTimeScreen();

void drawDateTimeRow(const char* label, int value, const Rect& minusR, const Rect& plusR, int y);
void drawProceedToRealBalanceScreen();
void drawRealBalanceStubScreen();
void updateCalibrationFlow();
void enterCalibrationLayoutScreen();
void enterOscLayoutSelectScreen();
void enterCalibrationConfirmScreen();
void startCalibrationCountdown();
void startCalibrationMeasuring();
void enterCalibrationSavePrompt();
void startRecordNameFlow();
void enterRecordNameScreen();
void enterRecordNameKeyboardScreen();
void enterProceedToRealBalanceScreen();
void enterRealBalanceStubScreen();
void enterSettingsMenuScreen();
void enterSettingsDateTimeScreen();
void resetZeroAccum();
void saveZeroAccumToBaseline();
void buildDefaultRecordName();
void resetCurrentNameToDefault();
void appendRecordName(const char* letter);
void popRecordNameChar();
bool isCurrentNameEmpty();
void clearRecordName();
void loadSavedZeroBaseline();
void loadManualDateTimeFromNvs();
void loadBuzzerVolumeFromNvs();
void saveBuzzerVolumeToNvs();
void tickManualDateTime();
void formatManualDateTime(char* out, size_t outLen);
void adjustDateTimeField(uint8_t field, int delta);
void diagnoseSensors();
int printI2cScan(const char* label = "[I2C SCAN]");
void applyI2cClock(uint32_t hz);
void enqueueZeroBaselineSave();
void requestBatteryRedraw();
void batteryServiceTick();
void buzzerBeep(uint16_t freq, uint16_t durationMs);

void buzzerBeep(uint16_t freq, uint16_t durationMs) {
  if (buzzerVolumePercent == 0) {
    delay(durationMs);
    return;
  }

  uint32_t duty = (255UL * buzzerVolumePercent) / 100UL;
  if (duty < 2) duty = 2;

  // Passive piezo is more audible with short burst impulses in ~2.6..3.6kHz range.
  uint16_t baseFreq = constrain((int)freq, 2600, 3600);
  if (baseFreq < 2900) baseFreq = 3000;

  uint8_t bursts = (durationMs >= 120) ? 3 : 2;
  uint16_t onMs = max<uint16_t>(8, (durationMs * 4) / (bursts * 5));
  uint16_t offMs = max<uint16_t>(2, (durationMs - (onMs * bursts)) / bursts);

  for (uint8_t i = 0; i < bursts; ++i) {
    uint16_t f = baseFreq + ((i & 1) ? 180 : 0);
    ledcWriteTone(kBuzzerChannel, f);
    ledcWrite(kBuzzerChannel, duty);
    delay(onMs);

    ledcWrite(kBuzzerChannel, 0);
    ledcWriteTone(kBuzzerChannel, 0);

    if (i + 1 < bursts) delay(offMs);
  }
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
  // Always clear the status box before redrawing to avoid stale text.
  tft.fillRect(sensorStatusRect.x, sensorStatusRect.y, sensorStatusRect.w, sensorStatusRect.h, TFT_BLACK);
  tft.setFont(&fonts::efontCN_16);
  
  if (sensorsAllOk) {
    // Всички сензори OK - показвай зелено
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(10, 35);
    tft.println("Сензори свързани и работещи");
  } else {
    // Има проблем - показвай червено и описанието
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 35);
    tft.println("Няма свързани сензори!");
    
    // Показай детайлите на проблема
    if (!sensorErrorMessage.isEmpty()) {
      tft.setFont(&fonts::efontCN_12);
      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(10, 55);
      tft.println(sensorErrorMessage.c_str());
    }
  }
}

void updateHomeSensorWarningIfNeeded() {
  if (currentScreen != ScreenMode::Home) return;

  static bool prevAllOk = false;
  static String prevMsg = "";
  static bool first = true;

  if (first || prevAllOk != sensorsAllOk || prevMsg != sensorErrorMessage) {
    drawNoSensorsWarning();
    prevAllOk = sensorsAllOk;
    prevMsg = sensorErrorMessage;
    first = false;
  }
}

bool pointInRect(uint16_t px, uint16_t py, const Rect& r) {
  return (px >= r.x && px < (r.x + r.w) && py >= r.y && py < (r.y + r.h));
}

bool isDiagnosticsScreen(ScreenMode m) {
  switch (m) {
    case ScreenMode::Home:
    case ScreenMode::SettingsMenu:
    case ScreenMode::SettingsDateTime:
    case ScreenMode::CalibLayoutSelect:
    case ScreenMode::CalibConfirmStart:
    case ScreenMode::CalibSavePrompt:
    case ScreenMode::CalibDone:
    case ScreenMode::CalibRecordName:
    case ScreenMode::CalibRecordNameKeyboard:
    case ScreenMode::CalibProceedToBalance:
    case ScreenMode::RealBalanceStub:
    case ScreenMode::OscLayoutSelect:
      return true;

    // Never run heavy diagnostics in live acquisition screens.
    case ScreenMode::Oscilloscope:
    case ScreenMode::CalibCountdown:
    case ScreenMode::CalibMeasuring:
      return false;
  }

  return false;
}

void drawTwoLineCentered(const char* l1, const char* l2, const Rect& r) {
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);

  int tw1 = tft.textWidth(l1);
  int tw2 = tft.textWidth(l2);
  tft.drawString(l1, r.x + (r.w - tw1) / 2, r.y + 8);
  tft.drawString(l2, r.x + (r.w - tw2) / 2, r.y + 28);
}

void drawLayoutLabelUnderIcon(const Rect& r, const char* l1, const char* l2) {
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);

  int w1 = tft.textWidth(l1);
  int w2 = tft.textWidth(l2);

  int y1 = r.y + r.h - 34;
  int y2 = r.y + r.h - 16;

  tft.drawString(l1, r.x + (r.w - w1) / 2, y1);
  tft.drawString(l2, r.x + (r.w - w2) / 2, y2);
}

void drawSettingsIcon(int x, int y) {
  const uint16_t gearColor = tft.color565(255, 176, 40);
  const uint16_t wrenchColor = tft.color565(70, 216, 245);
  const uint16_t wrenchBg = tft.color565(90, 72, 18);

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

  // Wrench (cyan) with visible jaw gap.
  const int wx = x + 22;
  const int wy = y - 2;
  tft.fillCircle(wx, wy, 7, wrenchColor);
  tft.fillCircle(wx, wy, 4, wrenchBg);
  tft.fillTriangle(wx - 2, wy + 1, wx + 8, wy - 6, wx + 8, wy + 8, wrenchBg);
  for (int d = -1; d <= 1; ++d) {
    tft.drawLine(wx - 3 + d, wy + 4, wx - 14 + d, wy + 15, wrenchColor);
  }
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

  requestBatteryRedraw();
  initStatusIcon();
  drawStatusText();
}

void sampleBatteryState() {
  float vbat = readBatteryVoltage();
  vbat = constrain(vbat, kBatEmpty, kBatFull);
  int percent = ((vbat - kBatEmpty) / (kBatFull - kBatEmpty)) * 100.0f;
  percent = constrain(percent, 0, 100);
  currentBatteryVoltage = vbat;
  currentBatteryPercent = percent;
}

void requestBatteryRedraw() {
  batteryUiDirty = true;
}

void batteryServiceTick() {
  unsigned long now = millis();
  if (now - lastBatUpdate >= kBatUpdateIntervalMs) {
    lastBatUpdate = now;
    sampleBatteryState();
    batteryUiDirty = true;
  }

  if (batteryUiDirty) {
    drawBatteryIcon(currentBatteryVoltage, currentBatteryPercent);
    batteryUiDirty = false;
  }
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

int printI2cScan(const char* label) {
  int found = 0;
  Serial.print(label);
  Serial.print(" Found:");
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.print(" none");
  }
  Serial.println();
  return found;
}

void applyI2cClock(uint32_t hz) {
  Wire.setClock(hz);
  currentI2cHz = hz;
}

int16_t readMpuAxisX(uint8_t addr, bool* ok);

int16_t applyDeadband(int16_t v, int16_t deadband) {
  if (v > -deadband && v < deadband) return 0;
  if (v > 0) return (int16_t)(v - deadband);
  return (int16_t)(v + deadband);
}

bool readMpuWhoAmI(uint8_t addr, uint8_t* whoAmI) {
  *whoAmI = 0;

  Wire.beginTransmission(addr);
  Wire.write(0x75);  // WHO_AM_I
  if (Wire.endTransmission((uint8_t)false) != 0) return false;

  size_t got = Wire.requestFrom((uint8_t)addr, (size_t)1, (bool)true);
  if (got != 1) return false;

  *whoAmI = (uint8_t)Wire.read();
  return true;
}

bool probeMpuStable(uint8_t addr) {
  // Retry a few times because I2C can fail transiently during startup/noise.
  // Important: a sleeping MPU can still answer WHO_AM_I while accel outputs stay zero,
  // so we must try to wake/configure it before accepting it as healthy.
  for (int attempt = 0; attempt < 3; ++attempt) {
    bool initOk = initMpu(addr);
    delay(2);

    uint8_t who = 0;
    bool whoOk = readMpuWhoAmI(addr, &who);

    bool axisOk = false;
    (void)readMpuAxisX(addr, &axisOk);

    bool whoLooksValid = whoOk && (who == 0x68);
    if (initOk && whoLooksValid && axisOk) return true;

    // Re-init and retry before declaring hard failure.
    initMpu(addr);
    delay(2);
  }

  return false;
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

void readMpuAxesXYZ(uint8_t addr, bool* ok, int16_t* ax, int16_t* ay, int16_t* az) {
  *ok = false;
  *ax = 0;
  *ay = 0;
  *az = 0;

  Wire.beginTransmission(addr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission((uint8_t)false) != 0) return;

  size_t got = Wire.requestFrom((uint8_t)addr, (size_t)6, (bool)true);
  if (got != 6) return;

  *ax = ((int16_t)Wire.read() << 8) | Wire.read();
  *ay = ((int16_t)Wire.read() << 8) | Wire.read();
  *az = ((int16_t)Wire.read() << 8) | Wire.read();
  *ok = true;
}

void scopePushSample() {
  // IMPORTANT: this function feeds only oscilloscope rendering/indicators.
  // It is intentionally independent from calibration accumulation and future
  // real-balance math, which must consume raw sensor data paths.
  unsigned long now = millis();
  if (now - lastScopeSampleMs < scopeSampleIntervalMs()) return;
  lastScopeSampleMs = now;

  bool okL = false;
  bool okR = false;
  int16_t leftAx = 0, leftAy = 0, leftAz = 0;
  int16_t rightAx = 0, rightAy = 0, rightAz = 0;
  readMpuAxesXYZ(kMpuLeftAddr, &okL, &leftAx, &leftAy, &leftAz);
  readMpuAxesXYZ(kMpuRightAddr, &okR, &rightAx, &rightAy, &rightAz);

  // 3-axis high-pass motion signal for scope + activity indicators.
  // This reacts to movement in any direction and suppresses static gravity offset.
  int16_t leftRaw = 0;
  int16_t rightRaw = 0;

  // Live status must reflect real read success, not just init writes.
  mpuLeftOk = okL;
  mpuRightOk = okR;
  sensorsAllOk = mpuLeftOk && mpuRightOk && opticalSensorOk;

  // Show real sensor availability; avoid synthetic curves that can mask faults.
  if (!okL) {
    leftRaw = 0;
  } else {
    // Orientation-independent motion metric avoids axis cancellation on smooth moves.
    int32_t mixL = (int32_t)abs((int)leftAx) + (int32_t)abs((int)leftAy) + (int32_t)abs((int)leftAz);
    leftMixLp = leftMixLp * 0.997f + (float)mixL * 0.003f;
    int32_t hpL = mixL - (int32_t)leftMixLp;
    leftRaw = (int16_t)constrain((int)hpL, -kScopeHpClamp, kScopeHpClamp);
    leftRaw = applyDeadband(leftRaw, kScopeNoiseDeadband);
  }

  if (!okR) {
    rightRaw = 0;
  } else {
    int32_t mixR = (int32_t)abs((int)rightAx) + (int32_t)abs((int)rightAy) + (int32_t)abs((int)rightAz);
    rightMixLp = rightMixLp * 0.997f + (float)mixR * 0.003f;
    int32_t hpR = mixR - (int32_t)rightMixLp;
    rightRaw = (int16_t)constrain((int)hpR, -kScopeHpClamp, kScopeHpClamp);
    rightRaw = applyDeadband(rightRaw, kScopeNoiseDeadband);
  }

  // 3-axis motion detection for visual blink indicators.
  // Threshold tuned for machine balancing work to suppress micro-noise.

  if (okL) {
    int32_t dL = abs((int)leftRaw);
    if (dL >= kScopeMotionDeltaThresh) leftMotionUntilMs = now + kScopeMotionHoldMs;
    lastLeftAx = leftAx;
    lastLeftAy = leftAy;
    lastLeftAz = leftAz;
  } else {
    leftMotionUntilMs = 0;
  }

  if (okR) {
    int32_t dR = abs((int)rightRaw);
    if (dR >= kScopeMotionDeltaThresh) rightMotionUntilMs = now + kScopeMotionHoldMs;
    lastRightAx = rightAx;
    lastRightAy = rightAy;
    lastRightAz = rightAz;
  } else {
    rightMotionUntilMs = 0;
  }

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
  const int16_t clampV = constrain(v, (int16_t)-kScopeDrawRange, (int16_t)kScopeDrawRange);
  return kScopeTop + (kScopeH / 2) - ((clampV * (kScopeH / 2 - 2)) / kScopeDrawRange);
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

  // Начална рисунка на точките при влизане в екрана.
  tft.fillCircle(244, 35, 6, mpuLeftOk  ? TFT_GREEN    : TFT_RED);
  tft.fillCircle(258, 35, 6, mpuRightOk ? TFT_YELLOW   : TFT_RED);
  tft.fillCircle(272, 35, 6, TFT_DARKGREY);
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

  // -- Dot radius е 6 (2× от предишните 3) за по-добра видимост --
  constexpr int kDotR = 6;
  constexpr int kDotY = 35;

  // Blink логика за ляв и десен сензор на база 3-осна активност (XYZ).
  static unsigned long lastBlinkMs  = 0;
  static bool          blinkPhase   = false;

  unsigned long nowDot = millis();

  // Blink период ~200ms независим от вибрационната честота.
  if (nowDot - lastBlinkMs >= 200) {
    lastBlinkMs = nowDot;
    blinkPhase  = !blinkPhase;
  }

  bool leftActivity = (nowDot < leftMotionUntilMs);
  bool rightActivity = (nowDot < rightMotionUntilMs);

  // Цвят: червен при грешка, иначе цвят/черен при blink ако има активност.
  uint16_t dotL, dotR;
  if (!mpuLeftOk) {
    dotL = TFT_RED;
  } else if (leftActivity && blinkPhase) {
    dotL = TFT_BLACK;
  } else {
    dotL = TFT_GREEN;
  }

  if (!mpuRightOk) {
    dotR = TFT_RED;
  } else if (rightActivity && blinkPhase) {
    dotR = TFT_BLACK;
  } else {
    dotR = TFT_YELLOW;
  }

  uint16_t dotO = digitalRead(kOpticalPin) ? TFT_CYAN : TFT_DARKGREY;

  tft.fillCircle(244, kDotY, kDotR, dotL);
  tft.fillCircle(258, kDotY, kDotR, dotR);
  tft.fillCircle(272, kDotY, kDotR, dotO);
}

void scopeTryRecoverSensors() {
  unsigned long now = millis();
  if (now - lastMpuRetryMs < 2000) return;
  lastMpuRetryMs = now;

  // Recover only after a real probe; init-only success is not enough.
  if (!mpuLeftOk) mpuLeftOk = probeMpuStable(kMpuLeftAddr);
  if (!mpuRightOk) mpuRightOk = probeMpuStable(kMpuRightAddr);
  sensorsAllOk = mpuLeftOk && mpuRightOk && opticalSensorOk;
}

void diagnoseSensors() {
  unsigned long now = millis();
  if (sensorDiagnosticsDone && (now - lastSensorDiagnosticMs < kSensorDiagIntervalMs)) return;
  lastSensorDiagnosticMs = now;

  // Robust MPU check: WHO_AM_I + axis read with retries/re-init.
  mpuLeftOk = probeMpuStable(kMpuLeftAddr);
  mpuRightOk = probeMpuStable(kMpuRightAddr);

  // If both MPU sensors fail at 400kHz, retry once at 100kHz for noisy/long wiring.
  if (!mpuLeftOk && !mpuRightOk && currentI2cHz != kI2cSafeHz) {
    applyI2cClock(kI2cSafeHz);
    delay(2);
    mpuLeftOk = probeMpuStable(kMpuLeftAddr);
    mpuRightOk = probeMpuStable(kMpuRightAddr);
  }

  uint8_t whoL = 0;
  uint8_t whoR = 0;
  bool whoLOk = readMpuWhoAmI(kMpuLeftAddr, &whoL);
  bool whoROk = readMpuWhoAmI(kMpuRightAddr, &whoR);

  // Test Optical sensor (check if pin is accessible and can read)
  // We consider it OK if we can read the digital pin
  (void)digitalRead(kOpticalPin);
  opticalSensorOk = true;  // GPIO reads always work if pin is configured

  // Overall status
  sensorsAllOk = mpuLeftOk && mpuRightOk && opticalSensorOk;

  // Build error message
  sensorErrorMessage = "";
  if (!mpuLeftOk) {
    sensorErrorMessage += "ЛЯВ ГРЕШКА";
    if (whoLOk) {
      sensorErrorMessage += " (WHO=0x";
      sensorErrorMessage += String(whoL, HEX);
      sensorErrorMessage += ") ";
    } else {
      sensorErrorMessage += " (няма I2C) ";
    }
  }
  if (!mpuRightOk) {
    sensorErrorMessage += "ДЕСЕН ГРЕШКА";
    if (whoROk) {
      sensorErrorMessage += " (WHO=0x";
      sensorErrorMessage += String(whoR, HEX);
      sensorErrorMessage += ") ";
    } else {
      sensorErrorMessage += " (няма I2C) ";
    }
  }
  if (!opticalSensorOk) sensorErrorMessage += "Оптичен сензор - ГРЕШКА! ";

  Serial.printf("[SENSOR DIAG] SDA:%d SCL:%d I2C:%luHz L:%d WHO:%s0x%02X  R:%d WHO:%s0x%02X  ALL:%d\n",
                currentI2cSdaPin,
                currentI2cSclPin,
                (unsigned long)currentI2cHz,
                mpuLeftOk ? 1 : 0,
                whoLOk ? "" : "N/A ",
                whoL,
                mpuRightOk ? 1 : 0,
                whoROk ? "" : "N/A ",
                whoR,
                sensorsAllOk ? 1 : 0);
  if (!sensorsAllOk) {
    int foundMain = printI2cScan("[I2C SCAN]");

    // If nothing is found at all, probe swapped SDA/SCL once and report clearly.
    if (foundMain == 0) {
      Wire.begin(kI2cSclPin, kI2cSdaPin, kI2cSafeHz);
      delay(2);
      int foundSwap = printI2cScan("[I2C SWAP 22/21]");
      Wire.begin(kI2cSdaPin, kI2cSclPin, currentI2cHz);
      delay(2);

      if (foundSwap > 0) {
        sensorErrorMessage += " Провери разменени SDA/SCL.";
      } else {
        sensorErrorMessage += " Няма I2C устройства: провери 3.3V, GND и pull-up на SDA/SCL.";
      }
    }
  }

  sensorDiagnosticsDone = true;
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
  zeroBaseline.layout = selectedLayoutForCalib;
  zeroBaseline.lX = zeroLxSum / zeroSamples;
  zeroBaseline.lY = zeroLySum / zeroSamples;
  zeroBaseline.rX = zeroRxSum / zeroSamples;
  zeroBaseline.rY = zeroRySum / zeroSamples;
  zeroBaseline.sampleCount = zeroSamples;
  zeroBaseline.valid = true;
}

void loadSavedZeroBaseline() {
  Preferences prefs;
  if (!prefs.begin("kubol", true)) return;
  if (!prefs.getBool("valid", false)) {
    prefs.end();
    return;
  }
  zeroBaseline.layout      = (SensorLayout)prefs.getUChar("layout", 0);
  zeroBaseline.lX          = prefs.getFloat("lX", 0.0f);
  zeroBaseline.lY          = prefs.getFloat("lY", 0.0f);
  zeroBaseline.rX          = prefs.getFloat("rX", 0.0f);
  zeroBaseline.rY          = prefs.getFloat("rY", 0.0f);
  zeroBaseline.sampleCount = prefs.getULong("samples", 0);
  zeroBaseline.valid       = true;
  prefs.getString("recName", currentRecordName, kMaxRecordNameLen + 1);
  prefs.end();
}

void loadManualDateTimeFromNvs() {
  Preferences prefs;
  if (!prefs.begin("kubol", true)) return;

  int y = prefs.getInt("dtYear", 0);
  int m = prefs.getInt("dtMonth", 0);
  int d = prefs.getInt("dtDay", 0);
  int hh = prefs.getInt("dtHour", 0);
  int mm = prefs.getInt("dtMin", 0);
  int ss = prefs.getInt("dtSec", 0);

  if (y >= 2024 && y <= 2099 && m >= 1 && m <= 12 && d >= 1 && d <= 31 && hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0 && ss <= 59) {
    manualDateTime.year = y;
    manualDateTime.month = m;
    manualDateTime.day = d;
    manualDateTime.hour = hh;
    manualDateTime.minute = mm;
    manualDateTime.second = ss;
    prefs.end();
    return;
  }

  int mon = 1;
  char monStr[4] = {0};
  int day = 1;
  int year = 2026;
  int hour = 0;
  int minute = 0;
  sscanf(__DATE__, "%3s %d %d", monStr, &day, &year);
  sscanf(__TIME__, "%d:%d", &hour, &minute);
  if (strcmp(monStr, "Jan") == 0) mon = 1;
  else if (strcmp(monStr, "Feb") == 0) mon = 2;
  else if (strcmp(monStr, "Mar") == 0) mon = 3;
  else if (strcmp(monStr, "Apr") == 0) mon = 4;
  else if (strcmp(monStr, "May") == 0) mon = 5;
  else if (strcmp(monStr, "Jun") == 0) mon = 6;
  else if (strcmp(monStr, "Jul") == 0) mon = 7;
  else if (strcmp(monStr, "Aug") == 0) mon = 8;
  else if (strcmp(monStr, "Sep") == 0) mon = 9;
  else if (strcmp(monStr, "Oct") == 0) mon = 10;
  else if (strcmp(monStr, "Nov") == 0) mon = 11;
  else if (strcmp(monStr, "Dec") == 0) mon = 12;

  manualDateTime.year = year;
  manualDateTime.month = mon;
  manualDateTime.day = day;
  manualDateTime.hour = hour;
  manualDateTime.minute = minute;
  manualDateTime.second = 0;
  prefs.end();
}

void saveManualDateTimeToNvs() {
  Preferences prefs;
  if (!prefs.begin("kubol", false)) return;
  prefs.putInt("dtYear", manualDateTime.year);
  prefs.putInt("dtMonth", manualDateTime.month);
  prefs.putInt("dtDay", manualDateTime.day);
  prefs.putInt("dtHour", manualDateTime.hour);
  prefs.putInt("dtMin", manualDateTime.minute);
  prefs.putInt("dtSec", manualDateTime.second);
  prefs.end();
}

void loadBuzzerVolumeFromNvs() {
  Preferences prefs;
  if (!prefs.begin("kubol", false)) return;
  int loaded = 80;
  if (prefs.isKey("buzzVol")) {
    loaded = prefs.getUChar("buzzVol", 80);
  } else {
    prefs.putUChar("buzzVol", 80);
  }
  prefs.end();
  buzzerVolumePercent = constrain(loaded, 0, 100);
}

void saveBuzzerVolumeToNvs() {
  Preferences prefs;
  if (!prefs.begin("kubol", false)) return;
  prefs.putUChar("buzzVol", buzzerVolumePercent);
  prefs.end();
}

static bool isLeapYear(int year) {
  if ((year % 400) == 0) return true;
  if ((year % 100) == 0) return false;
  return (year % 4) == 0;
}

static int daysInMonth(int year, int month) {
  static const int kDays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (month == 2) return isLeapYear(year) ? 29 : 28;
  return kDays[month - 1];
}

void tickManualDateTime() {
  unsigned long now = millis();
  if (lastDateTickMs == 0) {
    lastDateTickMs = now;
    return;
  }

  while (now - lastDateTickMs >= 1000) {
    lastDateTickMs += 1000;
    manualDateTime.second++;
    if (manualDateTime.second < 60) continue;
    manualDateTime.second = 0;
    manualDateTime.minute++;
    if (manualDateTime.minute < 60) continue;
    manualDateTime.minute = 0;
    manualDateTime.hour++;
    if (manualDateTime.hour < 24) continue;
    manualDateTime.hour = 0;
    manualDateTime.day++;
    int dim = daysInMonth(manualDateTime.year, manualDateTime.month);
    if (manualDateTime.day <= dim) continue;
    manualDateTime.day = 1;
    manualDateTime.month++;
    if (manualDateTime.month <= 12) continue;
    manualDateTime.month = 1;
    manualDateTime.year++;
    if (manualDateTime.year > 2099) manualDateTime.year = 2024;
  }
}

void formatManualDateTime(char* out, size_t outLen) {
  snprintf(out, outLen, "%02d-%02d-%04d_%02d-%02d",
           manualDateTime.day, manualDateTime.month, manualDateTime.year,
           manualDateTime.hour, manualDateTime.minute);
}

void adjustDateTimeField(uint8_t field, int delta) {
  if (field == 0) {
    manualDateTime.year += delta;
    if (manualDateTime.year < 2024) manualDateTime.year = 2099;
    if (manualDateTime.year > 2099) manualDateTime.year = 2024;
  } else if (field == 1) {
    manualDateTime.month += delta;
    if (manualDateTime.month < 1) manualDateTime.month = 12;
    if (manualDateTime.month > 12) manualDateTime.month = 1;
  } else if (field == 2) {
    manualDateTime.day += delta;
  } else if (field == 3) {
    manualDateTime.hour += delta;
    if (manualDateTime.hour < 0) manualDateTime.hour = 23;
    if (manualDateTime.hour > 23) manualDateTime.hour = 0;
  } else if (field == 4) {
    manualDateTime.minute += delta;
    if (manualDateTime.minute < 0) manualDateTime.minute = 59;
    if (manualDateTime.minute > 59) manualDateTime.minute = 0;
  }

  int dim = daysInMonth(manualDateTime.year, manualDateTime.month);
  if (manualDateTime.day < 1) manualDateTime.day = dim;
  if (manualDateTime.day > dim) manualDateTime.day = 1;
}

void enqueueZeroBaselineSave() {
  if (gSaveQueue == nullptr) return;
  SaveRequest req;
  req.layout = zeroBaseline.layout;
  req.lX = zeroBaseline.lX;
  req.lY = zeroBaseline.lY;
  req.rX = zeroBaseline.rX;
  req.rY = zeroBaseline.rY;
  req.sampleCount = zeroBaseline.sampleCount;
  req.valid = zeroBaseline.valid;
  strncpy(req.name, currentRecordName, kMaxRecordNameLen);
  req.name[kMaxRecordNameLen] = '\0';
  xQueueSend(gSaveQueue, &req, 0);  // non-blocking: drop if saving still busy
}

void buildDefaultRecordName() {
  formatManualDateTime(defaultRecordName, sizeof(defaultRecordName));
}

void resetCurrentNameToDefault() {
  strncpy(currentRecordName, defaultRecordName, kMaxRecordNameLen);
  currentRecordName[kMaxRecordNameLen] = '\0';
}

bool isCurrentNameEmpty() {
  return currentRecordName[0] == '\0';
}

void appendRecordName(const char* letter) {
  size_t curLen = strlen(currentRecordName);
  size_t addLen = strlen(letter);
  if (curLen + addLen > kMaxRecordNameLen) return;
  strcat(currentRecordName, letter);
}

void clearRecordName() {
  currentRecordName[0] = '\0';
}

void popRecordNameChar() {
  size_t len = strlen(currentRecordName);
  if (len == 0) return;

  // UTF-8 backspace: step back over continuation bytes and cut one codepoint.
  size_t i = len - 1;
  while (i > 0 && (((unsigned char)currentRecordName[i] & 0xC0) == 0x80)) {
    i--;
  }
  currentRecordName[i] = '\0';
}

void drawLayoutSensorIcon(const Rect& r, bool oppositeMode) {
  // Lighter backgrounds for better contrast
  uint16_t bg = oppositeMode ? tft.color565(60, 88, 128) : tft.color565(42, 115, 68);
  tft.fillRoundRect(r.x, r.y, r.w, r.h, 10, bg);
  tft.drawRoundRect(r.x, r.y, r.w, r.h, 10, TFT_WHITE);

  const int cx     = r.x + r.w / 2;
  const int shaftY = r.y + 35;
  const int shaftH = 6;
  const int sensW  = 18;
  const int sensH  = 36;
  const int sensY  = shaftY - sensH / 2;
  const int lsx    = r.x + 8;
  const int rsx    = r.x + r.w - 8 - sensW;
  const int cableH = 10;
  
  // Colors: wheel = white, shaft/arms = dark grey
  const uint16_t wheelCol = TFT_WHITE;
  const uint16_t shaftCol = tft.color565(60, 60, 60);  // dark grey
  const uint16_t hubCol   = TFT_WHITE;
  
  // Both modes now use I-beam wheel design
  const int topCapW = 30, topCapH = 8;
  const int botCapW = 30, botCapH = 8;
  const int neckW   = 9;
  const int hubW    = 20, hubH = 20;
  const int d       = 5;   // 3-D perspective depth
  
  const uint16_t dkGrn = tft.color565(0,   130, 0);
  const uint16_t dkYlw = tft.color565(180, 140, 0);

  const int topCapY  = r.y + 4;
  const int neckTopY = topCapY + topCapH;
  const int neckTopH = (shaftY - hubH/2) - neckTopY;
  const int neckBotY = shaftY + hubH/2;
  const int botCapY  = r.y + 58;
  const int neckBotH = botCapY - neckBotY;

  // Draw I-beam wheel (white)
  tft.fillRect(cx - topCapW/2, topCapY, topCapW, topCapH, wheelCol);
  tft.fillRect(cx - neckW/2, neckTopY, neckW, neckTopH, wheelCol);
  tft.fillRect(cx - neckW/2, neckBotY, neckW, neckBotH, wheelCol);
  tft.fillRect(cx - botCapW/2, botCapY, botCapW, botCapH, wheelCol);
  
  // Horizontal shaft (dark grey)
  tft.fillRect(r.x + 8, shaftY - shaftH/2, r.w - 16, shaftH, shaftCol);
  
  // Center hub (white, over shaft)
  tft.fillRect(cx - hubW/2, shaftY - hubH/2, hubW, hubH, hubCol);

  if (!oppositeMode) {
    // Same-plane mode: clean flat sensor blocks for clearer distinction.
    tft.fillRect(lsx, sensY, sensW, sensH, TFT_GREEN);
    tft.fillRect(rsx, sensY, sensW, sensH, TFT_YELLOW);
  } else {
    // Opposite mode: keep 3D shading to visually differentiate layout.
    tft.fillTriangle(lsx,             sensY,
                     lsx + sensW,     sensY,
                     lsx + sensW + d, sensY - d, dkGrn);
    tft.fillTriangle(lsx,             sensY,
                     lsx + sensW + d, sensY - d,
                     lsx + d,         sensY - d, dkGrn);
    tft.fillTriangle(lsx + sensW,     sensY,
                     lsx + sensW,     sensY + sensH,
                     lsx + sensW + d, sensY + sensH - d, dkGrn);
    tft.fillTriangle(lsx + sensW,     sensY,
                     lsx + sensW + d, sensY + sensH - d,
                     lsx + sensW + d, sensY - d, dkGrn);
    tft.fillRect(lsx, sensY, sensW, sensH, TFT_GREEN);

    tft.fillTriangle(rsx,             sensY,
                     rsx + sensW,     sensY,
                     rsx + sensW - d, sensY - d, dkYlw);
    tft.fillTriangle(rsx,             sensY,
                     rsx + sensW - d, sensY - d,
                     rsx - d,         sensY - d, dkYlw);
    tft.fillTriangle(rsx,             sensY,
                     rsx,             sensY + sensH,
                     rsx - d,         sensY + sensH - d, dkYlw);
    tft.fillTriangle(rsx,             sensY,
                     rsx - d,         sensY + sensH - d,
                     rsx - d,         sensY - d, dkYlw);
    tft.fillRect(rsx, sensY, sensW, sensH, TFT_YELLOW);
  }

  tft.fillRect(lsx + sensW/2 - 1, sensY + sensH, 3, cableH, TFT_BLACK);
  tft.fillRect(rsx + sensW/2 - 1, sensY + sensH, 3, cableH, TFT_BLACK);
}

void drawOscLayoutSelectScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.fillRoundRect(btnScopeBackRect.x, btnScopeBackRect.y, btnScopeBackRect.w,
                    btnScopeBackRect.h, 6, tft.color565(30, 72, 112));
  tft.drawRoundRect(btnScopeBackRect.x, btnScopeBackRect.y, btnScopeBackRect.w,
                    btnScopeBackRect.h, 6, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Назад", btnScopeBackRect.x + 10, btnScopeBackRect.y + 4);

  // Center the title
  tft.setFont(&fonts::efontCN_16);
  int titleW = tft.textWidth("Изберете разположение");
  tft.drawString("Изберете разположение", (kScreenW - titleW) / 2, 58);
  int subtitleW = tft.textWidth("на сензорите");
  tft.drawString("на сензорите", (kScreenW - subtitleW) / 2, 76);

  drawLayoutSensorIcon(btnOscLayoutSameRect, false);
  drawLayoutSensorIcon(btnOscLayoutOppRect, true);

  if (hasLayoutSelectionForOsc) {
    Rect selectedRect = (selectedLayoutForOsc == SensorLayout::SameDirection) ? btnOscLayoutSameRect : btnOscLayoutOppRect;
    tft.drawRoundRect(selectedRect.x - 2, selectedRect.y - 2, selectedRect.w + 4, selectedRect.h + 4, 10, TFT_CYAN);
  }

  drawLayoutLabelUnderIcon(btnOscLayoutSameRect, "Една", "равнина");
  drawLayoutLabelUnderIcon(btnOscLayoutOppRect, "Срещуположно", "един срещу друг");
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

  // Center the title
  tft.setFont(&fonts::efontCN_16);
  int titleW = tft.textWidth("Изберете разположение");
  tft.drawString("Изберете разположение", (kScreenW - titleW) / 2, 58);
  int subtitleW = tft.textWidth("на сензорите");
  tft.drawString("на сензорите", (kScreenW - subtitleW) / 2, 76);

  drawLayoutSensorIcon(btnLayoutSameRect, false);
  drawLayoutSensorIcon(btnLayoutOppRect, true);

  if (hasLayoutSelectionForCalib) {
    Rect selectedRect = (selectedLayoutForCalib == SensorLayout::SameDirection) ? btnLayoutSameRect : btnLayoutOppRect;
    tft.drawRoundRect(selectedRect.x - 2, selectedRect.y - 2, selectedRect.w + 4, selectedRect.h + 4, 10, TFT_CYAN);
  }

  drawLayoutLabelUnderIcon(btnLayoutSameRect, "Една", "равнина");
  drawLayoutLabelUnderIcon(btnLayoutOppRect, "Срещуположно", "един срещу друг");
}

void drawCalibrationConfirmScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Започване нулево", 68, 66);
  tft.drawString("измерване?", 102, 86);

  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Машината трябва да е в покой.", 34, 112);

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
  tft.drawString("Всичко да бъде в покой!", 34, 102);
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
    tft.drawString("Нулевите данни са записани!", 34, 84);
    tft.drawString("Готово", 130, 108);
  } else {
    tft.setTextColor(TFT_YELLOW);
    tft.drawString("Процесът приключи.", 70, 84);
    tft.drawString("Записът е пропуснат.", 66, 108);
  }
}

void drawRecordNameScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Име на записа", 90, 50);

  drawRecordNameValueBox(16, 78, 288, 46);

  tft.fillRoundRect(btnNameManualRect.x, btnNameManualRect.y, btnNameManualRect.w,
                    btnNameManualRect.h, 8, tft.color565(34, 78, 106));
  tft.drawRoundRect(btnNameManualRect.x, btnNameManualRect.y, btnNameManualRect.w,
                    btnNameManualRect.h, 8, TFT_WHITE);
  tft.fillRoundRect(btnNameDefaultRect.x, btnNameDefaultRect.y, btnNameDefaultRect.w,
                    btnNameDefaultRect.h, 8, tft.color565(40, 98, 56));
  tft.drawRoundRect(btnNameDefaultRect.x, btnNameDefaultRect.y, btnNameDefaultRect.w,
                    btnNameDefaultRect.h, 8, TFT_WHITE);
  tft.fillRoundRect(btnNameContinueRect.x, btnNameContinueRect.y, btnNameContinueRect.w,
                    btnNameContinueRect.h, 8, tft.color565(86, 64, 22));
  tft.drawRoundRect(btnNameContinueRect.x, btnNameContinueRect.y, btnNameContinueRect.w,
                    btnNameContinueRect.h, 8, TFT_WHITE);

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Ръчно име", btnNameManualRect.x + 18, btnNameManualRect.y + 14);
  tft.drawString("По дата/час", btnNameDefaultRect.x + 12, btnNameDefaultRect.y + 14);
  tft.drawString("Потвърди името", btnNameContinueRect.x + 50, btnNameContinueRect.y + 12);
}

void drawRecordNameValueBox(int x, int y, int w, int h) {
  tft.fillRect(x + 1, y + 1, w - 2, h - 2, TFT_BLACK);
  tft.drawRoundRect(x, y, w, h, 8, TFT_WHITE);

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_CYAN);

  const int textX = x + 6;
  const int textY = y + 14;
  const int maxTextW = w - 12;

  const char* shown = currentRecordName;
  int fullW = tft.textWidth(currentRecordName);
  if (fullW > maxTextW) {
    size_t len = strlen(currentRecordName);
    size_t start = len;
    while (start > 0) {
      start--;
      while (start > 0 && (((unsigned char)currentRecordName[start] & 0xC0) == 0x80)) {
        start--;
      }
      if (tft.textWidth(currentRecordName + start) <= maxTextW) {
        shown = currentRecordName + start;
        break;
      }
    }
  }

  tft.drawString(shown, textX, textY);
}

void drawKeyboardNameField() {
  drawRecordNameValueBox(8, 62, 304, 34);
}

void drawRecordNameKeyboardScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.fillRoundRect(btnKeyDoneRect.x, btnKeyDoneRect.y, btnKeyDoneRect.w,
                    btnKeyDoneRect.h, 6, tft.color565(36, 78, 108));
  tft.drawRoundRect(btnKeyDoneRect.x, btnKeyDoneRect.y, btnKeyDoneRect.w,
                    btnKeyDoneRect.h, 6, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Готово", btnKeyDoneRect.x + 2, btnKeyDoneRect.y + 4);

  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Въведи име", 104, 36);

  drawKeyboardNameField();

  const int startX = 5;
  const int startY = 104;
  const int keyW = 28;
  const int keyH = 28;
  const int gapX = 3;
  const int gapY = 4;

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  for (int r = 0; r < kKbRows; ++r) {
    for (int c = 0; c < kKbCols; ++c) {
      int x = startX + c * (keyW + gapX);
      int y = startY + r * (keyH + gapY);
      kbKeyRects[r][c] = {x, y, keyW, keyH};
      tft.fillRoundRect(x, y, keyW, keyH, 4, tft.color565(52, 52, 64));
      tft.drawRoundRect(x, y, keyW, keyH, 4, TFT_LIGHTGREY);
      tft.drawString(kbKeyLabels[r][c], x + 7, y + 7);
    }
  }

  tft.fillRoundRect(kbBackspaceRect.x, kbBackspaceRect.y, kbBackspaceRect.w,
                    kbBackspaceRect.h, 6, tft.color565(92, 50, 46));
  tft.drawRoundRect(kbBackspaceRect.x, kbBackspaceRect.y, kbBackspaceRect.w,
                    kbBackspaceRect.h, 6, TFT_WHITE);
  tft.fillRoundRect(kbSpaceRect.x, kbSpaceRect.y, kbSpaceRect.w, kbSpaceRect.h, 6,
                    tft.color565(46, 72, 102));
  tft.drawRoundRect(kbSpaceRect.x, kbSpaceRect.y, kbSpaceRect.w, kbSpaceRect.h, 6,
                    TFT_WHITE);
  tft.fillRoundRect(kbOkRect.x, kbOkRect.y, kbOkRect.w, kbOkRect.h, 6,
                    tft.color565(34, 108, 56));
  tft.drawRoundRect(kbOkRect.x, kbOkRect.y, kbOkRect.w, kbOkRect.h, 6, TFT_WHITE);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Изтрий", kbBackspaceRect.x + 16, kbBackspaceRect.y + 7);
  tft.drawString("Интервал", kbSpaceRect.x + 10, kbSpaceRect.y + 7);
  tft.drawString("ОК", kbOkRect.x + 36, kbOkRect.y + 7);
}

void drawSettingsMenuScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.fillRoundRect(btnSettingsBackRect.x, btnSettingsBackRect.y, btnSettingsBackRect.w,
                    btnSettingsBackRect.h, 6, tft.color565(30, 72, 112));
  tft.drawRoundRect(btnSettingsBackRect.x, btnSettingsBackRect.y, btnSettingsBackRect.w,
                    btnSettingsBackRect.h, 6, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Назад", btnSettingsBackRect.x + 10, btnSettingsBackRect.y + 4);

  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Настройки", 116, 42);

  tft.fillRoundRect(btnSettingsDateTimeRect.x, btnSettingsDateTimeRect.y,
                    btnSettingsDateTimeRect.w, btnSettingsDateTimeRect.h, 8,
                    tft.color565(24, 72, 92));
  tft.drawRoundRect(btnSettingsDateTimeRect.x, btnSettingsDateTimeRect.y,
                    btnSettingsDateTimeRect.w, btnSettingsDateTimeRect.h, 8, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Дата и час", 34, 92);

  char dtBuf[28];
  formatManualDateTime(dtBuf, sizeof(dtBuf));
  tft.setFont(&fonts::efontCN_12);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(dtBuf, 168, 96);

  tft.fillRoundRect(btnSettingsBuzzerRect.x, btnSettingsBuzzerRect.y,
                    btnSettingsBuzzerRect.w, btnSettingsBuzzerRect.h, 8,
                    tft.color565(76, 58, 22));
  tft.drawRoundRect(btnSettingsBuzzerRect.x, btnSettingsBuzzerRect.y,
                    btnSettingsBuzzerRect.w, btnSettingsBuzzerRect.h, 8, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Сила на зумер", 34, 152);

  tft.fillRoundRect(btnBuzzerMinusRect.x, btnBuzzerMinusRect.y, btnBuzzerMinusRect.w,
                    btnBuzzerMinusRect.h, 4, tft.color565(92, 46, 46));
  tft.drawRoundRect(btnBuzzerMinusRect.x, btnBuzzerMinusRect.y, btnBuzzerMinusRect.w,
                    btnBuzzerMinusRect.h, 4, TFT_WHITE);
  tft.fillRoundRect(btnBuzzerPlusRect.x, btnBuzzerPlusRect.y, btnBuzzerPlusRect.w,
                    btnBuzzerPlusRect.h, 4, tft.color565(30, 92, 52));
  tft.drawRoundRect(btnBuzzerPlusRect.x, btnBuzzerPlusRect.y, btnBuzzerPlusRect.w,
                    btnBuzzerPlusRect.h, 4, TFT_WHITE);
  tft.drawString("-", btnBuzzerMinusRect.x + 12, btnBuzzerMinusRect.y + 5);
  tft.drawString("+", btnBuzzerPlusRect.x + 10, btnBuzzerPlusRect.y + 5);

  char volBuf[12];
  sprintf(volBuf, "%d%%", buzzerVolumePercent);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(volBuf, 174, 152);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("(тук ще добавим още опции)", 56, 204);
}

void drawDateTimeRow(const char* label, int value, const Rect& minusR, const Rect& plusR, int y) {
  tft.fillRoundRect(minusR.x, minusR.y, minusR.w, minusR.h, 4, tft.color565(90, 46, 46));
  tft.drawRoundRect(minusR.x, minusR.y, minusR.w, minusR.h, 4, TFT_WHITE);
  tft.fillRoundRect(plusR.x, plusR.y, plusR.w, plusR.h, 4, tft.color565(30, 92, 52));
  tft.drawRoundRect(plusR.x, plusR.y, plusR.w, plusR.h, 4, TFT_WHITE);

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("-", minusR.x + 12, minusR.y + 6);
  tft.drawString("+", plusR.x + 10, plusR.y + 6);

  char buf[24];
  sprintf(buf, "%s: %02d", label, value);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(buf, 62, y);
}

void drawSettingsDateTimeScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.fillRoundRect(btnDateTimeBackRect.x, btnDateTimeBackRect.y, btnDateTimeBackRect.w,
                    btnDateTimeBackRect.h, 6, tft.color565(30, 72, 112));
  tft.drawRoundRect(btnDateTimeBackRect.x, btnDateTimeBackRect.y, btnDateTimeBackRect.w,
                    btnDateTimeBackRect.h, 6, TFT_WHITE);
  tft.fillRoundRect(btnDateTimeSaveRect.x, btnDateTimeSaveRect.y, btnDateTimeSaveRect.w,
                    btnDateTimeSaveRect.h, 6, tft.color565(30, 98, 52));
  tft.drawRoundRect(btnDateTimeSaveRect.x, btnDateTimeSaveRect.y, btnDateTimeSaveRect.w,
                    btnDateTimeSaveRect.h, 6, TFT_WHITE);

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Назад", btnDateTimeBackRect.x + 10, btnDateTimeBackRect.y + 4);
  tft.drawString("Запази", btnDateTimeSaveRect.x + 6, btnDateTimeSaveRect.y + 4);
  tft.drawString("Ръчна настройка дата/час", 50, 52);

  drawDateTimeRow("Година", manualDateTime.year % 100, btnYearMinusRect, btnYearPlusRect, 84);
  drawDateTimeRow("Месец", manualDateTime.month, btnMonthMinusRect, btnMonthPlusRect, 118);
  drawDateTimeRow("Ден", manualDateTime.day, btnDayMinusRect, btnDayPlusRect, 152);
  drawDateTimeRow("Час", manualDateTime.hour, btnHourMinusRect, btnHourPlusRect, 186);
  drawDateTimeRow("Минута", manualDateTime.minute, btnMinuteMinusRect, btnMinutePlusRect, 216);
}

void drawProceedToRealBalanceScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Преминаване към", 84, 72);
  tft.drawString("реален БАЛАНС?", 84, 92);

  tft.setFont(&fonts::efontCN_12);
  tft.drawString(currentRecordName, 28, 122);

  tft.fillRoundRect(btnCalibYesRect.x, btnCalibYesRect.y, btnCalibYesRect.w, btnCalibYesRect.h,
                    8, tft.color565(28, 112, 58));
  tft.drawRoundRect(btnCalibYesRect.x, btnCalibYesRect.y, btnCalibYesRect.w, btnCalibYesRect.h,
                    8, TFT_WHITE);
  tft.fillRoundRect(btnCalibNoRect.x, btnCalibNoRect.y, btnCalibNoRect.w, btnCalibNoRect.h,
                    8, tft.color565(92, 46, 46));
  tft.drawRoundRect(btnCalibNoRect.x, btnCalibNoRect.y, btnCalibNoRect.w, btnCalibNoRect.h,
                    8, TFT_WHITE);

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("ДА", btnCalibYesRect.x + 38, btnCalibYesRect.y + 12);
  tft.drawString("НЕ", btnCalibNoRect.x + 38, btnCalibNoRect.y + 12);
}

void drawRealBalanceStubScreen() {
  tft.fillScreen(TFT_BLACK);
  drawTopBar();

  tft.setFont(&fonts::efontCN_16);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Реален БАЛАНС", 90, 78);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("(предстои разработка)", 52, 102);
  tft.setFont(&fonts::efontCN_12);
  tft.drawString(currentRecordName, 34, 130);

  tft.fillRoundRect(btnRealBalBackHomeRect.x, btnRealBalBackHomeRect.y,
                    btnRealBalBackHomeRect.w, btnRealBalBackHomeRect.h, 8,
                    tft.color565(30, 72, 112));
  tft.drawRoundRect(btnRealBalBackHomeRect.x, btnRealBalBackHomeRect.y,
                    btnRealBalBackHomeRect.w, btnRealBalBackHomeRect.h, 8, TFT_WHITE);
  tft.setFont(&fonts::efontCN_16);
  tft.drawString("Към начало", btnRealBalBackHomeRect.x + 34, btnRealBalBackHomeRect.y + 10);
}

void enterCalibrationLayoutScreen() {
  currentScreen = ScreenMode::CalibLayoutSelect;
  hasLayoutSelectionForCalib = false;
  drawCalibrationLayoutScreen();
}

void enterOscLayoutSelectScreen() {
  currentScreen = ScreenMode::OscLayoutSelect;
  hasLayoutSelectionForOsc = false;
  drawOscLayoutSelectScreen();
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

void startRecordNameFlow() {
  buildDefaultRecordName();
  resetCurrentNameToDefault();
  enterRecordNameScreen();
}

void enterRecordNameScreen() {
  currentScreen = ScreenMode::CalibRecordName;
  drawRecordNameScreen();
}

void enterRecordNameKeyboardScreen() {
  currentScreen = ScreenMode::CalibRecordNameKeyboard;
  drawRecordNameKeyboardScreen();
}

void enterProceedToRealBalanceScreen() {
  currentScreen = ScreenMode::CalibProceedToBalance;
  enqueueZeroBaselineSave();
  drawProceedToRealBalanceScreen();
}

void enterRealBalanceStubScreen() {
  currentScreen = ScreenMode::RealBalanceStub;
  drawRealBalanceStubScreen();
}

void enterSettingsMenuScreen() {
  currentScreen = ScreenMode::SettingsMenu;
  drawSettingsMenuScreen();
}

void enterSettingsDateTimeScreen() {
  currentScreen = ScreenMode::SettingsDateTime;
  drawSettingsDateTimeScreen();
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
      uint16_t bg = calibBlinkOn ? TFT_RED : TFT_WHITE;
      uint16_t fg = calibBlinkOn ? TFT_WHITE : TFT_RED;
      tft.fillRect(0, 138, kScreenW, 28, bg);
      tft.setFont(&fonts::efontCN_16);
      tft.setTextColor(fg);
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
    // Calibration path uses raw XY readings for zero baseline accumulation.
    // Do not apply scope deadband/high-pass tuning here.
    bool okL = false;
    bool okR = false;
    int16_t lx = 0, ly = 0, rx = 0, ry = 0;
    readMpuAxesXY(kMpuLeftAddr, &okL, &lx, &ly);
    readMpuAxesXY(kMpuRightAddr, &okR, &rx, &ry);

    mpuLeftOk = okL;
    mpuRightOk = okR;
    sensorsAllOk = mpuLeftOk && mpuRightOk && opticalSensorOk;

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
  // Reset transient oscilloscope interaction state when leaving live screen.
  scopePaused = false;
  lastScopeDrawMs = 0;
  lastScopeSampleMs = 0;
  currentScreen = ScreenMode::Home;
  drawStartupScreen();
}

void enterOscilloscopeScreen() {
  // Always enter oscilloscope in running mode to avoid stale paused state.
  scopePaused = false;
  lastScopeDrawMs = 0;
  lastScopeSampleMs = 0;
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
        requestBatteryRedraw();
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
  unsigned long debounceMs = (currentScreen == ScreenMode::CalibRecordNameKeyboard) ? 120 : 180;
  if (now - lastUiTouchMs < debounceMs) return;
  lastUiTouchMs = now;

  if (currentScreen == ScreenMode::Home) {
    if (pointInRect(x, y, sensorStatusRect)) {
      diagnoseSensors();
      drawStartupScreen();
      if (!sensorsAllOk) {
        // Ръчен refresh на диагностиката и индикация за грешка.
        buzzerBeep(800, 100);
      } else {
        buzzerBeep(3000, 40);
      }
      return;
    }
    if (pointInRect(x, y, btnCalibRect)) {
      if (!sensorsAllOk) {
        buzzerBeep(1200, 150);  // Звук за грешка
        return;
      }
      enterCalibrationLayoutScreen();
      return;
    }
    if (pointInRect(x, y, btnSettingsRect)) {
      enterSettingsMenuScreen();
      return;
    }
    if (pointInRect(x, y, btnScopeRect)) {
      enterOscLayoutSelectScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::Oscilloscope) {
    if (pointInRect(x, y, btnScopeBackRect)) {
      enterHomeScreen();
      return;
    }

    Rect sliderTouch = {scopeSliderRect.x - 4, scopeSliderRect.y - 8,
                        scopeSliderRect.w + 8, scopeSliderRect.h + 16};
    if (pointInRect(x, y, sliderTouch)) {
      int relX = (int)x - scopeSliderRect.x;
      relX = constrain(relX, 0, scopeSliderRect.w - 1);
      scopeSliderValue = (relX * 100) / (scopeSliderRect.w - 1);
      // Slider interaction should never pause live acquisition.
      scopePaused = false;

      tft.fillRect(scopeSliderRect.x - 6, scopeSliderRect.y - 6,
                   scopeSliderRect.w + 12, 14, TFT_BLACK);
      drawScopeTimeSlider();
      return;
    }

    // Plot tap toggles pause/resume (kept after slider handling to avoid conflicts).
    if (pointInRect(x, y, scopePlotRect)) {
      scopePaused = !scopePaused;
      return;
    }
  } else if (currentScreen == ScreenMode::CalibLayoutSelect) {
    if (pointInRect(x, y, btnCalibBackRect)) {
      enterHomeScreen();
      return;
    }
    if (pointInRect(x, y, btnLayoutSameRect)) {
      selectedLayoutForCalib = SensorLayout::SameDirection;
      hasLayoutSelectionForCalib = true;
      enterCalibrationConfirmScreen();
      return;
    }
    if (pointInRect(x, y, btnLayoutOppRect)) {
      selectedLayoutForCalib = SensorLayout::OppositeFacing;
      hasLayoutSelectionForCalib = true;
      enterCalibrationConfirmScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::OscLayoutSelect) {
    if (pointInRect(x, y, btnScopeBackRect)) {
      enterHomeScreen();
      return;
    }
    if (pointInRect(x, y, btnOscLayoutSameRect)) {
      selectedLayoutForOsc = SensorLayout::SameDirection;
      hasLayoutSelectionForOsc = true;
      enterOscilloscopeScreen();
      return;
    }
    if (pointInRect(x, y, btnOscLayoutOppRect)) {
      selectedLayoutForOsc = SensorLayout::OppositeFacing;
      hasLayoutSelectionForOsc = true;
      enterOscilloscopeScreen();
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
      startRecordNameFlow();
      return;
    }
    if (pointInRect(x, y, btnCalibNoRect)) {
      currentScreen = ScreenMode::CalibDone;
      calibPhaseStartMs = millis();
      drawCalibrationDoneScreen(false);
      return;
    }
  } else if (currentScreen == ScreenMode::CalibRecordName) {
    if (pointInRect(x, y, btnNameManualRect)) {
      enterRecordNameKeyboardScreen();
      return;
    }
    if (pointInRect(x, y, btnNameDefaultRect)) {
      resetCurrentNameToDefault();
      drawRecordNameScreen();
      return;
    }
    if (pointInRect(x, y, btnNameContinueRect)) {
      if (isCurrentNameEmpty()) resetCurrentNameToDefault();
      enterProceedToRealBalanceScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::CalibRecordNameKeyboard) {
    if (pointInRect(x, y, btnKeyDoneRect)) {
      if (isCurrentNameEmpty()) resetCurrentNameToDefault();
      enterRecordNameScreen();
      return;
    }

    for (int r = 0; r < kKbRows; ++r) {
      for (int c = 0; c < kKbCols; ++c) {
        if (pointInRect(x, y, kbKeyRects[r][c])) {
          appendRecordName(kbKeyLabels[r][c]);
          drawKeyboardNameField();
          return;
        }
      }
    }

    if (pointInRect(x, y, kbBackspaceRect)) {
      popRecordNameChar();
      drawKeyboardNameField();
      return;
    }
    if (pointInRect(x, y, kbSpaceRect)) {
      appendRecordName(" ");
      drawKeyboardNameField();
      return;
    }
    if (pointInRect(x, y, kbOkRect)) {
      if (isCurrentNameEmpty()) resetCurrentNameToDefault();
      enterRecordNameScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::CalibProceedToBalance) {
    if (pointInRect(x, y, btnCalibNoRect)) {
      enterHomeScreen();
      return;
    }
    if (pointInRect(x, y, btnCalibYesRect)) {
      enterRealBalanceStubScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::CalibDone) {
    enterHomeScreen();
    return;
  } else if (currentScreen == ScreenMode::RealBalanceStub) {
    if (pointInRect(x, y, btnRealBalBackHomeRect)) {
      enterHomeScreen();
      return;
    }
  } else if (currentScreen == ScreenMode::SettingsMenu) {
    if (pointInRect(x, y, btnSettingsBackRect)) {
      enterHomeScreen();
      return;
    }
    if (pointInRect(x, y, btnSettingsDateTimeRect)) {
      enterSettingsDateTimeScreen();
      return;
    }
    if (pointInRect(x, y, btnBuzzerMinusRect)) {
      if (buzzerVolumePercent >= 5) buzzerVolumePercent -= 5;
      else buzzerVolumePercent = 0;
      saveBuzzerVolumeToNvs();
      drawSettingsMenuScreen();
      buzzerBeep(1400, 20);
      return;
    }
    if (pointInRect(x, y, btnBuzzerPlusRect)) {
      if (buzzerVolumePercent <= 95) buzzerVolumePercent += 5;
      else buzzerVolumePercent = 100;
      saveBuzzerVolumeToNvs();
      drawSettingsMenuScreen();
      buzzerBeep(1600, 20);
      return;
    }
  } else if (currentScreen == ScreenMode::SettingsDateTime) {
    if (pointInRect(x, y, btnDateTimeBackRect)) {
      enterSettingsMenuScreen();
      return;
    }
    if (pointInRect(x, y, btnDateTimeSaveRect)) {
      saveManualDateTimeToNvs();
      enterSettingsMenuScreen();
      return;
    }

    if (pointInRect(x, y, btnYearMinusRect)) adjustDateTimeField(0, -1);
    else if (pointInRect(x, y, btnYearPlusRect)) adjustDateTimeField(0, 1);
    else if (pointInRect(x, y, btnMonthMinusRect)) adjustDateTimeField(1, -1);
    else if (pointInRect(x, y, btnMonthPlusRect)) adjustDateTimeField(1, 1);
    else if (pointInRect(x, y, btnDayMinusRect)) adjustDateTimeField(2, -1);
    else if (pointInRect(x, y, btnDayPlusRect)) adjustDateTimeField(2, 1);
    else if (pointInRect(x, y, btnHourMinusRect)) adjustDateTimeField(3, -1);
    else if (pointInRect(x, y, btnHourPlusRect)) adjustDateTimeField(3, 1);
    else if (pointInRect(x, y, btnMinuteMinusRect)) adjustDateTimeField(4, -1);
    else if (pointInRect(x, y, btnMinutePlusRect)) adjustDateTimeField(4, 1);
    else return;

    drawSettingsDateTimeScreen();
    return;
  }
}

void updateBatteryPeriodic() {
  batteryServiceTick();
}
}  // namespace ui

// Runs on Core 0 (PRO_CPU). Blocks waiting for save requests so it never
// touches flash during active sensor reads on Core 1 (APP_CPU).
void saveTaskFunc(void* param) {
  Preferences prefs;
  ui::SaveRequest req;
  for (;;) {
    if (xQueueReceive(gSaveQueue, &req, portMAX_DELAY) == pdTRUE) {
      prefs.begin("kubol", false);
      prefs.putUChar("layout",   (uint8_t)req.layout);
      prefs.putFloat("lX",       req.lX);
      prefs.putFloat("lY",       req.lY);
      prefs.putFloat("rX",       req.rX);
      prefs.putFloat("rY",       req.rY);
      prefs.putULong("samples",  req.sampleCount);
      prefs.putBool("valid",     req.valid);
      prefs.putString("recName", req.name);
      prefs.end();
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ui::kHwStartStopPin, INPUT_PULLUP);

  ledcSetup(ui::kBuzzerChannel, 2000, 8);
  ledcAttachPin(ui::kBuzzerPin, ui::kBuzzerChannel);
  ledcWriteTone(ui::kBuzzerChannel, 0);

  pinMode(ui::kOpticalPin, INPUT_PULLUP);

  // I2C Bus Recovery: освобождава заседнал slave преди Wire.begin().
  // Изпраща 9 CLK импулса + STOP condition за да отключи блокирана шина.
  pinMode(ui::kI2cSdaPin, OUTPUT);  digitalWrite(ui::kI2cSdaPin, HIGH);
  pinMode(ui::kI2cSclPin, OUTPUT);  digitalWrite(ui::kI2cSclPin, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(ui::kI2cSclPin, LOW);  delayMicroseconds(5);
    digitalWrite(ui::kI2cSclPin, HIGH); delayMicroseconds(5);
  }
  // STOP condition: SDA low -> SDA high докато SCL е high
  digitalWrite(ui::kI2cSdaPin, LOW);  delayMicroseconds(5);
  digitalWrite(ui::kI2cSclPin, HIGH); delayMicroseconds(5);
  digitalWrite(ui::kI2cSdaPin, HIGH); delayMicroseconds(5);
  pinMode(ui::kI2cSdaPin, INPUT_PULLUP);
  pinMode(ui::kI2cSclPin, INPUT_PULLUP);
  delay(10);

  Wire.begin(ui::kI2cSdaPin, ui::kI2cSclPin, ui::kI2cFastHz);
  Wire.setTimeOut(8);
  ui::currentI2cHz = ui::kI2cFastHz;
  ui::currentI2cSdaPin = ui::kI2cSdaPin;
  ui::currentI2cSclPin = ui::kI2cSclPin;
  ui::mpuLeftOk = false;
  ui::mpuRightOk = false;
  ui::sensorsAllOk = false;

  tft.init();
  tft.setRotation(3);  // Landscape: 320x240

  ui::sampleBatteryState();
  ui::lastBatUpdate = millis();
  ui::batteryUiDirty = true;

  // Non-volatile storage: save task pinned to Core 0, out of sensor/UI path.
  gSaveQueue = xQueueCreate(3, sizeof(ui::SaveRequest));
  xTaskCreatePinnedToCore(saveTaskFunc, "SaveTask", 8192, nullptr, 1, nullptr, 0);
  ui::loadSavedZeroBaseline();
  ui::loadManualDateTimeFromNvs();
  ui::loadBuzzerVolumeFromNvs();
  ui::buildDefaultRecordName();

  ui::drawStartupScreen();

  // Диагностика на сензорите (след първото рисуване на UI),
  // за да не изглежда уредът блокирал при тежка I2C повреда.
  delay(1000);
  ui::diagnoseSensors();
  ui::drawStartupScreen();
}

void loop() {
  ui::tickManualDateTime();

  // Run diagnostics only in static screens to avoid adding latency/jitter
  // to live sampling and calibration acquisition.
  if (ui::isDiagnosticsScreen(ui::currentScreen)) {
    ui::diagnoseSensors();
  }

  ui::updateHomeSensorWarningIfNeeded();

  ui::handleUiTouch();
  ui::updateStatusIcon();
  ui::drawStatusText();
  ui::handleBatteryTouchToggle();
  ui::updateBatteryPeriodic();

  if (ui::currentScreen == ui::ScreenMode::Oscilloscope) {
    ui::scopeTryRecoverSensors();
    if (!ui::scopePaused) {
      ui::scopePushSample();
      ui::drawOscilloscopeWaveforms();
    }
  }

  ui::updateCalibrationFlow();

  delay(10);
}
