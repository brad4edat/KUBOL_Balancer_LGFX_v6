#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <time.h>
#include <Preferences.h>

#include "LGFX_Config.h"

LGFX tft;

static QueueHandle_t gSaveQueue = nullptr;

// ============================================================
// namespace ui вЂ” state, constants, forward declarations, impl
// Each ui_impl_*.h file is included here (ONCE) as an
// implementation partition.  They are NOT standalone headers.
// ============================================================
namespace ui {

// ---- Hardware constants ------------------------------------
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
constexpr unsigned long kBatUpdateIntervalMs = 30000;
constexpr float kBatDividerCoeff = 3.162f;

// ---- Types (enums / structs) --------------------------------
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
  SensorVisual = 15,
};

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

// ВНИМАНИЕ: Оси/знаци/въртене между левия и десния MPU6050 се определят
// единствено по SENSOR_LAYOUT_SPEC_BG.md (финална операторска спецификация).
// При нови измервания: първо update на спецификацията, после промени по логиката.
enum class SensorLayout : uint8_t {
  VerticalSameDirection = 0,
  VerticalOppositeFacing = 1,
  HorizontalSameDirection = 2,
  HorizontalOppositeFacing = 3,
};

enum class SensorMountSide : uint8_t {
  Left = 0,
  Right = 1,
};

struct SensorAxesXYZ {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct SaveRequest {
  SensorLayout layout;
  float lX;
  float lY;
  float lZ;
  float rX;
  float rY;
  float rZ;
  uint32_t sampleCount;
  bool valid;
  char name[32];  // kMaxRecordNameLen + 1
};

struct ZeroBaselineData {
  SensorLayout layout;
  float lX;
  float lY;
  float lZ;
  float rX;
  float rY;
  float rZ;
  uint32_t sampleCount;
  bool valid;
};

// ---- Global state variables ---------------------------------
unsigned long lastBatUpdate = 0;
unsigned long lastTouchDraw = 0;
unsigned long lastAcceptedTouchMs = 0;
bool touchWasDown = false;
constexpr unsigned long kGlobalTouchDebounceMs = 350;
bool showVoltageInsteadOfPercent = false;
float currentBatteryVoltage = kBatFull;
int currentBatteryPercent = 100;
bool batteryUiDirty = true;
bool realBalForceRedraw = false;

bool isBalancing = false;
int statusAngle = 0;
unsigned long lastStatusUpdate = 0;

ScreenMode currentScreen = ScreenMode::Home;

// ---- Button Rect variables ----------------------------------
Rect btnCalibRect = {0, 0, 0, 0};
Rect btnMeasureRect = {0, 0, 0, 0};
Rect btnSettingsRect = {0, 0, 0, 0};
Rect btnScopeRect = {0, 0, 0, 0};
Rect btnScopeBackRect = {8, 32, 72, 24};
Rect sensorStatusRect = {10, 30, 300, 60};
Rect scopeSliderRect = {122, 52, 186, 4};
Rect scopeZoomInRect  = {251, 186, 60, 19};
Rect scopeZoomOutRect = {251, 165, 60, 19};
int scopeSliderValue = 30;
int scopeZoomIndex = 0;
bool scopePaused = false;
bool scopeForceRedraw = false;

Rect btnLayoutVertSameRect = {18, 92, 136, 56};
Rect btnLayoutVertOppRect = {166, 92, 136, 56};
Rect btnLayoutHorizSameRect = {18, 156, 136, 56};
Rect btnLayoutHorizOppRect = {166, 156, 136, 56};
Rect btnOscLayoutVertSameRect = {18, 92, 136, 56};
Rect btnOscLayoutVertOppRect = {166, 92, 136, 56};
Rect btnOscLayoutHorizSameRect = {18, 156, 136, 56};
Rect btnOscLayoutHorizOppRect = {166, 156, 136, 56};
Rect btnCalibYesRect = {38, 168, 110, 46};
Rect btnCalibNoRect = {172, 168, 110, 46};
Rect btnCalibBackRect = {8, 28, 72, 24};
Rect btnNameManualRect = {18, 138, 136, 46};
Rect btnNameDefaultRect = {166, 138, 136, 46};
Rect btnNameContinueRect = {38, 192, 244, 40};
Rect btnKeyDoneRect = {254, 32, 58, 24};
Rect btnRealBalCleanDataRect = {18, 118, 284, 34};
Rect btnRealBalNoCleanDataRect = {18, 156, 284, 34};
Rect btnRealBalFutureRect = {18, 194, 284, 34};
Rect btnRealBalBackHomeRect = {8, 28, 72, 24};
Rect btnSettingsBackRect = {8, 28, 72, 24};
Rect btnSettingsDateTimeRect = {18, 76, 284, 52};
Rect btnSettingsBuzzerRect = {18, 136, 284, 52};
Rect btnBuzzerMinusRect = {214, 148, 36, 28};
Rect btnBuzzerPlusRect = {258, 148, 36, 28};
Rect btnSettingsScrollRect = {304, 76, 10, 154};
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
Rect btnSettingsSensorVisRect = {18, 194, 284, 34};
Rect btnSensorVisBackRect = {8, 28, 72, 24};
Rect btnSensorVisLayoutVertSameRect = {4, 54, 152, 24};
Rect btnSensorVisLayoutVertOppRect = {160, 54, 156, 24};
Rect btnSensorVisLayoutHorizSameRect = {4, 82, 152, 24};
Rect btnSensorVisLayoutHorizOppRect = {160, 82, 156, 24};

// ---- Keyboard layout ----------------------------------------
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
Rect kbBackspaceRect = {10, 200, 94, 30};
Rect kbSpaceRect = {110, 200, 100, 30};
Rect kbOkRect = {216, 200, 94, 30};

// ---- Date/time & misc state ---------------------------------
DateTimeData manualDateTime = {2026, 3, 11, 12, 0, 0};
unsigned long lastDateTickMs = 0;
uint8_t buzzerVolumePercent = 80;
int settingsMenuScrollPx = 0;

// ---- Layout selection state ---------------------------------
SensorLayout selectedLayoutForCalib = SensorLayout::VerticalSameDirection;
bool hasLayoutSelectionForCalib = false;
SensorLayout selectedLayoutForOsc = SensorLayout::VerticalSameDirection;
bool hasLayoutSelectionForOsc = false;
SensorLayout sensorVisLayout = SensorLayout::VerticalSameDirection;
float sensorVisLp[2][3] = {};
float sensorVisGravLp[2][3] = {};
unsigned long lastSensorVisDrawMs = 0;

// ---- Calibration state --------------------------------------
unsigned long calibPhaseStartMs = 0;
unsigned long lastCalibBlinkMs = 0;
bool calibBlinkOn = false;
int lastCountdownBeepSecond = -1;

float zeroLxSum = 0.0f;
float zeroLySum = 0.0f;
float zeroLzSum = 0.0f;
float zeroRxSum = 0.0f;
float zeroRySum = 0.0f;
float zeroRzSum = 0.0f;
uint32_t zeroSamples = 0;

ZeroBaselineData zeroBaseline = {SensorLayout::VerticalSameDirection, 0, 0, 0, 0, 0, 0, 0, false};

// ---- Sensor state -------------------------------------------
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

// ---- Oscilloscope constants & state -------------------------
constexpr int kScopeLeft = 8;
constexpr int kScopeTop = 60;
constexpr int kScopeW = 304;
constexpr int kScopeH = 146;
constexpr int kScopeSamples = 304;
constexpr int kScopeSampleIntervalMs = 2;
constexpr int kScopeRawMaxSamples = 4000;
// Scope-only tuning вЂ” must NOT be reused for balance math.
constexpr int16_t kScopeHpClamp = 3200;
constexpr int16_t kScopeDrawRange = 2800;
constexpr uint8_t kScopeZoomLevels[] = {1, 2, 4, 8, 10, 12, 14};
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
int16_t lastLeftAx = 0, lastLeftAy = 0, lastLeftAz = 0;
int16_t lastRightAx = 0, lastRightAy = 0, lastRightAz = 0;
float leftScopeLpX = 0.0f;
float leftScopeLpY = 0.0f;
float leftScopeLpZ = 0.0f;
float rightScopeLpX = 0.0f;
float rightScopeLpY = 0.0f;
float rightScopeLpZ = 0.0f;
unsigned long leftMotionUntilMs = 0;
unsigned long rightMotionUntilMs = 0;
unsigned long lastScopeDrawMs = 0;
unsigned long lastScopeSampleMs = 0;
LGFX_Sprite scopeSprite(&tft);
bool scopeSpriteReady = false;

// ---- Status icon constants ----------------------------------
constexpr int kStatusIconX = 150;
constexpr int kStatusIconY = 12;
constexpr int kStatusIconRadius = 10;
constexpr int kStatusDotRadius = 2;
constexpr int kStatusTextX = 165;
constexpr int kStatusTextY = 6;

// ---- Forward declarations -----------------------------------
void drawStartupScreen();
unsigned long scopeSampleIntervalMs();
int scopeWindowMs();
unsigned long scopeDrawIntervalMs();
void drawScopeTimeSlider();
void rebuildScopeFromRawWindow();
void drawScopeRpmValue();
void drawScopeZoomControls();
int scopeCurrentDrawRange();
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
SensorAxesXYZ normalizeMeasurementAxesXYZ(SensorLayout layout, SensorMountSide side, int16_t ax, int16_t ay, int16_t az);
SensorAxesXYZ normalizeScopeAxesXYZ(SensorLayout layout, SensorMountSide side, int16_t ax, int16_t ay, int16_t az);
SensorAxesXYZ normalizeLayoutAxesXYZ(SensorLayout layout, SensorMountSide side, int16_t ax, int16_t ay, int16_t az);
int16_t readMpuAxisX(uint8_t addr, bool* ok);
void readMpuAxesXYZ(uint8_t addr, bool* ok, int16_t* ax, int16_t* ay, int16_t* az);
void initStatusIcon();
void drawStatusText();
void drawTopBar();
void drawMainMenuButtons();
void drawNoSensorsWarning();
void updateHomeSensorWarningIfNeeded();
bool pointInRect(uint16_t px, uint16_t py, const Rect& r);
void drawOscilloscopeScreenStatic();
void drawOscilloscopeWaveforms(bool forceDraw = false);
void scopePushSample();
void scopeTryRecoverSensors();
void enterHomeScreen();
void enterOscilloscopeScreen();
void enterSensorVisScreen();
void drawSensorVisScreen();
void updateSensorVisScreen();
bool takeDebouncedTouch(uint16_t* x, uint16_t* y);
void handleBatteryTouchToggle();
void handleUiTouch();
void updateBatteryPeriodic();

// ---- Implementation partitions (included in dependency order)
#include "ui_impl_battery.h"
#include "ui_impl_sensors.h"
#include "ui_impl_scope.h"
#include "ui_impl_storage.h"
#include "ui_impl_screens.h"
#include "ui_impl_navigation.h"
#include "ui_impl_touch.h"

}  // namespace ui

// ============================================================
// FreeRTOS save task (Core 0) вЂ” writes NVS off the UI path
// ============================================================
void saveTaskFunc(void* param) {
  Preferences prefs;
  ui::SaveRequest req;
  for (;;) {
    if (xQueueReceive(gSaveQueue, &req, portMAX_DELAY) == pdTRUE) {
      prefs.begin("kubol", false);
      prefs.putUChar("layout",   (uint8_t)req.layout);
      prefs.putFloat("lX",       req.lX);
      prefs.putFloat("lY",       req.lY);
      prefs.putFloat("lZ",       req.lZ);
      prefs.putFloat("rX",       req.rX);
      prefs.putFloat("rY",       req.rY);
      prefs.putFloat("rZ",       req.rZ);
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

  // I2C Bus Recovery: РѕСЃРІРѕР±РѕР¶РґР°РІР° Р·Р°СЃРµРґРЅР°Р» slave РїСЂРµРґРё Wire.begin().
  pinMode(ui::kI2cSdaPin, OUTPUT);  digitalWrite(ui::kI2cSdaPin, HIGH);
  pinMode(ui::kI2cSclPin, OUTPUT);  digitalWrite(ui::kI2cSclPin, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(ui::kI2cSclPin, LOW);  delayMicroseconds(5);
    digitalWrite(ui::kI2cSclPin, HIGH); delayMicroseconds(5);
  }
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
  tft.setRotation(3);

  ui::sampleBatteryState();
  ui::lastBatUpdate = millis();
  ui::batteryUiDirty = true;

  gSaveQueue = xQueueCreate(3, sizeof(ui::SaveRequest));
  xTaskCreatePinnedToCore(saveTaskFunc, "SaveTask", 8192, nullptr, 1, nullptr, 0);
  ui::loadSavedZeroBaseline();
  ui::loadManualDateTimeFromNvs();
  ui::loadBuzzerVolumeFromNvs();
  ui::buildDefaultRecordName();

  ui::drawStartupScreen();

  delay(1000);
  ui::diagnoseSensors();
  ui::drawStartupScreen();
}

void loop() {
  ui::tickManualDateTime();

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
      ui::scopeForceRedraw = false;
    } else if (ui::scopeForceRedraw) {
      ui::drawOscilloscopeWaveforms(true);
      ui::scopeForceRedraw = false;
    }
  }

  ui::updateCalibrationFlow();

  if (ui::currentScreen == ui::ScreenMode::SensorVisual) {
    ui::updateSensorVisScreen();
  }

  delay(10);
}
