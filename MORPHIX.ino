/*
  ============================================================
  MORPHIX — Smartwatch UI Prototype
  Seeed XIAO ESP32-S3 + 1.8" ST7735 TFT (128x160 panel, 160x128 landscape)
  ============================================================

  This sketch renders the MORPHIX watch UI. Temperature/pressure
  (BMP280), heart rate/SpO2 (MAX30102), and motion/gestures/steps
  (MPU6050) are all live, sensor-driven data — see sections 6/6b/6c.
  Phone connectivity (notifications, phone link, time sync) is also
  real now, via a BLE GATT server the Android companion app connects
  to — see section "PHONE BLE BRIDGE" below. That server exposes two
  parallel command layers on the same GATT service: the original binary
  protocol (CHAR_NOTIFICATION_TX/COMMAND_RX/etc, fragmented notification
  packets), and a newer plain-text ASCII layer (CHAR_ASCII_CMD/
  CHAR_ASCII_RESP — PING/GET_STATUS/GET_SENSORS/GET_DEVICE_INFO/GET_TIME/
  CLEAR_NOTIFICATIONS/SYNC_REQUEST/MARK_READ=<id>) for simpler phone-side
  tooling. Both read the same live sensor/notification state. What's still mocked (no
  matching hardware exists yet, genuinely out of scope): the on-screen
  clock (no RTC) and battery percentage (no fuel gauge) — see
  mockHour/mockBatteryPct. Notification timestamps and Phone Link's
  LAST SYNC use the phone's real time once TIME_SYNC has run at least
  once; before that they honestly show "--:--" rather than the mock
  clock.

  Screens: HOME, ENVIRONMENT, HEALTH, ACTIVITY, NOTIFICATIONS,
  CONNECTIVITY (WIFI / BLUETOOTH / PHONE LINK), MOTION (MOTION DATA /
  GESTURES / MOTION SETTINGS), SETTINGS (DISPLAY / SENSORS / TIME /
  DEVICE STATUS / ABOUT).

  Wiring (confirmed, do not change):
    TFT VCC   -> 3V3
    TFT GND   -> GND
    TFT LED   -> 3V3
    TFT SCK   -> D8
    TFT SDA   -> D10
    TFT A0/DC -> D1
    TFT RESET -> D0
    TFT CS    -> D2

  Buttons (PHYSICALLY WIRED AND TESTED — GPIO isolation test confirmed
  each pin reads HIGH when idle and LOW when pressed; see PIN_UP /
  PIN_DOWN / PIN_BACK / PIN_SELECT below):
    UP     -> D9
    DOWN   -> D3
    BACK   -> D6
    SELECT -> D7
  All buttons wired active-LOW with INPUT_PULLUP, sharing a common GND rail.

  IMU / motion sensor: MPU6050 (address 0x68), sharing the same I2C
  bus as BMP280/MAX30102 (SDA=D4, SCL=D5). Raw-register access via
  Wire is used instead of the Adafruit_MPU6050 library, which
  previously caused initialization trouble on this bus. See section
  6c for the driver, filtering, gesture engine, step counter, and
  raise-to-wake logic.
*/

// ============================================================
// 1. INCLUDES
// ============================================================
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <Preferences.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <time.h>
#include <sys/time.h>

// ===== MORPHIX STORAGE =====
// JPEG decode (image viewer + MJPEG/AVI video player, see below) and the
// local Wi-Fi file manager web server. TJpg_Decoder is NOT part of the
// esp32 board package and must be installed once via Library Manager
// ("TJpg_Decoder" by Bodmer) — WebServer.h ships with the esp32 core
// already, same as WiFi.h/BLEDevice.h above, no extra install needed.
#include <TJpg_Decoder.h>
#include <WebServer.h>
#include <new>   // std::nothrow, used by the JPEG contain-fit resample buffers below

// ============================================================
// WI-FI / NTP / REAL-TIME CLOCK CONFIGURATION
// ============================================================
// Fill these in with your own network's (or phone hotspot's) credentials
// before flashing. Never commit real credentials here — these are
// placeholders on purpose. The password is never printed to Serial or
// drawn on the TFT.
#define WIFI_SSID     "Walid ko bolo recharge krwaye"
#define WIFI_PASSWORD "Kyu chahiye hotspot?"

// NTP servers used for time sync once Wi-Fi/internet is available.
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.google.com"

// Asia/Kolkata is UTC+05:30 with no daylight saving — POSIX TZ string
// form (note the sign is inverted from the UTC offset by POSIX convention).
#define MORPHIX_TZ "IST-5:30"

// ------------------------------------------------------------
// Forward declarations (struct + functions that take it by
// reference). The Arduino IDE auto-generates function prototypes
// and inserts them near the TOP of the file, before the real
// "struct StabilityFilter { ... };" definition further down —
// without these lines that auto-generated prototype references an
// undefined type and the sketch fails to compile ("StabilityFilter
// was not declared in this scope"). Declaring the struct name here
// (incomplete type is fine for a reference-parameter prototype) and
// hand-writing these three prototypes makes the IDE skip generating
// its own conflicting ones.
// ------------------------------------------------------------
struct StabilityFilter;
void resetStabilityFilter(StabilityFilter &f);
void feedStabilityFilter(StabilityFilter &f, int candidate, int consistencyDelta, unsigned long now);
void checkStabilityTimeout(StabilityFilter &f, unsigned long now, unsigned long staleTimeoutMs);

// Same issue applies to the Button struct used by buttonPressed() below.
struct Button;
bool buttonPressed(Button &b);

// Same issue again for the Storage subsystem's StorageFileKind enum:
// functions taking/returning it are defined well below its enum
// definition, and the IDE's auto-generated prototype for those
// functions gets inserted up here, ahead of that definition, unless
// the enum itself is at least forward-declared first. An enum forward
// declaration requires a fixed underlying type in C++11, so this one
// (and its real definition further down) both specify ": uint8_t".
enum StorageFileKind : uint8_t;

// ============================================================
// TFT PINS
// ============================================================

#define TFT_CS   D2
#define TFT_DC   D1
#define TFT_RST  D0
#define TFT_SCK  D8   // hardware SPI clock (see wiring note at top of file)
#define TFT_MOSI D10  // hardware SPI data (TFT SDA in the wiring note)

// ============================================================
// SD CARD PINS (TFT-board microSD slot, sharing hardware SPI)
// ============================================================
// SD-SCK/SD-MOSI reuse the same physical bus lines as the TFT (normal —
// SCK/MOSI/MISO are meant to be a shared bus, only CS is per-device).
// SD_CS and SD_MISO share GPIOs with TFT_RST (D0) and PIN_UP (D9)
// respectively — per hardware confirmation, the SD card sits behind a
// level-shifted module with its own isolated CS/MISO, so this is safe
// on this build. MISO is only routed onto D9 for the brief window of an
// actual SD transaction (see sdBeginTransaction()/sdEndTransaction()
// below) and immediately handed back to the UP button the rest of the
// time, mirroring the existing MISO-avoidance fix already in this file.
#define SD_CS    D0
#define SD_MISO  D9

// ============================================================
// DISPLAY
// ============================================================

#define SCR_W 160
#define SCR_H 128

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Global redraw flag — declared up here (rather than down in the
// navigation section) because the WiFi/BLE backend functions need to set
// it and are defined earlier in the file than the navigation globals.
bool needsRedraw = true;

// ScreenID/currentScreen — also hoisted up here (out of the NAVIGATION
// section below) for the same reason as needsRedraw above:
// serviceWifiAutoConnect() and other WiFi/BLE backend functions defined
// earlier in the file need to read currentScreen/SCR_WIFI before the
// navigation globals would otherwise be declared.
enum ScreenID {
  SCR_HOME, SCR_ENV, SCR_HEALTH, SCR_ACTIVITY, SCR_NOTIF, SCR_CONNECTIVITY, SCR_MOTION, SCR_SETTINGS, SCR_STORAGE,
  SCR_TOP_COUNT,

  SCR_WIFI = SCR_TOP_COUNT, SCR_BLUETOOTH, SCR_PHONELINK,
  SCR_MOTION_DATA, SCR_GESTURES, SCR_MOTION_SETTINGS,
  SCR_DISPLAY_SETTINGS, SCR_SENSORS_SETTINGS, SCR_TIME_SETTINGS, SCR_DEVICE_STATUS, SCR_ABOUT,
  SCR_COUNT
};
ScreenID currentScreen = SCR_HOME;

// Also hoisted up here for the same reason as needsRedraw/currentScreen
// above: the Storage subsystem's storageSdEndSession() (defined ahead
// of the NAVIGATION section, where this used to live) needs to force a
// full repaint after an SD session, before this file's normal
// declaration point would otherwise be reached.
ScreenID lastRenderedScreen = (ScreenID)-1;

// ============================================================
// 2. PIN DEFINITIONS
// ============================================================
// --- Display: HARDWARE SPI ---
// Your wiring (SCK -> D8, SDA/MOSI -> D10) matches the XIAO ESP32-S3's
// default hardware SPI pins exactly, so we let the SPI peripheral drive
// those two lines instead of bit-banging them in software. This is
// ~10-20x faster than software SPI, which is what was causing the
// visible flicker on every clock update — the clear+redraw now
// completes fast enough to be imperceptible.
// CS/DC/RST are still plain GPIO, controlled directly by the library.
// --- BMP280 ---
#define BMP_SDA    D4
#define BMP_SCL    D5

Adafruit_BMP280 bmp;

bool bmp280Ready = false;

float realTemperatureC = 0.0;
float realPressureHpa = 0.0;

// --- MAX30102 ---
// Shares the SAME I2C bus as the BMP280 (Wire, SDA=D4/SCL=D5) — no
// second Wire instance, no pin changes.
MAX30105 max30102Sensor;

bool max30102Ready      = false;  // sensor.begin() succeeded at startup
bool max30102Contact    = false;  // IR level indicates skin/wrist contact
bool max30102SignalGood = false;  // contact + usable optical signal
bool heartRateValid     = false;  // getStableHeartRate() currently trustworthy
bool spo2Valid          = false;  // getStableSpO2() currently trustworthy
// ============================================================
// 4. BUTTON DEFINITIONS (debounced, edge-triggered)
// ============================================================
// Four tactile buttons are physically wired and have been verified with
// a GPIO isolation test (idle = HIGH on all four, LOW on the pin whose
// button is pressed). They share a common GND rail and use the ESP32's
// internal pull-ups, so no external resistors are needed. These pins
// don't overlap TFT_CS/TFT_DC/TFT_RST (D2/D1/D0), TFT hardware SPI
// (D8/D10), or the BMP280 I2C lines (D4/D5).
#define PIN_UP     D9
#define PIN_DOWN   D3
#define PIN_BACK   D6
#define PIN_SELECT D7

struct Button {
  uint8_t pin;
  bool stableState;            // debounced logical state (true = pressed)
  volatile bool lastReading;   // last raw reading, written by the pin's ISR
  volatile unsigned long lastChangeMs; // millis() at that raw edge, written by the ISR
};
const unsigned long DEBOUNCE_MS = 35; // 30-40ms target range

Button btnUp     = { PIN_UP,     false, false, 0 };
Button btnDown   = { PIN_DOWN,   false, false, 0 };
Button btnBack   = { PIN_BACK,   false, false, 0 };
Button btnSelect = { PIN_SELECT, false, false, 0 };

// ---- Per-pin ISRs: capture the raw edge + timestamp the instant it
// happens, instead of only whenever loop() next gets around to polling
// digitalRead(). This is what actually fixes "press missed / delayed /
// sluggish, especially while the UI is updating": a full-screen TFT
// redraw can block loop() for tens of ms, and a plain polling debounce
// only starts its debounce timer once loop() resumes and notices the
// pin has changed — so a press that started (or even finished) during
// the redraw gets its debounce window pushed back by however long the
// redraw took. The ISR removes that dependency entirely: the edge is
// timestamped in real time no matter what loop() is doing, so
// buttonPressed() sees a debounce window that started when the finger
// actually moved, not when loop() happened to notice.
// IRAM_ATTR is required on ESP32-S3 so the handler stays resident and
// safe to run even if flash (holding non-IRAM code) is mid-operation.
// The handlers do only a digitalRead()+compare+millis() — no Serial,
// no allocation, no drawing — so they stay short, as ISRs must.
void IRAM_ATTR isrButtonUp() {
  bool raw = (digitalRead(PIN_UP) == LOW);
  if (raw != btnUp.lastReading) {
    btnUp.lastReading = raw;
    btnUp.lastChangeMs = millis();
  }
}
void IRAM_ATTR isrButtonDown() {
  bool raw = (digitalRead(PIN_DOWN) == LOW);
  if (raw != btnDown.lastReading) {
    btnDown.lastReading = raw;
    btnDown.lastChangeMs = millis();
  }
}
void IRAM_ATTR isrButtonBack() {
  bool raw = (digitalRead(PIN_BACK) == LOW);
  if (raw != btnBack.lastReading) {
    btnBack.lastReading = raw;
    btnBack.lastChangeMs = millis();
  }
}
void IRAM_ATTR isrButtonSelect() {
  bool raw = (digitalRead(PIN_SELECT) == LOW);
  if (raw != btnSelect.lastReading) {
    btnSelect.lastReading = raw;
    btnSelect.lastChangeMs = millis();
  }
}

// Cheap, non-blocking, event-based: reads only the already-captured
// ISR state (no GPIO access here), applies the millis()-based debounce,
// and returns true exactly once, on the press edge (LOW = pressed). A
// held button still produces exactly one event — stableState only
// flips again on the matching release, which this function ignores.
bool buttonPressed(Button &b) {
  noInterrupts();
  bool raw = b.lastReading;
  unsigned long changeMs = b.lastChangeMs;
  interrupts();

  if ((millis() - changeMs) > DEBOUNCE_MS) {
    if (raw != b.stableState) {
      b.stableState = raw;
      if (b.stableState) return true; // fresh press
    }
  }
  return false;
}

// ============================================================
// 5. COLOR DEFINITIONS (RGB565)
// ============================================================
#define COL_BG          0x0000  // black
#define COL_PANEL       0x1082  // very dim panel fill (near-black)
#define COL_BORDER      0x2965  // subtle border gray-teal
#define COL_PRIMARY     0x07FF  // cyan
#define COL_PRIMARY_DIM 0x0410  // dim teal
#define COL_TEXT        0xFFFF  // white
#define COL_SECONDARY   0x8C71  // gray
#define COL_GOOD        0x07E0  // green
#define COL_WARN        0xFFE0  // yellow
#define COL_CRIT        0xF800  // red

// ============================================================
// 5b. MORPHIX <-> PHONE BLE PROTOCOL CONSTANTS
// ============================================================
// Shared "wire format" between this firmware and the Android companion
// app (see MorphixBle/Protocol.kt on the Android side — field names and
// values are kept identical on purpose). Defined early (ahead of where
// they're first used, e.g. by dismissNotification() below) because these
// are #define macros, not functions — unlike ordinary functions, the
// Arduino IDE does NOT auto-hoist macro definitions, so the #define has
// to physically appear above its first use in the file.
#define PKT_VERSION              1
#define FIRMWARE_VERSION         "1.1"

// Packet types carried over CHAR_NOTIFICATION_TX (phone -> watch)
#define PKT_TYPE_NOTIFICATION    1   // a (possibly fragmented) notification
#define PKT_TYPE_COMMAND         2   // a phone -> watch command (see CMD_*)

// Commands the phone can send to the watch (payload of a PKT_TYPE_COMMAND
// packet: [0]=cmdId [1]=payloadLen [2..]=payload)
#define CMD_PING                 1
#define CMD_STATUS_UPDATE        2   // payload: [0]=flags(bit0=notifAccessEnabled) [1]=batteryPct(0-100,255=unknown)
#define CMD_CLEAR_NOTIFICATIONS  3
#define CMD_MARK_READ            4   // payload: notification id bytes
#define CMD_SYNC_NOTIFICATIONS   5
#define CMD_GET_STATUS           6

// Commands the watch can send to the phone over CHAR_COMMAND_RX
// (format: [0]=PKT_VERSION [1]=cmdId [2]=payloadLen [3..]=payload)
#define WCMD_REQUEST_SYNC          1
#define WCMD_MARK_READ             2   // payload: notification id bytes
#define WCMD_DISMISS_NOTIFICATION  3   // payload: notification id bytes
#define WCMD_PING                  4
#define WCMD_GET_DEVICE_STATUS     5

// ============================================================
// 6. MOCK SENSOR / STATE DATA  (REPLACE LATER with real reads)
// ============================================================
// Daily step goal is a user-configurable target, not sensor data — kept
// as a plain constant (no "mock" prefix; it was never meant to come from
// hardware). Real step COUNT comes from getStepCount() in section 6c.
long dailyStepGoal = 5000;

int   mockBatteryPct = 86;
// mockConnected was removed — the HOME screen's status dot now reflects
// real connectivity (connData.wifiConnected / phoneConnected / bleConnected).

// NOTE: despite the "mock" name (kept so the rest of the UI/draw code
// below doesn't need to change), these are now driven by the real,
// NTP-synced ESP32 system clock — see updateClockData() and the
// "REAL-TIME CLOCK" section above. They only hold placeholder starting
// values until clockHasValidTime becomes true for the first time.
int mockHour = 0, mockMinute = 0, mockSecond = 0;
char mockDay[4]   = "---";
char mockDate[8]  = "--- --";

// ---------- REAL PHONE NOTIFICATION QUEUE (section 6d) ----------
// Replaces the old mock 3-item array. Fixed-size ring buffer, filled by
// the MORPHIX BLE GATT server (see "PHONE BLE BRIDGE" section) from
// notifications the Android companion app forwards from
// NotificationListenerService. No String/heap churn — every field is a
// fixed-size char array sized for a watch-sized display.
#define MAX_NOTIF_QUEUE   15
#define NOTIF_ID_LEN      24
#define NOTIF_APP_LEN     20
#define NOTIF_TITLE_LEN   40
#define NOTIF_TEXT_LEN    90

struct QueuedNotification {
  char id[NOTIF_ID_LEN];       // phone-side notification key, used for MARK_READ/DISMISS mapping
  char app[NOTIF_APP_LEN];     // app name (falls back to package id if phone didn't supply one)
  char title[NOTIF_TITLE_LEN];
  char text[NOTIF_TEXT_LEN];
  uint32_t timestampSec;       // unix seconds; 0 = unknown (no time sync yet when it arrived)
  bool unread;
};

QueuedNotification notifQueue[MAX_NOTIF_QUEUE];
int notifCount = 0;   // number of valid entries in notifQueue[0..notifCount-1], newest first
int notifIndex = 0;   // row currently shown on the NOTIFICATIONS screen

// Called from the ENV/ACTIVITY "SYNC" footer action. BMP280 gets an
// immediate forced re-read. (HEALTH's SYNC is handled separately by
// manualHealthSync() below, since it needs to restart the MAX30102
// acquisition window rather than just re-read a single value.)
bool max30102Measuring = false;

void updateBMP280() {
  if (!bmp280Ready) return;

  realTemperatureC = bmp.readTemperature();
  realPressureHpa = bmp.readPressure() / 100.0F;
}

// ---------- GESTURE SYSTEM (types/settings — declared here, ahead of
// section 6b/6c, since the real gesture engine below needs them; label
// helpers that only the UI needs live later, in section 8) ----------
enum GestureType {
  GESTURE_NONE, GESTURE_TILT_LEFT, GESTURE_TILT_RIGHT,
  GESTURE_TILT_UP, GESTURE_TILT_DOWN, GESTURE_SHAKE,
  GESTURE_TWIST_CW, GESTURE_TWIST_CCW
};

bool gestureControlEnabled = false;  // MOTION SETTINGS toggle
bool motionWakeEnabled     = false;  // MOTION SETTINGS toggle

enum GestureSensitivity { SENS_LOW, SENS_MED, SENS_HIGH };
GestureSensitivity gestureSensitivity = SENS_MED;

// ---------- MOTION DATA (populated live by updateMPU6050() below —
// the MOTION DATA screen and gesture engine only ever read this struct,
// same shape as before, just fed by real reads instead of a sine mock) ----------
struct MotionData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pitch, roll;
};
MotionData motionData = {0, 0, 1.0, 0, 0, 0, 0, 0};

// ============================================================
// 6c. MPU6050 SENSOR LAYER (accel/gyro, gestures, steps, raise-to-wake)
// ============================================================
// Raw-register access over the shared Wire bus (SDA=D4/SCL=D5,
// address 0x68) — the Adafruit_MPU6050 library previously caused
// init trouble on this project, so this talks to the registers
// directly instead. Ranges: accel +-8G, gyro +-500 deg/s, DLPF ~21 Hz.
// Sampled on a fixed millis() interval from loop() (non-blocking).

#define MPU6050_ADDR         0x68
#define MPU6050_REG_PWR_MGMT1  0x6B
#define MPU6050_REG_CONFIG     0x1A  // DLPF
#define MPU6050_REG_GYRO_CFG   0x1B
#define MPU6050_REG_ACCEL_CFG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// +-8G  -> 4096 LSB/g ;  +-500 deg/s -> 65.5 LSB/(deg/s)
#define MPU6050_ACCEL_LSB_PER_G   4096.0f
#define MPU6050_GYRO_LSB_PER_DPS  65.5f

#define MPU_SAMPLE_INTERVAL_MS 15   // ~66 Hz, inside the requested 50-100 Hz band

bool mpu6050Ready = false;

// ---- Raw + filtered motion state ----
struct MotionRawState {
  float accelXg, accelYg, accelZg;      // filtered, in g
  float gyroXdps, gyroYdps, gyroZdps;   // filtered, in deg/s (bias-corrected)
  float accelMag;                       // smoothed |accel| in g
  float gyroMag;                        // smoothed |gyro| in deg/s
  float pitch, roll;                    // degrees, from filtered accel
};
MotionRawState mpuState = {0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0};

// Gyro startup calibration (bias removed so a stationary watch reads ~0 dps).
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
bool  gyroCalibrated = false;

// Exponential low-pass filter coefficients (0..1, higher = more smoothing).
#define ACCEL_LPF_ALPHA 0.25f
#define GYRO_LPF_ALPHA  0.35f

// ---- Motion state (STABLE / MOVING) with hysteresis ----
#define MOTION_STABLE_ACCEL_THRESH 0.06f  // g, deviation from 1g at rest
#define MOTION_STABLE_GYRO_THRESH  6.0f   // deg/s
#define MOTION_MOVING_ACCEL_THRESH 0.12f
#define MOTION_MOVING_GYRO_THRESH  15.0f
#define MOTION_STATE_DEBOUNCE_MS   250

bool motionIsMoving = false;
unsigned long motionStateChangeCandidateMs = 0;
bool motionStateCandidateMoving = false;

bool readMPU6050Raw(int16_t &ax, int16_t &ay, int16_t &az,
                     int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t bytesNeeded = 14; // accel(6) + temp(2) + gyro(6)
  if (Wire.requestFrom((int)MPU6050_ADDR, (int)bytesNeeded) != bytesNeeded) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // discard temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// One-time bring-up: wake the sensor, set DLPF/full-scale ranges. Called
// once from initSensors(). Returns false (mpu6050Ready stays false) if the
// device doesn't ACK, so a missing/failed IMU degrades gracefully instead
// of hanging setup().
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT1);
  Wire.write(0x00); // wake from sleep, use internal oscillator
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_CONFIG);
  Wire.write(0x04); // DLPF_CFG=4 -> ~21 Hz bandwidth
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_GYRO_CFG);
  Wire.write(0x08); // FS_SEL=1 -> +-500 deg/s
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_CFG);
  Wire.write(0x10); // AFS_SEL=2 -> +-8g
  if (Wire.endTransmission() != 0) return false;

  return true;
}

// Averages ~40 raw gyro samples with the watch assumed stationary
// (right after power-up) to find the zero-rate bias. Brief and only
// runs once at startup — not on the main loop's hot path.
void calibrateGyro() {
  const int N = 40;
  long sumX = 0, sumY = 0, sumZ = 0;
  int got = 0;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPU6050Raw(ax, ay, az, gx, gy, gz)) {
      sumX += gx; sumY += gy; sumZ += gz;
      got++;
    }
    delay(5); // brief, startup-only — not inside loop()
  }
  if (got > 0) {
    gyroBiasX = (sumX / (float)got) / MPU6050_GYRO_LSB_PER_DPS;
    gyroBiasY = (sumY / (float)got) / MPU6050_GYRO_LSB_PER_DPS;
    gyroBiasZ = (sumZ / (float)got) / MPU6050_GYRO_LSB_PER_DPS;
    gyroCalibrated = true;
  }
}

// Pulls one fresh sample (called on the MPU_SAMPLE_INTERVAL_MS cadence
// from loop()), applies bias correction + low-pass filtering, and
// updates the STABLE/MOVING motion state with hysteresis + debounce so
// it doesn't flicker right at the threshold.
void updateMPU6050() {
  if (!mpu6050Ready) return;

  int16_t ax, ay, az, gx, gy, gz;
  if (!readMPU6050Raw(ax, ay, az, gx, gy, gz)) return;

  float axg = ax / MPU6050_ACCEL_LSB_PER_G;
  float ayg = ay / MPU6050_ACCEL_LSB_PER_G;
  float azg = az / MPU6050_ACCEL_LSB_PER_G;

  float gxdps = (gx / MPU6050_GYRO_LSB_PER_DPS) - gyroBiasX;
  float gydps = (gy / MPU6050_GYRO_LSB_PER_DPS) - gyroBiasY;
  float gzdps = (gz / MPU6050_GYRO_LSB_PER_DPS) - gyroBiasZ;

  // Low-pass filter accel + gyro so gesture/step logic never sees raw noise.
  mpuState.accelXg = mpuState.accelXg + ACCEL_LPF_ALPHA * (axg - mpuState.accelXg);
  mpuState.accelYg = mpuState.accelYg + ACCEL_LPF_ALPHA * (ayg - mpuState.accelYg);
  mpuState.accelZg = mpuState.accelZg + ACCEL_LPF_ALPHA * (azg - mpuState.accelZg);

  mpuState.gyroXdps = mpuState.gyroXdps + GYRO_LPF_ALPHA * (gxdps - mpuState.gyroXdps);
  mpuState.gyroYdps = mpuState.gyroYdps + GYRO_LPF_ALPHA * (gydps - mpuState.gyroYdps);
  mpuState.gyroZdps = mpuState.gyroZdps + GYRO_LPF_ALPHA * (gzdps - mpuState.gyroZdps);

  mpuState.accelMag = sqrtf(mpuState.accelXg * mpuState.accelXg +
                             mpuState.accelYg * mpuState.accelYg +
                             mpuState.accelZg * mpuState.accelZg);
  mpuState.gyroMag = sqrtf(mpuState.gyroXdps * mpuState.gyroXdps +
                            mpuState.gyroYdps * mpuState.gyroYdps +
                            mpuState.gyroZdps * mpuState.gyroZdps);

  // Pitch/roll from the filtered accelerometer vector (standard tilt formulas).
  mpuState.pitch = atan2f(-mpuState.accelXg,
                           sqrtf(mpuState.accelYg * mpuState.accelYg +
                                 mpuState.accelZg * mpuState.accelZg)) * 180.0f / PI;
  mpuState.roll = atan2f(mpuState.accelYg, mpuState.accelZg) * 180.0f / PI;

  // ---- STABLE / MOVING with hysteresis + debounce ----
  float accelDev = fabsf(mpuState.accelMag - 1.0f);
  bool candidateMoving = motionIsMoving
    ? !(accelDev < MOTION_STABLE_ACCEL_THRESH && mpuState.gyroMag < MOTION_STABLE_GYRO_THRESH)
    : (accelDev > MOTION_MOVING_ACCEL_THRESH || mpuState.gyroMag > MOTION_MOVING_GYRO_THRESH);

  unsigned long now = millis();
  if (candidateMoving != motionStateCandidateMoving) {
    motionStateCandidateMoving = candidateMoving;
    motionStateChangeCandidateMs = now;
  }
  if (candidateMoving != motionIsMoving &&
      (now - motionStateChangeCandidateMs) >= MOTION_STATE_DEBOUNCE_MS) {
    motionIsMoving = candidateMoving;
  }

  // Keep the legacy MotionData struct (drives the MOTION DATA screen) in
  // sync with the real filtered readings — no separate mock path remains.
  motionData.accelX = mpuState.accelXg;
  motionData.accelY = mpuState.accelYg;
  motionData.accelZ = mpuState.accelZg;
  motionData.gyroX  = mpuState.gyroXdps;
  motionData.gyroY  = mpuState.gyroYdps;
  motionData.gyroZ  = mpuState.gyroZdps;
  motionData.pitch  = mpuState.pitch;
  motionData.roll   = mpuState.roll;
}

// ---------------------------------------------------------------
// GESTURE ENGINE (event-based: fires once per gesture, then requires
// a return toward neutral before the same gesture can fire again)
// ---------------------------------------------------------------
#define GESTURE_TILT_ANGLE_DEG     22.0f   // minimum tilt angle to arm a tilt gesture
#define GESTURE_NEUTRAL_ANGLE_DEG  10.0f   // must return under this to re-arm
#define GESTURE_MIN_GYRO_DPS       40.0f   // minimum angular velocity for twist/shake
#define GESTURE_MIN_DURATION_MS    60      // minimum time condition must hold before firing
#define GESTURE_COOLDOWN_MS        500     // minimum gap between any two fired gestures
#define SHAKE_GYRO_THRESH_DPS      220.0f
#define SHAKE_MIN_COUNT            3       // direction reversals within the window
#define SHAKE_WINDOW_MS            600

bool gestureArmed = true; // true = neutral, ready to fire a new gesture
unsigned long gestureConditionStartMs = 0;
int gestureConditionCandidate = GESTURE_NONE;
unsigned long lastGestureFireMsReal = 0;

// Shake detection: counts high-gyro direction reversals on any axis
// within a rolling window, rather than a single-sample spike.
int shakeReversalCount = 0;
unsigned long shakeWindowStartMs = 0;
float lastShakeGyroSign = 0;

int detectGestureCandidate() {
  // Tilt gestures: pitch/roll past threshold.
  if (mpuState.pitch > GESTURE_TILT_ANGLE_DEG)  return GESTURE_TILT_UP;
  if (mpuState.pitch < -GESTURE_TILT_ANGLE_DEG) return GESTURE_TILT_DOWN;
  if (mpuState.roll  > GESTURE_TILT_ANGLE_DEG)  return GESTURE_TILT_RIGHT;
  if (mpuState.roll  < -GESTURE_TILT_ANGLE_DEG) return GESTURE_TILT_LEFT;

  // Twist gestures: sustained yaw-axis angular velocity (gyroZ).
  if (mpuState.gyroZdps > GESTURE_MIN_GYRO_DPS * 2) return GESTURE_TWIST_CW;
  if (mpuState.gyroZdps < -GESTURE_MIN_GYRO_DPS * 2) return GESTURE_TWIST_CCW;

  return GESTURE_NONE;
}

// Real threshold-based gesture detection against the filtered MPU6050
// state. Event-based: a condition must hold for GESTURE_MIN_DURATION_MS,
// the device must be "armed" (near neutral since the last gesture), and
// GESTURE_COOLDOWN_MS must have elapsed since the last fire. Returns
// GESTURE_NONE most calls.
int detectGesture() {
  if (!gestureControlEnabled || !mpu6050Ready) return GESTURE_NONE;
  unsigned long now = millis();

  // ---- Shake: count gyro-sign reversals above threshold within a window ----
  float gz = mpuState.gyroZdps;
  if (fabsf(gz) > SHAKE_GYRO_THRESH_DPS) {
    float sign = (gz > 0) ? 1.0f : -1.0f;
    if (shakeWindowStartMs == 0 || now - shakeWindowStartMs > SHAKE_WINDOW_MS) {
      shakeWindowStartMs = now;
      shakeReversalCount = 0;
      lastShakeGyroSign = sign;
    } else if (sign != lastShakeGyroSign) {
      shakeReversalCount++;
      lastShakeGyroSign = sign;
    }
  }
  if (shakeWindowStartMs != 0 && now - shakeWindowStartMs > SHAKE_WINDOW_MS) {
    shakeWindowStartMs = 0; // window expired, start fresh next time
  }

  bool nearNeutral = (fabsf(mpuState.pitch) < GESTURE_NEUTRAL_ANGLE_DEG &&
                       fabsf(mpuState.roll)  < GESTURE_NEUTRAL_ANGLE_DEG &&
                       mpuState.gyroMag < GESTURE_MIN_GYRO_DPS);
  if (nearNeutral) gestureArmed = true;

  if (!gestureArmed) return GESTURE_NONE;
  if (now - lastGestureFireMsReal < GESTURE_COOLDOWN_MS) return GESTURE_NONE;

  if (shakeReversalCount >= SHAKE_MIN_COUNT) {
    shakeReversalCount = 0;
    shakeWindowStartMs = 0;
    gestureArmed = false;
    lastGestureFireMsReal = now;
    gestureConditionCandidate = GESTURE_NONE;
    return GESTURE_SHAKE;
  }

  int candidate = detectGestureCandidate();
  if (candidate == GESTURE_NONE) {
    gestureConditionCandidate = GESTURE_NONE;
    return GESTURE_NONE;
  }

  if (candidate != gestureConditionCandidate) {
    gestureConditionCandidate = candidate;
    gestureConditionStartMs = now;
    return GESTURE_NONE;
  }

  if (now - gestureConditionStartMs < GESTURE_MIN_DURATION_MS) return GESTURE_NONE;

  // Condition has held long enough — fire once, then require re-arming.
  gestureArmed = false;
  lastGestureFireMsReal = now;
  gestureConditionCandidate = GESTURE_NONE;
  return candidate;
}

// ---------------------------------------------------------------
// STEP COUNTER (accelerometer peak detection, persisted via NVS)
// ---------------------------------------------------------------
#define STEP_PEAK_THRESHOLD_G     1.15f  // adaptive-ish fixed threshold on |accel|
#define STEP_VALLEY_THRESHOLD_G   0.92f  // must dip below this before the next peak counts (hysteresis)
#define STEP_MIN_INTERVAL_MS      280    // debounce: fastest plausible walking cadence
#define STEP_MAX_INTERVAL_MS      2000   // gap this long resets the "was above peak" latch (isolated spikes)

Preferences stepPrefs;
long stepCount = 0;
bool stepAbovePeak = false;
unsigned long lastStepMs = 0;

void loadStepCount() {
  stepPrefs.begin("morphix", false);
  stepCount = stepPrefs.getLong("steps", 0);
}

void saveStepCount() {
  stepPrefs.putLong("steps", stepCount);
}

long getStepCount() {
  return stepCount;
}

void resetStepCount() {
  stepCount = 0;
  saveStepCount();
}

// Peak/valley hysteresis around |accel|~1g: a step only counts once the
// signal has crossed back down below the valley threshold and then back
// up past the peak threshold, with a minimum time gap — rejects both
// single accidental spikes and rapid shake-driven false counts (shake
// gyro rates are far above what a walking cadence produces, but the
// interval + hysteresis gate rejects most of it regardless).
void updateStepCounter() {
  if (!mpu6050Ready) return;
  unsigned long now = millis();
  float mag = mpuState.accelMag;

  if (!stepAbovePeak && mag < STEP_VALLEY_THRESHOLD_G) {
    stepAbovePeak = false; // stays armed, waiting for the next peak
  }

  if (mag > STEP_PEAK_THRESHOLD_G && !stepAbovePeak) {
    if (now - lastStepMs >= STEP_MIN_INTERVAL_MS) {
      stepCount++;
      lastStepMs = now;
      stepAbovePeak = true;
      static unsigned long lastSaveMs = 0;
      if (now - lastSaveMs > 5000) { // don't hammer flash on every single step
        saveStepCount();
        lastSaveMs = now;
      }
    }
  } else if (mag < STEP_VALLEY_THRESHOLD_G) {
    stepAbovePeak = false;
  }

  if (now - lastStepMs > STEP_MAX_INTERVAL_MS) {
    stepAbovePeak = false; // stale latch — don't let one old peak block future steps
  }
}

// ---------------------------------------------------------------
// RAISE TO WAKE
// ---------------------------------------------------------------
// NOTE: the TFT backlight (TFT LED) is wired straight to 3V3 (see the
// wiring block at the top of this file), not to a switchable GPIO, so
// this build has no physical display-sleep circuit to integrate with.
// Rather than inventing a second, fake sleep/backlight system, raise-to-
// wake here produces a real, debounced WAKE event (raiseToWakeEvent()
// returns true at most once per raise) that the existing navigation
// wires up to jumping back to HOME — the same "wake" action already
// named in handleGesture()'s WAKE DISPLAY label. If a switchable
// backlight pin is added later, gate it off this same event.
#define RAISE_STATIONARY_ACCEL_DEV 0.05f
#define RAISE_STATIONARY_GYRO_DPS  5.0f
#define RAISE_STATIONARY_HOLD_MS   700
#define RAISE_PITCH_DELTA_DEG      25.0f
#define RAISE_COOLDOWN_MS          2000

bool raiseWasStationary = false;
unsigned long raiseStationarySinceMs = 0;
float raisePitchAtStationary = 0;
unsigned long lastRaiseWakeMs = 0;

bool raiseToWakeEvent() {
  if (!motionWakeEnabled || !mpu6050Ready) return false;
  unsigned long now = millis();

  float accelDev = fabsf(mpuState.accelMag - 1.0f);
  bool stationaryNow = (accelDev < RAISE_STATIONARY_ACCEL_DEV &&
                         mpuState.gyroMag < RAISE_STATIONARY_GYRO_DPS);

  if (stationaryNow) {
    if (!raiseWasStationary) {
      raiseWasStationary = true;
      raiseStationarySinceMs = now;
    }
    if (now - raiseStationarySinceMs >= RAISE_STATIONARY_HOLD_MS) {
      raisePitchAtStationary = mpuState.pitch; // baseline orientation while at rest
    }
    return false;
  }

  // Not stationary: check for a characteristic raise (pitch swings up
  // from the resting baseline) once we had a confirmed prior rest period.
  bool hadConfirmedRest = raiseWasStationary &&
    (now - raiseStationarySinceMs >= RAISE_STATIONARY_HOLD_MS);
  raiseWasStationary = false;

  if (!hadConfirmedRest) return false;
  if (now - lastRaiseWakeMs < RAISE_COOLDOWN_MS) return false;

  float pitchDelta = mpuState.pitch - raisePitchAtStationary;
  if (pitchDelta > RAISE_PITCH_DELTA_DEG) {
    lastRaiseWakeMs = now;
    return true;
  }
  return false;
}

// ============================================================
// 6b. MAX30102 SENSOR LAYER (heart rate / SpO2, wrist-optimized)
// ============================================================
// Design goals (see project brief):
//  - Never let a single noisy algorithm output reach the UI.
//  - Detect skin/wrist contact from IR level, with hysteresis so it
//    doesn't flicker at the threshold.
//  - Require several consecutive, mutually-consistent readings before
//    trusting a value ("stable" reading), and invalidate on contact
//    loss, on stale data, or on an implausible range.
//  - Never block the main loop: samples are pulled from the sensor's
//    FIFO a few at a time on every loop() call, and the Maxim
//    algorithm only runs once a 100-sample window is full.

// ---- Tunable thresholds ----
// IR threshold for "skin present". Tuned lower than a typical fingertip
// demo (~50000) because palm/wrist-side contact against the sensor
// window generally returns a weaker IR return than a fingertip pressed
// directly onto the sensor.
#define MAX30102_IR_CONTACT_THRESHOLD 12000
#define MAX30102_IR_MIN_SIGNAL 300

#define MAX30102_CONTACT_DEBOUNCE_SAMPLES 8

#define HR_MIN_BPM 40
#define HR_MAX_BPM 180

#define SPO2_MIN_PCT 85
#define SPO2_MAX_PCT 100

#define HR_CONSISTENCY_DELTA_BPM 15
#define SPO2_CONSISTENCY_DELTA_PCT 5

#define STABILITY_STREAK_LEN 3

#define HR_STALE_TIMEOUT_MS 8000
#define SPO2_STALE_TIMEOUT_MS 8000

// ---- Motion-aware quality gating ----
// Uses the MPU6050 (section 6c, declared above) to keep the MAX30102
// from reporting an absurd HR/SpO2 during vigorous wrist movement. High
// motion doesn't clear the stability filters (a brief real movement
// shouldn't nuke an otherwise-good reading), it just blocks NEW
// candidates from being accepted into them until motion settles.
#define HR_MOTION_GYRO_THRESH_DPS 60.0f

enum HRQuality { HRQ_NONE, HRQ_GOOD, HRQ_MOTION, HRQ_POOR_SIGNAL };
HRQuality hrQuality = HRQ_NONE;

// ---- Raw sample buffer for the Maxim algorithm ----
#define MAX30102_BUFFER_LEN 100
uint32_t irBuffer[MAX30102_BUFFER_LEN];
uint32_t redBuffer[MAX30102_BUFFER_LEN];
int max30102BufferIdx = 0;

// ---- Contact detection (hysteresis via consecutive-sample counters) ----
int max30102IrAboveCount = 0;
int max30102IrBelowCount = 0;

// ---- Rolling stability filter shared by HR and SpO2 ----
// Accepts range-checked candidates one at a time. A candidate that jumps
// too far from the previous one restarts the agreement streak instead of
// being blended in, so a single wild reading can't corrupt the output.
// Once STABILITY_STREAK_LEN consecutive candidates agree, their average
// becomes the exposed "stable" value.
struct StabilityFilter {
  int candidates[STABILITY_STREAK_LEN];
  int count;
  int stableValue;
  bool valid;
  unsigned long lastGoodMs;
};

StabilityFilter hrFilter   = { {0}, 0, -1, false, 0 };
StabilityFilter spo2Filter = { {0}, 0, -1, false, 0 };

void resetStabilityFilter(StabilityFilter &f) {
  f.count = 0;
  f.valid = false;
  f.stableValue = -1;
}

// Called from the HEALTH screen's "SYNC" footer action. Unlike
// manualSensorSync() (BMP280), MAX30102 doesn't take a single on-demand
// reading — it continuously accumulates samples into a rolling window.
// "Sync" here means: stop trusting whatever's currently in flight and
// force a brand-new acquisition to start right now, rather than doing
// nothing (the original bug) or waiting for the current partial window
// to finish on its own.
//  - max30102BufferIdx = 0 restarts the 100-sample window immediately.
//  - resetStabilityFilter() on both filters clears any stale/converged
//    HR/SpO2 value so the UI drops back to "--" until a fresh streak of
//    consistent readings comes in — same as what happens on first contact.
// max30102Contact/max30102SignalGood are deliberately left untouched:
// contact detection keeps running off live IR samples in
// updateMAX30102Contact() and shouldn't be forced one way or the other.
// Non-blocking: no delay(), no re-calling sensor.begin() (already done
// once in initSensors()/setup()) — updateMAX30102() in loop() picks the
// fresh window back up on its next iteration exactly as normal.
void manualHealthSync() {
  max30102BufferIdx = 0;
  resetStabilityFilter(hrFilter);
  resetStabilityFilter(spo2Filter);
}

void manualSensorSync() {
  manualHealthSync();
  needsRedraw = true;
}

void feedStabilityFilter(StabilityFilter &f, int candidate, int consistencyDelta, unsigned long now) {
  if (f.count > 0 && abs(candidate - f.candidates[f.count - 1]) > consistencyDelta) {
    // Too big a jump from the last candidate — start a fresh streak.
    f.count = 0;
  }

  if (f.count < STABILITY_STREAK_LEN) {
    f.candidates[f.count++] = candidate;
  } else {
    f.candidates[0] = f.candidates[1];
    f.candidates[1] = f.candidates[2];
    f.candidates[STABILITY_STREAK_LEN - 1] = candidate;
  }

  if (f.count >= STABILITY_STREAK_LEN) {
    int sum = 0;
    for (int i = 0; i < STABILITY_STREAK_LEN; i++) sum += f.candidates[i];
    f.stableValue = sum / STABILITY_STREAK_LEN;
    f.valid = true;
    f.lastGoodMs = now;
  }
}

void checkStabilityTimeout(StabilityFilter &f, unsigned long now, unsigned long staleTimeoutMs) {
  if (f.valid && (now - f.lastGoodMs > staleTimeoutMs)) {
    resetStabilityFilter(f);
  }
}

// Updates max30102Contact/max30102SignalGood from one raw IR sample, with
// debouncing so brief dips at the threshold don't cause flicker.
void updateMAX30102Contact(uint32_t irValue) {
  if (irValue > MAX30102_IR_CONTACT_THRESHOLD) {
    max30102IrAboveCount++;
    max30102IrBelowCount = 0;
    if (!max30102Contact && max30102IrAboveCount >= MAX30102_CONTACT_DEBOUNCE_SAMPLES) {
      max30102Contact = true;
    }
  } else {
    max30102IrBelowCount++;
    max30102IrAboveCount = 0;
    if (max30102Contact && max30102IrBelowCount >= MAX30102_CONTACT_DEBOUNCE_SAMPLES) {
      max30102Contact = false;
      // Contact just lost — don't let a stale HR/SpO2 linger on the UI.
      resetStabilityFilter(hrFilter);
      resetStabilityFilter(spo2Filter);
    }
  }
  max30102SignalGood = max30102Contact;
}

// Runs the Maxim HR/SpO2 algorithm on the current 100-sample window and
// feeds any plausible, in-range result into the stability filters.
// validHeartRate/validSPO2 from the algorithm are treated as necessary
// but NOT sufficient — range + consistency filtering still applies.
void processMAX30102Window() {

   max30102Measuring = false;
  int32_t spo2Value = -1;
  int8_t  spo2AlgoValid = 0;

  int32_t hrValue = -1;
  int8_t  hrAlgoValid = 0;

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    MAX30102_BUFFER_LEN,
    redBuffer,
    &spo2Value,
    &spo2AlgoValid,
    &hrValue,
    &hrAlgoValid
  );

  if (!max30102Contact) {
    return;
  }

  // ----------------------------------------------------------
  // Calculate optical signal amplitude
  // ----------------------------------------------------------

  uint32_t irMin = 0xFFFFFFFF;
  uint32_t irMax = 0;

  uint64_t irSum = 0;

  for (int i = 0; i < MAX30102_BUFFER_LEN; i++) {

    uint32_t v = irBuffer[i];

    if (v < irMin) irMin = v;
    if (v > irMax) irMax = v;

    irSum += v;
  }

  uint32_t irAverage = irSum / MAX30102_BUFFER_LEN;

  uint32_t irAC = irMax - irMin;

  // Reject almost-flat optical signals.
  if (irAverage < MAX30102_IR_CONTACT_THRESHOLD ||
      irAC < MAX30102_IR_MIN_SIGNAL) {

    max30102SignalGood = false;

    return;
  }

  max30102SignalGood = true;

  unsigned long now = millis();

  // High wrist motion during this window: don't feed a new candidate in
  // at all (a real vigorous-movement artifact could easily read as
  // 180+ BPM). The existing stable value is left untouched rather than
  // cleared, since a brief motion blip shouldn't wipe an otherwise-good
  // reading — checkStabilityTimeout() will age it out if motion persists
  // past HR_STALE_TIMEOUT_MS.
  bool highMotion = mpu6050Ready && (mpuState.gyroMag > HR_MOTION_GYRO_THRESH_DPS);

  // ----------------------------------------------------------
  // HEART RATE
  // ----------------------------------------------------------

  if (
    !highMotion &&
    hrAlgoValid &&
    hrValue >= HR_MIN_BPM &&
    hrValue <= HR_MAX_BPM
  ) {

    feedStabilityFilter(
      hrFilter,
      (int)hrValue,
      HR_CONSISTENCY_DELTA_BPM,
      now
    );
  }

  // ----------------------------------------------------------
  // SpO2
  // ----------------------------------------------------------

  if (
    !highMotion &&
    spo2AlgoValid &&
    spo2Value >= SPO2_MIN_PCT &&
    spo2Value <= SPO2_MAX_PCT
  ) {

    feedStabilityFilter(
      spo2Filter,
      (int)spo2Value,
      SPO2_CONSISTENCY_DELTA_PCT,
      now
    );
  }

  // ----------------------------------------------------------
  // DEBUG
  // ----------------------------------------------------------

  Serial.print("MAX30102 WINDOW | IR=");
  Serial.print(irAverage);

  Serial.print(" AC=");
  Serial.print(irAC);

  Serial.print(" HR=");
  Serial.print(hrValue);

  Serial.print(" HR_VALID=");
  Serial.print(hrAlgoValid);

  Serial.print(" SPO2=");
  Serial.print(spo2Value);

  Serial.print(" SPO2_VALID=");
  Serial.println(spo2AlgoValid);
}

// Pulls whatever samples are currently available from the MAX30102's FIFO
// (non-blocking — does nothing if no new sample is ready yet), updates
// contact detection per-sample, and runs the algorithm once per full
// 100-sample window, sliding the window by 25 samples each time (matches
// the SparkFun reference cadence: ~4 updates/sec at 100 Hz sampling).
void updateMAX30102() {

  if (!max30102Ready) {
    return;
  }

  unsigned long now = millis();
  checkStabilityTimeout(hrFilter, now, HR_STALE_TIMEOUT_MS);
  checkStabilityTimeout(spo2Filter, now, SPO2_STALE_TIMEOUT_MS);

  max30102Sensor.check(); // non-blocking: pulls any new FIFO samples into the library's ring buffer

  while (max30102Sensor.available()) {
    uint32_t ir  = max30102Sensor.getIR();
    uint32_t red = max30102Sensor.getRed();
    max30102Sensor.nextSample();

    updateMAX30102Contact(ir);

    irBuffer[max30102BufferIdx]  = ir;
    redBuffer[max30102BufferIdx] = red;
    max30102BufferIdx++;

    if (max30102BufferIdx >= MAX30102_BUFFER_LEN) {
      processMAX30102Window();

      const int keep = 25;
      for (int i = 0; i < keep; i++) {
        irBuffer[i]  = irBuffer[MAX30102_BUFFER_LEN - keep + i];
        redBuffer[i] = redBuffer[MAX30102_BUFFER_LEN - keep + i];
      }
      max30102BufferIdx = keep;
    }
  }

  if (!max30102Contact) {

      heartRateValid = false;
      spo2Valid = false;
      max30102SignalGood = false;
      hrQuality = HRQ_NONE; // PLACE ON SKIN — handled by the Health screen itself
  }
  else {

    heartRateValid = hrFilter.valid;
    spo2Valid = spo2Filter.valid;

    bool highMotionNow = mpu6050Ready && (mpuState.gyroMag > HR_MOTION_GYRO_THRESH_DPS);
    if (!max30102SignalGood)   hrQuality = HRQ_POOR_SIGNAL;
    else if (highMotionNow)    hrQuality = HRQ_MOTION;
    else                       hrQuality = HRQ_GOOD;
  }
}

// True once contact is detected and the signal hasn't dropped out.
// (Kept as its own function per the requested sensor-layer shape, even
// though it currently mirrors max30102Contact/max30102SignalGood — the
// separation leaves room to fold in AC-amplitude checks later without
// touching any call sites.)
bool isMAX30102SignalGood() {
  return max30102Ready && max30102SignalGood;
}

// Returns the filtered/stable heart rate in BPM, or -1 if no trustworthy
// value is currently available (UI should show "--").
int getStableHeartRate() {
  return heartRateValid ? hrFilter.stableValue : -1;
}

// Returns the filtered/stable SpO2 percentage, or -1 if no trustworthy
// value is currently available (UI should show "--").
int getStableSpO2() {
  return spo2Valid ? spo2Filter.stableValue : -1;
}

// One-time sensor bring-up, called ONCE from setup(). Each sensor is
// tracked independently so a failure on one doesn't stop the other from
// working (Environment keeps working if MAX30102 fails; Health keeps
// working if BMP280 fails).
void initSensors() {
  Serial.println("MORPHIX SENSOR INITIALIZATION");

  Wire.begin(BMP_SDA, BMP_SCL); // single shared I2C bus for both sensors

  // ---- BMP280 (address 0x76) ----
  if (bmp.begin(0x76)) {
    bmp280Ready = true;
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X4,
      Adafruit_BMP280::STANDBY_MS_1000
    );
    updateBMP280(); // seed a first real reading immediately
  } else {
    bmp280Ready = false;
  }
  Serial.print("BMP280: ");
  Serial.println(bmp280Ready ? "OK" : "FAIL");

  // ---- MAX30102 (address 0x57) ----
  if (max30102Sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    max30102Ready = true;

    byte ledBrightness = 70;
    byte sampleAverage  = 4;
    byte ledMode        = 2;
    int  sampleRate     = 100;
    int  pulseWidth     = 411;
    int  adcRange       = 4096;
    max30102Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  } else {
    max30102Ready = false;
  }
  Serial.print("MAX30102: ");
  Serial.println(max30102Ready ? "OK" : "FAIL");

  // ---- MPU6050 (address 0x68) ----
  if (initMPU6050()) {
    mpu6050Ready = true;
    calibrateGyro(); // brief, startup-only — assumes the watch is at rest
  } else {
    mpu6050Ready = false;
  }
  Serial.print("MPU6050: ");
  Serial.println(mpu6050Ready ? "OK" : "FAIL");

  loadStepCount(); // restore today's step count from NVS
}

// Pumps all three sensors. Cheap/non-blocking; safe to call from loop()
// at whatever cadence the caller chooses. MPU6050 is intentionally NOT
// pumped here — it runs on its own fixed millis() cadence in loop() (see
// MPU_SAMPLE_INTERVAL_MS) so its 50-100 Hz target rate isn't tied to
// whatever rate updateSensors() happens to get called at.
void updateSensors() {
  updateBMP280();
  updateMAX30102();
}


// Removes the currently-shown notification locally. If it's still linked
// to a real phone notification (has a non-empty id), also tells the
// phone so the Android app can cancel the matching system notification —
// see WCMD_DISMISS_NOTIFICATION / sendCommandToPhone() in the BLE bridge
// section below. We never claim the phone-side dismissal succeeded; we
// just ask, per the "do not claim dismissal succeeded unless Android
// confirms" requirement — the phone is the source of truth.
void dismissNotification() {
  if (notifCount <= 0) return;

  if (notifQueue[notifIndex].id[0] != 0) {
    sendCommandToPhone(WCMD_DISMISS_NOTIFICATION,
                        (const uint8_t*)notifQueue[notifIndex].id,
                        (uint8_t)strlen(notifQueue[notifIndex].id));
  }

  for (int i = notifIndex; i < notifCount - 1; i++) {
    notifQueue[i] = notifQueue[i + 1];
  }
  notifCount--;
  if (notifIndex >= notifCount) notifIndex = max(0, notifCount - 1);
}

// Inserts a freshly-received notification at the front of the queue
// (index 0 = newest, matches how the screen pages through them). When
// full, the oldest entry (tail) is dropped to make room — a bounded ring
// buffer, never a dynamic allocation.
void pushNotificationToQueue(const char* id, const char* app, const char* title,
                              const char* text, uint32_t timestampSec) {
  int insertCount = min(notifCount + 1, MAX_NOTIF_QUEUE);
  for (int i = insertCount - 1; i > 0; i--) {
    notifQueue[i] = notifQueue[i - 1];
  }
  QueuedNotification &n = notifQueue[0];
  strncpy(n.id, id, NOTIF_ID_LEN - 1);       n.id[NOTIF_ID_LEN - 1] = 0;
  strncpy(n.app, app, NOTIF_APP_LEN - 1);    n.app[NOTIF_APP_LEN - 1] = 0;
  strncpy(n.title, title, NOTIF_TITLE_LEN - 1); n.title[NOTIF_TITLE_LEN - 1] = 0;
  strncpy(n.text, text, NOTIF_TEXT_LEN - 1); n.text[NOTIF_TEXT_LEN - 1] = 0;
  n.timestampSec = timestampSec;
  n.unread = true;

  notifCount = insertCount;
  notifIndex = 0; // jump to the new notification, same as a phone unlocking to the latest alert

  needsRedraw = true; // screen-agnostic here: currentScreen/ScreenID isn't declared this early in the file yet
}

// Marks the queue entry with a matching phone-side id as read (used by
// CMD_MARK_READ arriving from the phone, e.g. because the user read it
// on their phone instead of on the watch).
void markNotificationReadById(const char* id) {
  for (int i = 0; i < notifCount; i++) {
    if (strncmp(notifQueue[i].id, id, NOTIF_ID_LEN) == 0) {
      notifQueue[i].unread = false;
      needsRedraw = true; // see note above — currentScreen isn't declared yet at this point in the file
      return;
    }
  }
}

int unreadNotificationCount() {
  int c = 0;
  for (int i = 0; i < notifCount; i++) if (notifQueue[i].unread) c++;
  return c;
}

// ---------- LIVE CONNECTIVITY DATA ----------
// wifiConnected/wifiSSID/wifiRSSI are kept in sync with real
// WiFi.status()/WiFi.SSID()/WiFi.RSSI() reads; ble* fields are kept in
// sync with the real ESP32-S3 BLE stack (BLEDevice/BLEScan/BLEClient);
// phone* fields track whether a real BLE link is up (MORPHIX doesn't yet
// speak a phone-link protocol over it, so "connected" just means "a real
// BLE peer is connected").
struct ConnectivityData {
  bool wifiConnected;
  char wifiSSID[16];
  int  wifiRSSI;

  bool bleReady;       // BLE stack initialized, independent of a link
  bool bleConnected;
  char bleDeviceName[16];
  int  bleRSSI;

  bool phoneConnected;         // real MORPHIX BLE GATT server has a connected central (the phone)
  char phoneLastSync[9];       // "HH:MM:SS", real time when synced, "--:--:--" when not
  bool notifAccessEnabled;     // reported by the Android app via CMD_STATUS_UPDATE
  int  phoneBatteryPct;        // -1 = unknown (phone hasn't reported one yet)
};

ConnectivityData connData = {
  false, "", 0,
  false, false, "", -100,
  false, "--:--:--", false, -1
};

// ---------- WI-FI DISCOVERY (hierarchical: dashboard -> scan -> list ->
// details -> connect) ----------
// WifiNetwork is the "result" shape, filled from a real WiFi.scanNetworks()
// pass (ssid = WiFi.SSID(i), rssi = WiFi.RSSI(i), security derived from
// WiFi.encryptionType(i)). Nothing in the UI cares how it was filled.
struct WifiNetwork {
  char ssid[16];
  int  rssi;          // dBm
  const char* security; // "WPA2" / "WPA3" / "WEP" / "OPEN" / etc. (static string, not owned)
};

const int MAX_WIFI_NETWORKS = 6;
WifiNetwork wifiNetworks[MAX_WIFI_NETWORKS];
int wifiNetworkCount   = 0;  // how many of the above are valid, filled by pollWifiScan()
int wifiListIndex      = 0;  // row currently highlighted while browsing AVAILABLE NETWORKS
int wifiListScroll     = 0;  // index of the first visible row (for >4 results)
int selectedWifiNetwork = 0; // confirmed via SELECT on the list; used by DETAILS/CONNECTING/etc.

// ---------- WI-FI TEST CREDENTIALS (development only) ----------
// Fill these in with a real nearby network's SSID/password to test REAL
// WiFi connect. Left blank, MORPHIX will still connect to OPEN networks,
// but will show NOT CONFIGURED / PASSWORD REQUIRED for any protected
// network it doesn't have a matching credential for. Never printed to
// the TFT or Serial Monitor.
const char* TEST_WIFI_SSID     = "";
const char* TEST_WIFI_PASSWORD = "";

enum WifiScreenState {
  WIFI_SCR_STATUS,      // dashboard
  WIFI_SCR_SCANNING,
  WIFI_SCR_LIST,        // available networks
  WIFI_SCR_DETAILS,     // network details
  WIFI_SCR_NOCRED,      // protected network with no usable credentials
  WIFI_SCR_CONNECTING,
  WIFI_SCR_CONNECTED,   // brief confirmation, then auto-returns to dashboard
  WIFI_SCR_FAILED
};
WifiScreenState wifiScreenState = WIFI_SCR_STATUS;
unsigned long wifiStateTimerMs = 0;
const unsigned long WIFI_SCAN_TIMEOUT_MS    = 8000; // safety fallback if scanComplete() never resolves
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000; // give up on a connection attempt after this long
const unsigned long WIFI_CONNECTED_HOLD_MS  = 1200; // how long CONNECTED shows before returning to dashboard

// ============================================================
// BACKGROUND WI-FI AUTO-CONNECT / AUTO-RECONNECT
// ============================================================
// Fully separate, non-blocking state machine that keeps MORPHIX attached
// to the configured WIFI_SSID/WIFI_PASSWORD (e.g. a phone hotspot) in the
// background, independent of whatever the user is doing on the manual
// WIFI screen above. It never calls delay() and never loops on
// WiFi.status(); every state transition is driven by millis() timers
// polled once per loop() iteration. To avoid fighting the user, it stays
// out of the way whenever the WIFI screen is in the middle of a manual
// scan/connect flow (see serviceWifiAutoConnect()).
enum WifiAutoState {
  WIFI_AUTO_IDLE,
  WIFI_AUTO_CONNECTING,
  WIFI_AUTO_CONNECTED,
  WIFI_AUTO_DISCONNECTED,
  WIFI_AUTO_RECONNECTING
};
WifiAutoState wifiAutoState = WIFI_AUTO_IDLE;
unsigned long wifiAutoStateTimerMs = 0;
const unsigned long WIFI_AUTO_CONNECT_TIMEOUT_MS    = 15000; // give up on one attempt after this long
const unsigned long WIFI_AUTO_RECONNECT_INTERVAL_MS = 10000; // wait this long between retry attempts

// ============================================================
// REAL-TIME CLOCK (NTP-synced ESP32 system time, Asia/Kolkata)
// ============================================================
// wifiConnected tracks the Wi-Fi link itself; ntpSynced tracks whether the
// ESP32 system clock has actually been set from an NTP server at least
// once — internet access is never assumed just because Wi-Fi associated.
bool ntpSynced           = false;  // true once a real NTP sync has succeeded this boot
bool clockHasValidTime   = false;  // true once the system clock holds a real time (NTP, or restored from NVS)
bool ntpSyncInProgress   = false;  // an SNTP sync request is currently outstanding
unsigned long ntpSyncStartedMs   = 0;
unsigned long lastNtpPollMs      = 0;
const unsigned long NTP_POLL_INTERVAL_MS = 2000;   // how often to check whether sync completed
// Any time() value below this is treated as "not real" — the ESP32 boots
// with its clock at/near the epoch, so this sanity threshold (roughly
// 2023-11-14) reliably distinguishes a synced clock from an unsynced one.
const time_t NTP_VALID_EPOCH_THRESHOLD = 1700000000;

Preferences timePrefs; // separate NVS namespace from stepPrefs — see loadStepCount() above

// ---------- BLUETOOTH DISCOVERY (dashboard -> scan -> list -> details -> pair) ----------
// NOTE: named BleDeviceInfo, not "BLEDevice" — the ESP32 Arduino BLE
// library (<BLEDevice.h>) already defines a class called BLEDevice, and
// reusing that name here would collide with it at compile time.
struct BleDeviceInfo {
  char name[16];
  int  rssi;           // dBm
  char addr[18];        // "aa:bb:cc:dd:ee:ff" — used internally to connect/dedupe, not always shown
  bool hasName;         // false if the device didn't advertise a name (shown as UNKNOWN DEVICE)
};

const int MAX_BLE_DEVICES = 6;
BleDeviceInfo bleDevices[MAX_BLE_DEVICES];
int bleDeviceCount    = 0;   // filled live by the BLE scan callback during BLE_SCR_SCANNING
int bleListIndex      = 0;   // row currently highlighted while browsing AVAILABLE DEVICES
int bleListScroll     = 0;   // index of the first visible row (for >4 results)
int selectedBLEDevice = 0;   // confirmed via SELECT on the list

enum BleScreenState {
  BLE_SCR_STATUS,      // dashboard
  BLE_SCR_SCANNING,
  BLE_SCR_LIST,        // available devices
  BLE_SCR_DETAILS,     // device details
  BLE_SCR_PAIRING,
  BLE_SCR_CONNECTED,   // brief confirmation, then auto-returns to dashboard
  BLE_SCR_FAILED
};
BleScreenState bleScreenState = BLE_SCR_STATUS;
unsigned long bleStateTimerMs = 0;
const unsigned long BLE_SCAN_DURATION_SEC = 3;     // real active-scan duration
const unsigned long BLE_SCAN_TIMEOUT_MS   = 8000;  // safety fallback if the completion callback never fires
const unsigned long BLE_PAIR_MS           = 300;   // lets "PAIRING..." paint before the (briefly blocking) connect call
const unsigned long BLE_CONNECTED_HOLD_MS = 1200;  // how long CONNECTED shows before returning to dashboard

// BLE client connections use this library's synchronous connect() call,
// so unlike WiFi there's no separate CONNECTING/timeout poll — see
// connectBLEDevice() and the "BLE limitations" note in the summary.
BLEScan*  pBLEScan  = nullptr;
BLEClient* pBLEClient = nullptr;
volatile bool bleScanDone = false;      // set by the scan-complete callback (BLE task context)
unsigned long bleScanStartMs = 0;
const char* bleFailReason = "CONNECTION FAILED"; // or "NOT CONNECTABLE" — shown on BLE_SCR_FAILED

// Shown list rows at once — the TFT is only 160x128, so lists scroll
// instead of trying to cram everything on screen (see LIST DESIGN below).
const int LIST_VISIBLE_ROWS = 4;

// Keeps a scroll window's start index following a highlighted selection
// as it moves (and wrapping cleanly back to the top when the selection
// wraps). Shared by the WiFi and BLE result lists.
void clampListScroll(int &scrollIdx, int selIdx, int count) {
  if (count <= LIST_VISIBLE_ROWS) { scrollIdx = 0; return; }
  if (selIdx < scrollIdx) scrollIdx = selIdx;
  if (selIdx >= scrollIdx + LIST_VISIBLE_ROWS) scrollIdx = selIdx - LIST_VISIBLE_ROWS + 1;
}

// ---------- SIGNAL STRENGTH (shared by WiFi + BLE rows/details) ----------
// Bucket an RSSI reading (dBm, more negative = weaker) into a 1-4 bar count.
int signalBars(int rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  return 1;
}

uint16_t signalColor(int rssi) {
  if (rssi >= -65) return COL_GOOD;   // strong -> green
  if (rssi >= -75) return COL_WARN;   // medium -> yellow
  return COL_CRIT;                    // weak   -> red
}

// Tiny 4-bar signal indicator drawn with plain rectangles (no unicode
// glyphs, since the TFT font doesn't have them). ~11x8px.
void drawSignalBars(int16_t x, int16_t y, int rssi) {
  int bars = signalBars(rssi);
  uint16_t color = signalColor(rssi);
  for (int i = 0; i < 4; i++) {
    int16_t bw = 2, gap = 1;
    int16_t bh = 2 + i * 2;           // 2,4,6,8 px tall, ascending
    int16_t bx = x + i * (bw + gap);
    int16_t by = y + (8 - bh);        // bottoms aligned
    if (i < bars) tft.fillRect(bx, by, bw, bh, color);
    else          tft.drawRect(bx, by, bw, bh, COL_BORDER);
  }
}

// ---------- WI-FI REAL BACKEND ----------
// Maps an ESP32 wifi_auth_mode_t to the short label the UI already knows
// how to display. Returns a pointer to a static string literal, so it's
// safe to stash directly into WifiNetwork::security.
const char* wifiAuthModeToStr(wifi_auth_mode_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN:            return "OPEN";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/3";
    default:                        return "UNKNOWN";
  }
}

// Kicks off a real, non-blocking Wi-Fi scan. The UI moves to
// WIFI_SCR_SCANNING immediately after calling this; pollWifiScan() (called
// every loop() while in that state) picks up the results once ready.
void startWifiScan() {
  Serial.println("WiFi scan started");
  // Cancel any connection attempt still in flight (e.g. auto-connect's
  // WiFi.begin() from a prior loop iteration). ESP32 refuses to run a scan
  // while STA is mid-connect, which is what made scans silently fail.
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.scanDelete();          // release any previous scan's results first
  WiFi.scanNetworks(true);    // async = true: returns immediately, runs in background
}

// Polls WiFi.scanComplete(). No-op while still running. Populates
// wifiNetworks[]/wifiNetworkCount and advances wifiScreenState once
// results are ready (or treats a failed/empty scan as "no networks").
void pollWifiScan() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;

  if (n == WIFI_SCAN_FAILED) {
    Serial.println("WiFi scan failed");
    wifiNetworkCount = 0;
    wifiScreenState = WIFI_SCR_STATUS;
    needsRedraw = true;
    return;
  }

  Serial.print("WiFi scan complete. Found ");
  Serial.println(n);

  wifiNetworkCount = min(n, MAX_WIFI_NETWORKS);
  for (int i = 0; i < wifiNetworkCount; i++) {
    strncpy(wifiNetworks[i].ssid, WiFi.SSID(i).c_str(), sizeof(wifiNetworks[i].ssid) - 1);
    wifiNetworks[i].ssid[sizeof(wifiNetworks[i].ssid) - 1] = 0;
    wifiNetworks[i].rssi = WiFi.RSSI(i);
    wifiNetworks[i].security = wifiAuthModeToStr(WiFi.encryptionType(i));
  }
  WiFi.scanDelete(); // free the scan result buffer now that we've copied what we need

  wifiListIndex = 0; wifiListScroll = 0;
  wifiScreenState = (wifiNetworkCount > 0) ? WIFI_SCR_LIST : WIFI_SCR_STATUS;
  needsRedraw = true;
}

// Starts (or rejects) a real connection attempt for a scanned network.
// OPEN networks connect with no password. Protected networks only
// proceed if they match the TEST_WIFI_SSID/PASSWORD dev credentials —
// otherwise the UI moves to WIFI_SCR_NOCRED instead of guessing a
// password. Returns true if WiFi.begin() was actually called.
bool attemptWifiConnect(int index) {
  if (index < 0 || index >= wifiNetworkCount) return false;
  WifiNetwork &n = wifiNetworks[index];

  bool isOpen = (strcmp(n.security, "OPEN") == 0);
  bool haveTestCreds = (strlen(TEST_WIFI_SSID) > 0 && strlen(TEST_WIFI_PASSWORD) > 0 &&
                        strcmp(n.ssid, TEST_WIFI_SSID) == 0);

  if (!isOpen && !haveTestCreds) {
    wifiScreenState = WIFI_SCR_NOCRED;
    needsRedraw = true;
    return false;
  }

  Serial.println("Connection attempt:");
  Serial.print("SSID = "); Serial.println(n.ssid);
  if (isOpen) WiFi.begin(n.ssid);
  else        WiFi.begin(n.ssid, TEST_WIFI_PASSWORD);

  wifiScreenState = WIFI_SCR_CONNECTING;
  wifiStateTimerMs = millis();
  return true;
}

// Real disconnect — also updates connData immediately so the UI never
// keeps showing CONNECTED after this returns.
void disconnectWiFi() {
  WiFi.disconnect(true);
  connData.wifiConnected = false;
  connData.wifiSSID[0] = 0;
  connData.wifiRSSI = 0;
  Serial.println("WiFi disconnected");
}

// ============================================================
// BACKGROUND WI-FI AUTO-CONNECT / AUTO-RECONNECT (implementation)
// ============================================================

// Kicks off (or re-kicks off) NTP sync. Safe to call repeatedly — it just
// (re)points the SNTP client at the configured servers and lets it work in
// the background; it never blocks. ntpSynced is intentionally NOT reset
// here, so a brief Wi-Fi drop/reconnect doesn't make an already-synced
// clock look unsynced again.
void startNtpSync() {
  configTzTime(MORPHIX_TZ, NTP_SERVER_1, NTP_SERVER_2);
  ntpSyncInProgress = true;
  ntpSyncStartedMs = millis();
  Serial.println("NTP synchronization started");
}

// Non-blocking poll: checks whether the system clock has picked up a real
// time yet. Called periodically from loop(), never every iteration.
void serviceNtpSync(unsigned long now) {
  if (!ntpSyncInProgress) return;
  if (now - lastNtpPollMs < NTP_POLL_INTERVAL_MS) return;
  lastNtpPollMs = now;

  time_t nowSec = time(nullptr);
  if (nowSec >= NTP_VALID_EPOCH_THRESHOLD) {
    ntpSyncInProgress = false;
    bool firstSync = !ntpSynced;
    ntpSynced = true;
    clockHasValidTime = true;

    // Persist the last known-good time so a future boot without internet
    // can still show a continuing (if stale) clock instead of a blank one.
    timePrefs.putULong("lastEpoch", (uint32_t)nowSec);

    if (firstSync) {
      struct tm tmNow;
      localtime_r(&nowSec, &tmNow);
      char timeBuf[9];
      snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec);
      Serial.println("NTP synchronized");
      Serial.print("Time: ");
      Serial.println(timeBuf);
    }
  }
  // Not yet valid: SNTP keeps retrying internally on its own schedule;
  // we just keep polling at NTP_POLL_INTERVAL_MS until it lands.
}

// Starts (or restarts) a background connection attempt to the configured
// WIFI_SSID. Never blocks — WiFi.begin() returns immediately and the
// actual result is picked up later by serviceWifiAutoConnect().
void startWifiAutoConnect() {
  Serial.println("WiFi connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiAutoState = WIFI_AUTO_CONNECTING;
  wifiAutoStateTimerMs = millis();
}

// Called once per loop() iteration. All actual work (connection attempts,
// retries) is gated by millis() timers — this does NOT attempt a
// connection every loop, and never blocks with delay() or a status-polling
// while() loop. Steps aside whenever the user is actively driving the
// manual WIFI screen (scanning/connecting/browsing details) so the two
// don't fight over the radio.
void serviceWifiAutoConnect(unsigned long now) {
  // Step aside for the ENTIRE time the user is on the WIFI screen, not just
  // once they've already started scanning/connecting. Auto-connect calls
  // WiFi.begin() on its own retry timer; if that leaves the radio mid
  // connection-attempt right when the user hits SELECT to scan,
  // WiFi.scanNetworks() fails immediately (ESP32 refuses to scan while a
  // connect is in progress). Pausing as soon as the screen is entered
  // avoids that race instead of just avoiding it after the fact.
  bool userOnManualWifiFlow = (currentScreen == SCR_WIFI);
  if (userOnManualWifiFlow) return;

  switch (wifiAutoState) {
    case WIFI_AUTO_IDLE:
      // Nothing to do — startWifiAutoConnect() hasn't been called yet
      // (only happens once, from setup()).
      break;

    case WIFI_AUTO_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        wifiAutoState = WIFI_AUTO_CONNECTED;
        wifiAutoStateTimerMs = now;
        Serial.println("WiFi connected");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP().toString());
        startNtpSync();
      } else if (now - wifiAutoStateTimerMs >= WIFI_AUTO_CONNECT_TIMEOUT_MS) {
        wifiAutoState = WIFI_AUTO_DISCONNECTED;
        wifiAutoStateTimerMs = now;
      }
      break;

    case WIFI_AUTO_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected");
        wifiAutoState = WIFI_AUTO_RECONNECTING;
        wifiAutoStateTimerMs = now;
      }
      break;

    case WIFI_AUTO_DISCONNECTED:
    case WIFI_AUTO_RECONNECTING:
      if (now - wifiAutoStateTimerMs >= WIFI_AUTO_RECONNECT_INTERVAL_MS) {
        Serial.println("WiFi reconnecting...");
        startWifiAutoConnect(); // sets state back to WIFI_AUTO_CONNECTING
      }
      break;
  }
}

// Reads the ESP32 system clock (already in Asia/Kolkata local time once
// MORPHIX_TZ has been applied by startNtpSync()/setup() — no manual
// +5:30 offset is added here, which would double-convert it) into the
// existing on-screen clock fields. Only overwrites those fields once the
// clock actually holds a real value (fresh NTP sync, or a timestamp
// restored from NVS at boot) — otherwise the UI's placeholder is left
// alone rather than fabricating a time. This is the function the existing
// UI's once-per-second tick calls; the display code itself is untouched.
void updateClockData() {
  if (!clockHasValidTime) return;

  time_t nowSec = time(nullptr);
  struct tm tmNow;
  localtime_r(&nowSec, &tmNow);

  mockHour   = tmNow.tm_hour;
  mockMinute = tmNow.tm_min;
  mockSecond = tmNow.tm_sec;

  static const char* dayNames[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};
  strncpy(mockDay, dayNames[tmNow.tm_wday], sizeof(mockDay) - 1);
  mockDay[sizeof(mockDay) - 1] = 0;

  static const char* monNames[12] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
  snprintf(mockDate, sizeof(mockDate), "%02d %s", tmNow.tm_mday, monNames[tmNow.tm_mon]);
}

// ---------- BLUETOOTH LE REAL BACKEND ----------
// Advertised-device callback: runs on the BLE stack's task while a scan
// is in progress, and appends each newly-seen device straight into
// bleDevices[]. Duplicate addresses within one scan are skipped.
class MorphixBleScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (bleDeviceCount >= MAX_BLE_DEVICES) return;

    String addrStr = advertisedDevice.getAddress().toString().c_str();
    for (int i = 0; i < bleDeviceCount; i++) {
      if (strncmp(bleDevices[i].addr, addrStr.c_str(), sizeof(bleDevices[i].addr)) == 0) return;
    }

    BleDeviceInfo &d = bleDevices[bleDeviceCount];
    if (advertisedDevice.haveName()) {
      strncpy(d.name, advertisedDevice.getName().c_str(), sizeof(d.name) - 1);
      d.name[sizeof(d.name) - 1] = 0;
      d.hasName = true;
    } else {
      strncpy(d.name, "UNKNOWN DEVICE", sizeof(d.name) - 1);
      d.name[sizeof(d.name) - 1] = 0;
      d.hasName = false;
    }
    d.rssi = advertisedDevice.getRSSI();
    strncpy(d.addr, addrStr.c_str(), sizeof(d.addr) - 1);
    d.addr[sizeof(d.addr) - 1] = 0;

    bleDeviceCount++;
  }
};

// Fires when the scan duration finishes. Runs off the main loop's thread,
// so it only ever sets a flag — pollBleScan() does the actual work.
void bleScanCompleteCB(BLEScanResults results) {
  bleScanDone = true;
}

// Kicks off a real, non-blocking BLE scan. bleDeviceCount is reset to 0
// here; results stream in live via MorphixBleScanCallbacks::onResult()
// as advertisements arrive, so the list can in principle be watched
// filling in even before the scan finishes.
void startBLEScan() {
  Serial.println("BLE scan started");
  bleDeviceCount = 0;
  bleScanDone = false;
  bleScanStartMs = millis();
  pBLEScan->clearResults();
  pBLEScan->start((uint32_t)BLE_SCAN_DURATION_SEC, bleScanCompleteCB, false);
}

// Real BLE connect. NOTE: BLEClient::connect() in this library is
// synchronous — it blocks for the duration of the connection attempt
// (typically well under a second, occasionally a couple of seconds if
// the peer is slow/out of range). This is a known limitation of the
// bundled ESP32 Arduino BLE stack; see the summary for details.
bool connectBLEDevice(int index) {
  if (index < 0 || index >= bleDeviceCount) return false;
  BleDeviceInfo &d = bleDevices[index];

  if (pBLEClient == nullptr) pBLEClient = BLEDevice::createClient();
  if (pBLEClient->isConnected()) pBLEClient->disconnect();

  Serial.println("BLE connection attempt");
  BLEAddress addr(d.addr);
  bool ok = pBLEClient->connect(addr);

  if (ok) {
    connData.bleConnected = true;
    strncpy(connData.bleDeviceName, d.name, sizeof(connData.bleDeviceName) - 1);
    connData.bleDeviceName[sizeof(connData.bleDeviceName) - 1] = 0;
    connData.bleRSSI = d.rssi;
    Serial.println("BLE connection result: SUCCESS");
  } else {
    bleFailReason = "NOT CONNECTABLE";
    Serial.println("BLE connection result: FAILED");
  }
  return ok;
}

// Real BLE disconnect.
void disconnectBLEDevice() {
  if (pBLEClient != nullptr && pBLEClient->isConnected()) {
    pBLEClient->disconnect();
  }
  connData.bleConnected = false;
  connData.bleDeviceName[0] = 0;
  connData.bleRSSI = 0;
  Serial.println("BLE disconnected");
}

// Phone-link state machine. phoneState is now driven by the real MORPHIX
// BLE GATT server below (MorphixServerCallbacks sets PHONE_CONNECTED /
// PHONE_NOT_PAIRED directly on connect/disconnect). SYNCING/SYNC_DONE are
// still just a short UI hold around a real WCMD_REQUEST_SYNC round trip
// (see the SELECT handler for SCR_PHONELINK and serviceMorphixBle()).
enum PhoneLinkState { PHONE_NOT_PAIRED, PHONE_PAIRING, PHONE_CONNECTED, PHONE_SYNCING, PHONE_SYNC_DONE };
PhoneLinkState phoneState = PHONE_NOT_PAIRED;
unsigned long phoneStateTimerMs = 0;
const unsigned long PHONE_PAIR_MS           = 1200;
const unsigned long PHONE_SYNC_MS           = 900;
const unsigned long PHONE_SYNC_DONE_HOLD_MS = 1200;

// ============================================================
// 6e. PHONE BLE BRIDGE (MORPHIX BLE GATT server / Android companion app)
// ============================================================
// MORPHIX acts as a BLE peripheral/server here — a separate role from
// the BLUETOOTH screen's pBLEScan/pBLEClient above, which is MORPHIX
// acting as a BLE central to pair with arbitrary nearby BLE devices.
// Both roles run concurrently on the same radio; the ESP32 BLE stack
// supports that. The Android companion app is the BLE central/client for
// THIS service: it scans for the advertised name "MORPHIX", connects,
// discovers this service, and subscribes to the NOTIFY characteristics.
//
// Every callback below (onConnect/onDisconnect/onWrite) runs on the
// Bluedroid BLE task, not the Arduino loop() task — exactly like
// MorphixBleScanCallbacks::onResult() above, callbacks here only touch
// plain globals and never call tft.* or block, and loop() picks up the
// results via needsRedraw / serviceMorphixBle(). This matches the
// concurrency approach already used elsewhere in this file.

#define MORPHIX_SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_NOTIFICATION_TX_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Phone -> Watch, WRITE (notifications + commands)
#define CHAR_COMMAND_RX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Watch -> Phone, NOTIFY
#define CHAR_STATUS_TX_UUID       "6e400004-b5a3-f393-e0a9-e50e24dcca9e" // Watch -> Phone, NOTIFY
#define CHAR_TIME_SYNC_UUID       "6e400005-b5a3-f393-e0a9-e50e24dcca9e" // Phone -> Watch, WRITE (4-byte unix seconds, little-endian)
#define CHAR_DEVICE_INFO_UUID     "6e400006-b5a3-f393-e0a9-e50e24dcca9e" // Watch -> Phone, READ

// ---------- ASCII command layer (added on top of the binary protocol
// above; nothing above this line changes). Human-readable text commands
// in, human-readable text responses out — same GATT service, two new
// characteristics, so existing clients using the binary protocol are
// completely unaffected. ----------
#define CHAR_ASCII_CMD_UUID       "6e400007-b5a3-f393-e0a9-e50e24dcca9e" // Phone -> Watch, WRITE (text command, e.g. "PING")
#define CHAR_ASCII_RESP_UUID      "6e400008-b5a3-f393-e0a9-e50e24dcca9e" // Watch -> Phone, NOTIFY (text response, e.g. "PONG")
#define ASCII_CMD_MAX_LEN         64   // longest accepted incoming text command (rejects anything longer)
#define ASCII_RESP_MAX_LEN        220  // fixed response buffer, never heap-allocated

BLEServer*         pMorphixServer      = nullptr;
BLECharacteristic* pCharNotificationTx = nullptr;
BLECharacteristic* pCharCommandRx      = nullptr;
BLECharacteristic* pCharStatusTx       = nullptr;
BLECharacteristic* pCharTimeSync       = nullptr;
BLECharacteristic* pCharDeviceInfo     = nullptr;
BLECharacteristic* pCharAsciiCmd       = nullptr;
BLECharacteristic* pCharAsciiResp      = nullptr;

volatile bool phoneLinkConnected = false;

// ---------- time sync ----------
// No RTC/NTP required: the phone writes its current unix time once on
// connect (and again whenever it likes); we remember millis() at that
// moment and derive "now" from the offset. Falls back to "not synced"
// (0) if the phone hasn't sent one yet this session.
volatile bool morphixTimeSynced = false;
uint32_t phoneEpochBaseSec = 0;
unsigned long epochBaseMillis = 0;

uint32_t currentUnixSeconds() {
  if (!morphixTimeSynced) return 0;
  return phoneEpochBaseSec + (uint32_t)((millis() - epochBaseMillis) / 1000UL);
}

// ---------- fragment reassembly (single in-flight message; MORPHIX only
// ever has one phone connected at a time) ----------
#define NOTIF_REASSEMBLY_MAX      400   // fixed buffer, never heap-allocated
#define NOTIF_REASSEMBLY_TIMEOUT_MS 5000
#define MAX_FRAGMENTS_PER_MESSAGE 32    // sanity cap — anything claiming more is rejected as malformed

uint8_t  notifReassemblyBuf[NOTIF_REASSEMBLY_MAX];
int      notifReassemblyLen = 0;
uint8_t  notifReassemblyMsgId = 0;
uint8_t  notifReassemblyExpectedFragments = 0;
uint8_t  notifReassemblyReceivedFragments = 0;
bool     notifReassemblyActive = false;
unsigned long notifReassemblyStartMs = 0;

void resetReassembly() {
  notifReassemblyActive = false;
  notifReassemblyLen = 0;
  notifReassemblyReceivedFragments = 0;
}

// Parses a fully-reassembled notification payload:
//   [0]        appLen
//   [1..]      app bytes
//   [+0]       titleLen
//   [+1..]     title bytes
//   [+0]       textLen
//   [+1..]     text bytes
//   [+0]       idLen
//   [+1..]     id bytes
//   [+0..+3]   timestampSec (uint32, little-endian; 0 = "use device sync time")
// Every offset is bounds-checked against `len` before it's read — a
// malformed/truncated payload is dropped, never read out of bounds.
void parseAndQueueNotification(const uint8_t* buf, int len) {
  int off = 0;
  char app[NOTIF_APP_LEN] = "";
  char titleBuf[NOTIF_TITLE_LEN] = "";
  char textBuf[NOTIF_TEXT_LEN] = "";
  char idBuf[NOTIF_ID_LEN] = "";

  #define READ_FIELD(dst, dstCap) do { \
    if (off >= len) return; \
    uint8_t fLen = buf[off++]; \
    if (off + fLen > len) return; \
    int copyLen = min((int)fLen, (int)(dstCap) - 1); \
    memcpy(dst, buf + off, copyLen); \
    dst[copyLen] = 0; \
    off += fLen; \
  } while (0)

  READ_FIELD(app, NOTIF_APP_LEN);
  READ_FIELD(titleBuf, NOTIF_TITLE_LEN);
  READ_FIELD(textBuf, NOTIF_TEXT_LEN);
  READ_FIELD(idBuf, NOTIF_ID_LEN);
  #undef READ_FIELD

  uint32_t ts = 0;
  if (off + 4 <= len) {
    ts = (uint32_t)buf[off] | ((uint32_t)buf[off+1] << 8) |
         ((uint32_t)buf[off+2] << 16) | ((uint32_t)buf[off+3] << 24);
  }
  if (ts == 0) ts = currentUnixSeconds(); // fall back to our own synced clock

  if (app[0] == 0) strcpy(app, "UNKNOWN");
  pushNotificationToQueue(idBuf, app, titleBuf, textBuf, ts);
}

// A phone -> watch command packet (PKT_TYPE_COMMAND): payload[0]=cmdId,
// payload[1]=payloadLen, payload[2..]=command-specific data. Handled
// immediately and cheaply — no reassembly needed, these are always tiny.
void handleIncomingCommand(const uint8_t* payload, int len) {
  if (len < 2) return; // need at least cmdId + payloadLen
  uint8_t cmdId = payload[0];
  uint8_t plLen = payload[1];
  if (2 + plLen > len) return; // malformed length, drop safely
  const uint8_t* pl = payload + 2;

  switch (cmdId) {
    case CMD_PING:
      // No-op reply needed; the phone can tell it's alive from the
      // GATT connection itself.
      break;

    case CMD_STATUS_UPDATE: {
      if (plLen < 1) break;
      uint8_t flags = pl[0];
      connData.notifAccessEnabled = (flags & 0x01) != 0;
      if (plLen >= 2) {
        uint8_t batt = pl[1];
        connData.phoneBatteryPct = (batt <= 100) ? (int)batt : -1;
      }
      needsRedraw = true; // see note above — currentScreen isn't declared yet at this point in the file
      break;
    }

    case CMD_CLEAR_NOTIFICATIONS:
      notifCount = 0;
      notifIndex = 0;
      needsRedraw = true; // see note above — currentScreen isn't declared yet at this point in the file
      break;

    case CMD_MARK_READ: {
      if (plLen == 0 || plLen >= NOTIF_ID_LEN) break;
      char idBuf[NOTIF_ID_LEN];
      memcpy(idBuf, pl, plLen);
      idBuf[plLen] = 0;
      markNotificationReadById(idBuf);
      break;
    }

    case CMD_SYNC_NOTIFICATIONS:
    case CMD_GET_STATUS:
      sendStatusUpdate(true); // force an immediate STATUS_TX notify
      break;

    default:
      break; // unknown command — ignored safely, per spec
  }
}

// Handles one BLE write to CHAR_NOTIFICATION_TX — one fragment of either
// a notification or (for commands, always a single "fragment") a command.
// Header (7 bytes): version, type, msgId, fragIndex, fragTotal, payloadLen(u16 LE)
class NotificationTxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    // getValue() returns Arduino String on newer arduino-esp32 BLE cores
    // (and std::string on older ones); c_str()/length() work for both,
    // so this avoids depending on which one is installed.
    String v = ch->getValue();
    const uint8_t* data = (const uint8_t*)v.c_str();
    int len = (int)v.length();
    if (len < 7) return; // too short to even hold a header — reject

    uint8_t version    = data[0];
    uint8_t type       = data[1];
    uint8_t msgId       = data[2];
    uint8_t fragIndex  = data[3];
    uint8_t fragTotal  = data[4];
    uint16_t payloadLen = (uint16_t)data[5] | ((uint16_t)data[6] << 8);

    if (version != PKT_VERSION) return;
    if (7 + payloadLen > len) return;                 // declared length doesn't match actual — reject
    if (fragTotal == 0 || fragTotal > MAX_FRAGMENTS_PER_MESSAGE) return; // impossible fragment count — reject
    if (fragIndex >= fragTotal) return;

    const uint8_t* payload = data + 7;

    if (type == PKT_TYPE_COMMAND) {
      handleIncomingCommand(payload, payloadLen);
      return;
    }
    if (type != PKT_TYPE_NOTIFICATION) return; // unknown type — ignore safely

    if (fragIndex == 0) {
      resetReassembly();
      notifReassemblyActive = true;
      notifReassemblyMsgId = msgId;
      notifReassemblyExpectedFragments = fragTotal;
      notifReassemblyStartMs = millis();
    }

    // Fragments must belong to the message currently in progress and
    // arrive in order — anything else aborts the reassembly rather than
    // risk splicing two different messages together.
    if (!notifReassemblyActive || msgId != notifReassemblyMsgId ||
        fragIndex != notifReassemblyReceivedFragments) {
      resetReassembly();
      return;
    }

    if (notifReassemblyLen + payloadLen > NOTIF_REASSEMBLY_MAX) {
      // Would overflow the fixed buffer — drop the whole message rather
      // than truncate silently mid-field.
      resetReassembly();
      return;
    }

    memcpy(notifReassemblyBuf + notifReassemblyLen, payload, payloadLen);
    notifReassemblyLen += payloadLen;
    notifReassemblyReceivedFragments++;

    if (notifReassemblyReceivedFragments == notifReassemblyExpectedFragments) {
      parseAndQueueNotification(notifReassemblyBuf, notifReassemblyLen);
      resetReassembly();
    }
  }
};

// CHAR_TIME_SYNC: phone writes 4 bytes, unix seconds, little-endian.
class TimeSyncCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String v = ch->getValue();
    if (v.length() < 4) return; // malformed — ignore
    const uint8_t* d = (const uint8_t*)v.c_str();
    phoneEpochBaseSec = (uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                        ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    epochBaseMillis = millis();
    morphixTimeSynced = true;
  }
};

// ---------- ASCII command layer: text builders ----------
// Each writes into a caller-owned fixed buffer (no heap allocation).
// Unavailable numeric values are always shown as "--", never fabricated
// — the mock-data audit in the spec applies here too.

void buildAsciiStatus(char* out, size_t cap) {
  int hr   = heartRateValid ? getStableHeartRate() : -1;
  int spo2 = spo2Valid      ? getStableSpO2()      : -1;
  char hrStr[8], spo2Str[8];
  if (hr >= 0) snprintf(hrStr, sizeof(hrStr), "%d", hr); else strcpy(hrStr, "--");
  if (spo2 >= 0) snprintf(spo2Str, sizeof(spo2Str), "%d", spo2); else strcpy(spo2Str, "--");

  snprintf(out, cap,
    "MORPHIX ONLINE\n"
    "BLE CONNECTED\n"
    "BMP280 %s\n"
    "MAX30102 %s\n"
    "MPU6050 %s\n"
    "STEPS=%ld\n"
    "HR=%s\n"
    "SPO2=%s",
    bmp280Ready ? "LIVE" : "OFFLINE",
    max30102Ready ? "LIVE" : "OFFLINE",
    mpu6050Ready ? "LIVE" : "OFFLINE",
    stepCount, hrStr, spo2Str);
}

void buildAsciiSensors(char* out, size_t cap) {
  char tempStr[12], pressStr[12], hrStr[8], spo2Str[8], pitchStr[10], rollStr[10];

  if (bmp280Ready) snprintf(tempStr, sizeof(tempStr), "%.1f", realTemperatureC); else strcpy(tempStr, "--");
  if (bmp280Ready) snprintf(pressStr, sizeof(pressStr), "%.1f", realPressureHpa); else strcpy(pressStr, "--");
  if (heartRateValid) snprintf(hrStr, sizeof(hrStr), "%d", getStableHeartRate()); else strcpy(hrStr, "--");
  if (spo2Valid) snprintf(spo2Str, sizeof(spo2Str), "%d", getStableSpO2()); else strcpy(spo2Str, "--");
  if (mpu6050Ready) fmtSigned(mpuState.pitch, pitchStr, 1); else strcpy(pitchStr, "--");
  if (mpu6050Ready) fmtSigned(mpuState.roll, rollStr, 1); else strcpy(rollStr, "--");

  snprintf(out, cap,
    "TEMP=%s;PRESS=%s;HR=%s;SPO2=%s;STEPS=%ld;MOTION=%s;PITCH=%s;ROLL=%s",
    tempStr, pressStr, hrStr, spo2Str, stepCount,
    mpu6050Ready ? (motionIsMoving ? "MOVING" : "STABLE") : "--",
    pitchStr, rollStr);
}

void buildAsciiDeviceInfo(char* out, size_t cap) {
  snprintf(out, cap,
    "Device:\nMORPHIX\n\n"
    "MCU:\nXIAO ESP32-S3\n\n"
    "Firmware:\n%s\n\n"
    "BLE:\nCONNECTED\n\n"
    "Sensors:\nBMP280 %s\nMAX30102 %s\nMPU6050 %s",
    FIRMWARE_VERSION,
    bmp280Ready ? "LIVE" : "OFFLINE",
    max30102Ready ? "LIVE" : "OFFLINE",
    mpu6050Ready ? "LIVE" : "OFFLINE");
}

void buildAsciiTime(char* out, size_t cap) {
  if (morphixTimeSynced) {
    snprintf(out, cap, "TIME=%lu", (unsigned long)currentUnixSeconds());
  } else {
    strcpy(out, "TIME=UNSYNCED");
  }
}

// Handles one write to CHAR_ASCII_CMD: a short text command, e.g. "PING".
// Responds (when the command produces a reply) via CHAR_ASCII_RESP notify.
// Unknown commands are ignored safely — no response, no crash — per spec.
// Cheap and bounded: no dynamic allocation, no blocking, safe to run
// directly on the BLE callback context like every other callback here.
class AsciiCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String v = ch->getValue();
    if (v.length() == 0 || v.length() > ASCII_CMD_MAX_LEN) return; // reject empty/oversized

    char cmd[ASCII_CMD_MAX_LEN + 1];
    int n = min((int)v.length(), ASCII_CMD_MAX_LEN);
    memcpy(cmd, v.c_str(), n);
    cmd[n] = 0;
    while (n > 0 && (cmd[n-1] == '\n' || cmd[n-1] == '\r' || cmd[n-1] == ' ')) cmd[--n] = 0;

    char resp[ASCII_RESP_MAX_LEN] = "";

    if (strcmp(cmd, "PING") == 0) {
      strcpy(resp, "PONG");
    } else if (strcmp(cmd, "GET_STATUS") == 0) {
      buildAsciiStatus(resp, sizeof(resp));
    } else if (strcmp(cmd, "GET_SENSORS") == 0) {
      buildAsciiSensors(resp, sizeof(resp));
    } else if (strcmp(cmd, "GET_DEVICE_INFO") == 0) {
      buildAsciiDeviceInfo(resp, sizeof(resp));
    } else if (strcmp(cmd, "GET_TIME") == 0) {
      buildAsciiTime(resp, sizeof(resp));
    } else if (strcmp(cmd, "CLEAR_NOTIFICATIONS") == 0) {
      notifCount = 0;
      notifIndex = 0;
      needsRedraw = true; // currentScreen isn't declared yet at this point in the file
      strcpy(resp, "OK");
    } else if (strcmp(cmd, "SYNC_REQUEST") == 0) {
      sendCommandToPhone(WCMD_REQUEST_SYNC, nullptr, 0); // reuse existing binary wake-up-and-resync command
      strcpy(resp, "SYNC_REQUESTED");
    } else if (strncmp(cmd, "MARK_READ=", 10) == 0 && n > 10) {
      markNotificationReadById(cmd + 10);
      strcpy(resp, "OK");
    } else {
      return; // unknown command — ignored safely, no response sent
    }

    if (pCharAsciiResp != nullptr) {
      pCharAsciiResp->setValue((uint8_t*)resp, strlen(resp));
      pCharAsciiResp->notify();
    }
  }
};

// Server-level connect/disconnect. Only the plain single-BLEServer*
// overload is used here (rather than the esp_ble_gatts_cb_param_t
// variant some library versions add) so this compiles against the
// widest range of ESP32 BLE Arduino core versions.
class MorphixServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* srv) override {
    phoneLinkConnected = true;
    connData.phoneConnected = true;
    phoneState = PHONE_CONNECTED;
    Serial.println("Phone link: CONNECTED");
    needsRedraw = true; // see note above — currentScreen isn't declared yet at this point in the file
  }
  void onDisconnect(BLEServer* srv) override {
    phoneLinkConnected = false;
    connData.phoneConnected = false;
    connData.notifAccessEnabled = false;
    connData.phoneBatteryPct = -1;
    phoneState = PHONE_NOT_PAIRED;
    resetReassembly();
    Serial.println("Phone link: DISCONNECTED");
    needsRedraw = true; // see note above — currentScreen isn't declared yet at this point in the file
    BLEDevice::startAdvertising(); // this library stops advertising on connect; resume it
  }
};

// Sends a small watch -> phone command over CHAR_COMMAND_RX (format:
// [0]=PKT_VERSION [1]=cmdId [2]=payloadLen [3..]=payload). payload may be
// nullptr/0-length for commands that carry no data (PING, REQUEST_SYNC).
void sendCommandToPhone(uint8_t cmdId, const uint8_t* payload, uint8_t payloadLen) {
  if (!phoneLinkConnected || pCharCommandRx == nullptr) return;
  if (payloadLen > 32) payloadLen = 32; // safety cap, matches notification id length headroom
  uint8_t buf[3 + 32];
  buf[0] = PKT_VERSION;
  buf[1] = cmdId;
  buf[2] = payloadLen;
  if (payload != nullptr && payloadLen > 0) memcpy(buf + 3, payload, payloadLen);
  pCharCommandRx->setValue(buf, 3 + payloadLen);
  pCharCommandRx->notify();
}

// Sends a compact status snapshot over CHAR_STATUS_TX. Called on an
// interval and on meaningful change, never every loop() — per the
// "do not send commands repeatedly every loop" requirement.
unsigned long lastStatusNotifyMs = 0;
const unsigned long STATUS_NOTIFY_INTERVAL_MS = 3000;

void sendStatusUpdate(bool force) {
  if (!phoneLinkConnected || pCharStatusTx == nullptr) return;
  unsigned long now = millis();
  if (!force && (now - lastStatusNotifyMs) < STATUS_NOTIFY_INTERVAL_MS) return;
  lastStatusNotifyMs = now;

  uint8_t buf[8];
  buf[0] = PKT_VERSION;
  buf[1] = (uint8_t)notifCount;
  buf[2] = (uint8_t)unreadNotificationCount();
  uint32_t uptimeSec = now / 1000UL;
  buf[3] = (uint8_t)(uptimeSec & 0xFF);
  buf[4] = (uint8_t)((uptimeSec >> 8) & 0xFF);
  buf[5] = (uint8_t)((uptimeSec >> 16) & 0xFF);
  buf[6] = (uint8_t)((uptimeSec >> 24) & 0xFF);
  buf[7] = connData.wifiConnected ? 1 : 0;
  pCharStatusTx->setValue(buf, sizeof(buf));
  pCharStatusTx->notify();
}

// Called once per loop() (see serviceBLE() in the loop skeleton). Only
// does cheap, non-blocking housekeeping.
void serviceMorphixBle() {
  if (notifReassemblyActive && millis() - notifReassemblyStartMs > NOTIF_REASSEMBLY_TIMEOUT_MS) {
    Serial.println("Notification reassembly timed out — dropping partial message");
    resetReassembly();
  }
  if (phoneLinkConnected) sendStatusUpdate(false);
}

// Brings up the MORPHIX BLE GATT server. Called once from setup(), after
// BLEDevice::init("MORPHIX") and the existing BLEScan setup above — both
// roles share the one BLEDevice::init() call.
void initMorphixBleServer() {
  pMorphixServer = BLEDevice::createServer();
  pMorphixServer->setCallbacks(new MorphixServerCallbacks());

  BLEService* pService = pMorphixServer->createService(MORPHIX_SERVICE_UUID);

  pCharNotificationTx = pService->createCharacteristic(
      CHAR_NOTIFICATION_TX_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCharNotificationTx->setCallbacks(new NotificationTxCallbacks());

  pCharCommandRx = pService->createCharacteristic(
      CHAR_COMMAND_RX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharCommandRx->addDescriptor(new BLE2902());

  pCharStatusTx = pService->createCharacteristic(
      CHAR_STATUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharStatusTx->addDescriptor(new BLE2902());

  pCharTimeSync = pService->createCharacteristic(
      CHAR_TIME_SYNC_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCharTimeSync->setCallbacks(new TimeSyncCallbacks());

  pCharDeviceInfo = pService->createCharacteristic(
      CHAR_DEVICE_INFO_UUID, BLECharacteristic::PROPERTY_READ);
  pCharDeviceInfo->setValue("MORPHIX|ESP32-S3|" FIRMWARE_VERSION);

  // ASCII command layer — same service, two additional characteristics.
  pCharAsciiCmd = pService->createCharacteristic(
      CHAR_ASCII_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCharAsciiCmd->setCallbacks(new AsciiCommandCallbacks());

  pCharAsciiResp = pService->createCharacteristic(
      CHAR_ASCII_RESP_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharAsciiResp->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising* pAdvertising = pMorphixServer->getAdvertising();
  pAdvertising->addServiceUUID(MORPHIX_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("MORPHIX BLE GATT server advertising");
}

// Formats a float with an explicit leading sign, e.g. "+0.02" / "-0.14".
void fmtSigned(float v, char* out, int decimals) {
  char tmp[12];
  dtostrf(v, 0, decimals, tmp);
  if (tmp[0] != '-') {
    out[0] = '+';
    strcpy(out + 1, tmp);
  } else {
    strcpy(out, tmp);
  }
}

// ---------- GESTURE SYSTEM (label helpers; enum + settings flags now
// live in section 6c since the real gesture engine needs them earlier
// in the file — see GestureType/gestureControlEnabled/motionWakeEnabled) ----------

// NOTE: takes a plain int (not GestureSensitivity) and casts internally —
// same Arduino auto-prototype quirk worked around in goToScreen() above.
const char* sensitivityLabel(int sArg) {
  GestureSensitivity s = (GestureSensitivity)sArg;
  switch (s) {
    case SENS_LOW:  return "LOW";
    case SENS_HIGH: return "HIGH";
    default:        return "MED";
  }
}

GestureType lastGesture = GESTURE_NONE;
char lastActionLabel[16] = "-";

// NOTE: takes a plain int (not GestureType) and casts internally — same
// Arduino auto-prototype quirk worked around in goToScreen() above.
const char* gestureLabel(int gArg) {
  GestureType g = (GestureType)gArg;
  switch (g) {
    case GESTURE_TILT_LEFT:  return "TILT LEFT";
    case GESTURE_TILT_RIGHT: return "TILT RIGHT";
    case GESTURE_TILT_UP:    return "TILT UP";
    case GESTURE_TILT_DOWN:  return "TILT DOWN";
    case GESTURE_SHAKE:      return "SHAKE";
    case GESTURE_TWIST_CW:   return "TWIST CW";
    case GESTURE_TWIST_CCW:  return "TWIST CCW";
    default:                 return "NONE";
  }
}

// ============================================================
// 7. UI CONSTANTS / LAYOUT GRID
// ============================================================
#define HEADER_H   18
#define FOOTER_H   18
#define CONTENT_Y  HEADER_H
#define CONTENT_H  (SCR_H - HEADER_H - FOOTER_H)   // 92
#define CONTENT_BOTTOM (HEADER_H + CONTENT_H)      // 110
#define MARGIN     6

// ---- Rounded-UI look (MORPHIX visual refresh) ----
// Pure presentation constants — no screen logic, sensor reads, button
// handling, or redraw timing depends on these. Only the shape helpers
// below (drawCardOutline/drawRowHighlight/drawPill/drawTextBoldC etc.)
// use them.
#define UI_CARD_RADIUS   8   // outer screen bezel + content cards
#define UI_ROW_RADIUS    5   // selected menu row highlight
#define UI_PILL_RADIUS   6   // footer action pill / badges

// ============================================================
// 8. TEXT / LAYOUT HELPERS
// ============================================================
uint16_t textWidthOf(const char* s, uint8_t size) {
  int16_t x1, y1; uint16_t w, h;
  tft.setTextSize(size);
  tft.getTextBounds((char*)s, 0, 0, &x1, &y1, &w, &h);
  return w;
}

void drawTextL(const char* s, int16_t x, int16_t y, uint8_t size, uint16_t color) {
  tft.setTextSize(size);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(s);
}

void drawTextR(const char* s, int16_t xEnd, int16_t y, uint8_t size, uint16_t color) {
  uint16_t w = textWidthOf(s, size);
  drawTextL(s, xEnd - w, y, size, color);
}

void drawTextC(const char* s, int16_t xStart, int16_t xEnd, int16_t y, uint8_t size, uint16_t color) {
  uint16_t w = textWidthOf(s, size);
  int16_t x = xStart + ((xEnd - xStart) - (int16_t)w) / 2;
  drawTextL(s, x, y, size, color);
}

void drawDivider(int16_t y, uint16_t color = COL_BORDER) {
  tft.drawFastHLine(MARGIN, y, SCR_W - 2 * MARGIN, color);
}

// Centered text drawn twice (1px horizontal offset) to fake a bold/heavier
// weight for hero numerals (home clock, HR, step count) without needing a
// second font. Same bounding box math as drawTextC, so it's safe to use
// anywhere drawTextC was used — including inside the once-a-second partial
// redraw functions that already clear a full-width band before repainting.
void drawTextBoldC(const char* s, int16_t xStart, int16_t xEnd, int16_t y, uint8_t size, uint16_t color) {
  uint16_t w = textWidthOf(s, size);
  int16_t x = xStart + ((xEnd - xStart) - (int16_t)w) / 2;
  drawTextL(s, x, y, size, color);
  drawTextL(s, x + 1, y, size, color);
}

// Change-detection helper for dynamic regions: compares a freshly
// formatted display string against the value that was last actually
// painted to the glass and copies it in if different. Comparing the
// already-rounded/formatted string (not the raw sensor float) is what
// gives us "sensible rounding" for free — e.g. 30.401/30.402/30.403
// all format to "30.4" and never trigger a redraw against each other.
bool displayValueChanged(char* cache, size_t cacheSize, const char* freshVal) {
  if (strncmp(cache, freshVal, cacheSize) == 0) return false;
  strncpy(cache, freshVal, cacheSize - 1);
  cache[cacheSize - 1] = 0;
  return true;
}

// ============================================================
// 9. HEADER
// ============================================================
// left/right are always the physical UP / DOWN hints.
void drawHeader(const char* title) {
  tft.fillRect(0, 0, SCR_W, HEADER_H, COL_BG);
  drawTextL("UP",   4, 5, 1, COL_SECONDARY);
  drawTextR("DOWN", SCR_W - 4, 5, 1, COL_SECONDARY);
  drawTextC(title, 26, SCR_W - 30, 5, 1, COL_PRIMARY);
  tft.drawFastHLine(0, HEADER_H - 1, SCR_W, COL_BORDER);
  // Short bright teal accent centered under the title — a small glow-line
  // cue instead of a flat full-width rule, echoing the accent look used
  // throughout without changing where anything is positioned.
  uint16_t tW = textWidthOf(title, 1);
  int16_t accentW = min((int16_t)tW, (int16_t)(SCR_W - 60));
  int16_t accentX = 26 + (((SCR_W - 30) - 26) - accentW) / 2;
  tft.drawFastHLine(accentX, HEADER_H - 1, accentW, COL_PRIMARY);
}

// ============================================================
// 10. FOOTER
// ============================================================
// left is always BACK; right label is contextual per screen.
void drawFooter(const char* rightLabel, uint16_t rightColor = COL_PRIMARY) {
  int16_t y0 = SCR_H - FOOTER_H;
  tft.drawFastHLine(0, y0, SCR_W, COL_BORDER);
  tft.fillRect(0, y0 + 1, SCR_W, FOOTER_H - 1, COL_BG);
  drawTextL("BACK", 4, y0 + 5, 1, COL_SECONDARY);

  // Actionable labels (anything colored other than the "disabled/info"
  // COL_SECONDARY) get a small filled rounded pill, like a real button —
  // matches every other draw call site exactly as before (same string,
  // same color meaning), just a different shape underneath the text.
  if (rightLabel != NULL && rightLabel[0] != 0 && rightColor != COL_SECONDARY) {
    uint16_t w = textWidthOf(rightLabel, 1);
    int16_t padX = 5;
    int16_t pillW = w + padX * 2;
    int16_t pillH = 12;
    int16_t pillX = SCR_W - 4 - pillW;
    int16_t pillY = y0 + (FOOTER_H - pillH) / 2;
    tft.fillRoundRect(pillX, pillY, pillW, pillH, pillH / 2, rightColor);
    drawTextL(rightLabel, pillX + padX, pillY + (pillH - 8) / 2, 1, COL_BG);
  } else {
    drawTextR(rightLabel, SCR_W - 4, y0 + 5, 1, rightColor);
  }
}

// ============================================================
// 11. CARDS / COMPONENTS
// ============================================================
void drawBatteryIcon(int16_t x, int16_t y, int pct) {
  // body 12x7, nub 2x3
  uint16_t bodyColor = (pct <= 15) ? COL_CRIT : (pct <= 30 ? COL_WARN : COL_TEXT);
  tft.drawRoundRect(x, y, 12, 7, 1, bodyColor);
  tft.fillRect(x + 12, y + 2, 2, 3, bodyColor);
  int fillW = map(constrain(pct, 0, 100), 0, 100, 0, 10);
  if (fillW > 0) tft.fillRect(x + 1, y + 1, fillW, 5, bodyColor);
}

void drawStatusDot(int16_t x, int16_t y, bool ok) {
  // Live/ok state gets a soft halo ring (two concentric circles) for a
  // "glowing" cue; anything else stays a plain flat dot.
  if (ok) tft.drawCircle(x, y, 4, COL_PRIMARY_DIM);
  tft.fillCircle(x, y, 2, ok ? COL_GOOD : COL_CRIT);
}

void drawProgressBar(int16_t x, int16_t y, int16_t w, int16_t h, int pct, uint16_t color) {
  int16_t r = h / 2;
  tft.drawRoundRect(x, y, w, h, r, COL_BORDER);
  int fillW = map(constrain(pct, 0, 100), 0, 100, 0, w - 2);
  if (fillW > 0) {
    int16_t fr = max(1, (h - 2) / 2);
    tft.fillRoundRect(x + 1, y + 1, fillW, h - 2, fr, color);
  }
}

void drawBadge(const char* text, int16_t xStart, int16_t xEnd, int16_t y, int16_t h, uint16_t color) {
  tft.drawRoundRect(xStart, y, xEnd - xStart, h, h / 2, color);
  drawTextC(text, xStart, xEnd, y + (h - 8) / 2, 1, color);
}

// Shared rounded highlight box for a selected row — used by drawMenuRow()
// below plus the SETTINGS and MOTION SETTINGS row lists. Always clears the
// full row band first (fixes a pre-existing redraw quirk where a
// previously-selected row's highlight could linger after the selection
// moved away, since same-screen redraws never fillScreen() first).
void drawRowHighlight(int16_t x, int16_t y, int16_t w, int16_t h, bool selected) {
  tft.fillRect(0, y, SCR_W, h, COL_BG);
  if (selected) {
    tft.fillRoundRect(x, y, w, h, UI_ROW_RADIUS, COL_PANEL);
    tft.drawRoundRect(x, y, w, h, UI_ROW_RADIUS, COL_PRIMARY);
  }
}

// A generic 3-row selectable menu list, shared by CONNECTIVITY and
// MOTION so both hub screens look and behave the same way. Each row
// shows a label on the left and a short colored status word on the
// right; the highlighted row gets a rounded panel + border.
void drawMenuRow(int16_t y, int16_t rowH, const char* label, bool selected,
                  const char* statusText, uint16_t statusColor) {
  drawRowHighlight(MARGIN - 2, y + 2, SCR_W - 2 * (MARGIN - 2), rowH - 4, selected);
  drawTextL(label, MARGIN + 4, y + (rowH - 8) / 2, 1, selected ? COL_PRIMARY : COL_TEXT);
  if (statusText != NULL) {
    drawTextR(statusText, SCR_W - MARGIN - 2, y + (rowH - 8) / 2, 1, statusColor);
  }
}

// Thin rounded bezel around the whole screen — the "rounded card" look.
// Purely decorative outline (stroke only, never a fill), so it never
// covers or interferes with any content. Must be called LAST in each
// draw*Screen(), after drawHeader()/drawFooter(), since those two fill
// their own bands with COL_BG and would otherwise erase the corners.
void drawScreenFrame() {
  tft.drawRoundRect(0, 0, SCR_W, SCR_H, UI_CARD_RADIUS, COL_BORDER);
}

// ============================================================
// 11b. STORAGE / FILE MANAGER SUBSYSTEM (SD card)
// ============================================================
// Added as a self-contained subsystem so it's easy to strip back out
// later. Behaves like another MorPhix "hub" screen (SCR_STORAGE, which
// already existed as a top-level ScreenID slot) with its own small
// internal state machine, the same way SCR_WIFI/SCR_BLUETOOTH already
// manage wifiScreenState/bleScreenState internally instead of using
// separate top-level ScreenIDs. Nothing here runs on a timer or in the
// main loop() on its own — the SD card is only ever touched in direct
// response to a button press inside Storage (see handleStorageInput()).
//
// !!! HARDWARE CAVEAT — READ BEFORE FLASHING !!!
// Per the wiring notes above SD_CS (D0) is the SAME physical pin as
// TFT_RST, and SD_MISO (D9) is the SAME physical pin as the UP button.
// The comment where those are #defined states the board has isolation
// (a level-shifted SD module with its own CS/MISO) making this safe —
// that has NOT been verified here against real continuity on the
// physical board. If there is in fact no isolation:
//   - Every time SD_CS is pulled low, TFT_RST is pulled low too, which
//     hardware-resets the ST7735 controller. The SD protocol toggles
//     CS multiple times during a single directory listing or file
//     read, so this can happen several times per Storage action.
//   - storageSdEndSession() below defends against this unconditionally
//     (full tft.initR() + forced full redraw after any SD session), so
//     the display will always recover and the rest of MorPhix will
//     keep working — but expect a visible flash/redraw each time
///    Storage touches the card.
//   - The UP button (D9) may misread, or an SD read may get corrupted
//     data, if UP happens to be physically held down during an SD
//     session — storageSdBeginSession() detaches the UP interrupt for
//     the duration of the session to reduce (not fully eliminate) this.
// The clean permanent fix, if testing shows these ARE a bare shared
// node: tie TFT RST to 3V3 and construct the display with rst=-1 (the
// Adafruit_ST7735 library falls back to a software reset command),
// which frees D0 for SD_CS alone. That's a wiring change, so it is
// intentionally NOT made automatically here — see the delivered
// analysis notes.

#define STORAGE_NAME_LEN     40
#define STORAGE_PATH_LEN     160
#define STORAGE_MAX_ENTRIES  40   // fixed-size cache per directory listing; extra entries are hidden, not crashed on

enum StorageErrState { STORAGE_ERR_NONE, STORAGE_ERR_NO_CARD, STORAGE_ERR_INIT_FAILED };
StorageErrState storageError = STORAGE_ERR_NO_CARD;
bool storageMounted = false;
unsigned long storageLastInitAttemptMs = 0;
const unsigned long STORAGE_REINIT_COOLDOWN_MS = 5000; // don't hammer SD.begin() if the card is missing

enum StorageScreenState {
  STORAGE_SCR_HOME,      // Documents / Images / Music / Videos / Received / All Files / Wireless Transfer
  STORAGE_SCR_BROWSER,   // folder listing
  STORAGE_SCR_INFO,      // filename/type/size only, for unsupported/non-viewable files
  STORAGE_SCR_TEXT,      // scrolling text viewer
  STORAGE_SCR_IMAGE,     // BMP/JPEG image viewer
  STORAGE_SCR_VIDEO,     // ===== MORPHIX STORAGE ===== MJPEG/AVI video player
  STORAGE_SCR_WIRELESS   // ===== MORPHIX STORAGE ===== Wi-Fi file-transfer server control screen
};
StorageScreenState storageScreenState = STORAGE_SCR_HOME;

// ===== MORPHIX STORAGE ===== 7th entry (WIRELESS TRANSFER) added — see
// the STORAGE_HOME_WIRELESS_INDEX special case in handleStorageInput(),
// which opens STORAGE_SCR_WIRELESS instead of browsing a folder for it.
#define STORAGE_HOME_COUNT 7
#define STORAGE_HOME_WIRELESS_INDEX 6
const char* STORAGE_HOME_LABELS[STORAGE_HOME_COUNT] = {
  "DOCUMENTS", "IMAGES", "MUSIC", "VIDEOS", "RECEIVED", "ALL FILES", "WIRELESS TRANSFER"
};
const char* STORAGE_HOME_PATHS[STORAGE_HOME_COUNT]  = {
  "/MORPHIX/FILES/DOCUMENTS", "/MORPHIX/FILES/IMAGES", "/MORPHIX/FILES/MUSIC",
  "/MORPHIX/FILES/VIDEOS", "/MORPHIX/RECEIVED", "/MORPHIX/FILES", "" // last slot unused (wireless has no folder)
};
int storageHomeIndex = 0;

struct StorageEntry {
  char name[STORAGE_NAME_LEN];
  bool isDir;
  uint32_t size;
};
StorageEntry storageEntries[STORAGE_MAX_ENTRIES];
int storageEntryCount = 0;
int storageEntryIndex = 0;
int storageEntryScroll = 0;
bool storageListTruncated = false;

char storageCurrentPath[STORAGE_PATH_LEN] = "/MORPHIX/FILES";
// Simple one-level "back to home" flag: BROWSER only ever returns to
// STORAGE_SCR_HOME (per the spec's flat DOCUMENTS/IMAGES/.../ALL FILES
// structure — sub-folders inside a category browse further via SELECT
// but BACK always steps back exactly one path segment, see
// handleStorageInput()).

char storageSelName[STORAGE_NAME_LEN] = "";
char storageSelPath[STORAGE_PATH_LEN] = "";
uint32_t storageSelSize = 0;

// ---- text viewer state ----
#define STORAGE_TEXT_LINE_LEN   34   // matches ~34 chars at text size 1 across SCR_W minus margins
#define STORAGE_TEXT_LINES      6    // visible lines in the content area
char storageTextLines[STORAGE_TEXT_LINES][STORAGE_TEXT_LINE_LEN];
int storageTextLineCount = 0;        // lines actually loaded this page (<= STORAGE_TEXT_LINES)
uint32_t storageTextFileOffset = 0;  // byte offset in the file where the CURRENT page starts
bool storageTextAtEnd = false;
// ===== MORPHIX STORAGE ===== small back-page stack so UP can scroll
// backward through pages already visited, not just DOWN/SELECT forward —
// the underlying reader is still forward-only/line-based (never loads the
// whole file), this just remembers where each page it already displayed
// began so it can re-seek and re-read that page on request.
#define STORAGE_TEXT_PAGE_STACK 32
uint32_t storageTextPageStack[STORAGE_TEXT_PAGE_STACK];
int storageTextPageStackDepth = 0;
uint32_t storageTextPageStartOffset = 0; // offset where the CURRENTLY DISPLAYED page began

// ---- image viewer state ----
bool storageImageLoaded = false;
bool storageImageTooLarge = false;
// ===== MORPHIX STORAGE ===== set for a recognized-but-unhandled image
// variant (compressed/palette BMP, progressive JPEG, etc) — distinct
// from storageImageTooLarge (genuinely can't fit/allocate safely) and
// from a plain load failure (corrupt file), which stays as IMAGE ERROR.
bool storageImageUnsupported = false;
// Set by openSelectedFile() when a BMP/JPEG needs painting; consumed once,
// right after the normal redraw has drawn the image screen's chrome
// (header/footer/frame), so the pixel data isn't immediately wiped out
// again — see the STORAGE_SCR_IMAGE handling added to loop() below.
bool storagePendingImageDraw = false;

// ===== MORPHIX STORAGE ===== cheap MP3 duration estimate (CBR-only,
// computed once in openSelectedFile() — see storageMp3EstimateDuration()).
char storageMp3DurationStr[12] = "";
bool storageMp3HasDuration = false;

// ===== MORPHIX STORAGE ===== MJPEG/AVI VIDEO PLAYER STATE
// See the "VIDEO PLAYER (MJPEG/AVI)" section below (after the image
// viewer) for full implementation notes, including why MP4/H.264 itself
// is not realistically decodable on this hardware.
enum VideoPlayState { VIDEO_IDLE, VIDEO_PLAYING, VIDEO_PAUSED, VIDEO_ENDED, VIDEO_ERROR };
VideoPlayState storageVideoState = VIDEO_IDLE;
struct AviInfo {
  bool valid;
  uint32_t moviStart;   // file offset of the first byte AFTER the "movi" fourCC+size+listType
  uint32_t moviEnd;     // file offset one past the last byte of movi content
  uint16_t width, height;
  uint32_t frameDelayMs;
};
AviInfo storageAviInfo;
// Manual prototype: Arduino's auto-prototype generator inserts function
// prototypes near the top of the .ino, BEFORE this struct is defined,
// which causes "'AviInfo' has not been declared" errors. Declaring the
// prototype here (after AviInfo exists) makes Arduino skip generating
// its own broken one.
bool storageParseAviHeader(File &f, AviInfo &info);
File storageVideoFile;
uint32_t storageVideoPos = 0;          // read cursor within the movi region
uint8_t* storageVideoFrameBuf = nullptr;
size_t storageVideoFrameCap = 0;       // allocated size of storageVideoFrameBuf
int16_t storageVideoDrawX = 0, storageVideoDrawY = 0;
bool storagePendingVideoOpen = false;  // same deferred-until-after-chrome-draw pattern as storagePendingImageDraw
unsigned long storageVideoLastFrameMs = 0;

// ===== MORPHIX STORAGE ===== WIRELESS FILE-TRANSFER WEB SERVER STATE
// See the "WIRELESS WEB FILE MANAGER" section below. Off by default —
// only ever started by the user selecting START SERVER on
// STORAGE_SCR_WIRELESS, and stopped either explicitly (STOP SERVER) or
// implicitly never (it's a deliberately-backgrounded service, like BLE,
// so it keeps serving files even if the user leaves the Storage screen —
// only an explicit STOP SERVER or reboot ends it).
WebServer storageWebServer(80);
bool storageServerActive = false;
char storageApSsid[24] = "";
IPAddress storageApIp;
File storageUploadFile;
bool storageUploadOk = false;
char storageUploadPath[STORAGE_PATH_LEN] = "";

// ------------------------------------------------------------
// SD / TFT SPI ARBITRATION
// ------------------------------------------------------------
void storageSdBeginSession() {
  detachInterrupt(digitalPinToInterrupt(PIN_UP));
  // ===== MORPHIX STORAGE ===== PIN_BACK (D6) isn't itself a pin shared
  // with the SD bus, but D0 (TFT_RST) IS the same physical pin as SD_CS
  // (see the hardware caveat above), and every SD_CS toggle during a
  // file read was the one interrupt still left live throughout the
  // whole session — leaving BACK the one button that could pick up a
  // spurious edge from that shared-pin reset activity and bounce the
  // image viewer straight back to the file browser mid-load. Detaching
  // it for the SD session, same as PIN_UP already is, reduces (not
  // fully eliminates, per the caveat above) that risk.
  detachInterrupt(digitalPinToInterrupt(PIN_BACK));
  digitalWrite(TFT_CS, HIGH); // make sure TFT is deselected before touching the shared bus
  SPI.end();
  SPI.begin(TFT_SCK, SD_MISO, TFT_MOSI, -1); // -1: SD.h drives SD_CS itself per-transfer
}

void storageSdEndSession() {
  SPI.end();
  // Restore the bus exactly as setup() configured it for the TFT
  // (MISO disabled so D9 stays a clean button input).
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  pinMode(PIN_UP, INPUT_PULLUP);
  btnUp.lastReading = (digitalRead(PIN_UP) == LOW);
  btnUp.lastChangeMs = millis();
  attachInterrupt(digitalPinToInterrupt(PIN_UP), isrButtonUp, CHANGE);

  // ===== MORPHIX STORAGE ===== re-arm PIN_BACK the same way — re-read
  // its current physical level so a real press held through the SD
  // session is still honored once the interrupt is live again, rather
  // than assuming an edge that never actually happened.
  pinMode(PIN_BACK, INPUT_PULLUP);
  btnBack.lastReading = (digitalRead(PIN_BACK) == LOW);
  btnBack.lastChangeMs = millis();
  attachInterrupt(digitalPinToInterrupt(PIN_BACK), isrButtonBack, CHANGE);

  // Defensive re-init in case SD_CS toggling also reset the TFT
  // controller (see hardware caveat above). Cheap/harmless if the
  // pins turn out to be truly isolated.
  tft.initR(INITR_GREENTAB);
  tft.setRotation(1);
  tft.setSPISpeed(16000000);
  lastRenderedScreen = (ScreenID)-1; // force a full fillScreen()+repaint next loop
  needsRedraw = true;
}

// ------------------------------------------------------------
// INIT / DIRECTORY SETUP
// ------------------------------------------------------------
bool storageAvailable() {
  return storageMounted && storageError == STORAGE_ERR_NONE;
}

// Creates a directory only if it doesn't already exist. Never deletes
// or touches anything already present.
void storageEnsureDir(const char* path) {
  if (!SD.exists(path)) SD.mkdir(path);
}

void createStorageDirectories() {
  storageEnsureDir("/MORPHIX");
  storageEnsureDir("/MORPHIX/FILES");
  storageEnsureDir("/MORPHIX/FILES/DOCUMENTS");
  storageEnsureDir("/MORPHIX/FILES/IMAGES");
  storageEnsureDir("/MORPHIX/FILES/MUSIC");
  storageEnsureDir("/MORPHIX/FILES/VIDEOS");
  storageEnsureDir("/MORPHIX/RECEIVED");
  storageEnsureDir("/MORPHIX/SYSTEM");
}

// Non-blocking-ish: only actually attempts SD.begin() at boot and
// (rate-limited) whenever the user re-enters Storage while a card is
// reported missing — never from loop() on a timer, so a missing card
// never repeatedly stalls the rest of MorPhix.
void initStorage() {
  unsigned long now = millis();
  if (storageMounted) return;
  if (now - storageLastInitAttemptMs < STORAGE_REINIT_COOLDOWN_MS && storageLastInitAttemptMs != 0) return;
  storageLastInitAttemptMs = now;

  storageSdBeginSession();
  bool ok = SD.begin(SD_CS, SPI, 16000000);
  if (!ok) {
    storageSdEndSession();
    storageMounted = false;
    storageError = STORAGE_ERR_NO_CARD;
    Serial.println("Storage: SD card not found");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    SD.end();
    storageSdEndSession();
    storageMounted = false;
    storageError = STORAGE_ERR_NO_CARD;
    Serial.println("Storage: SD card type NONE");
    return;
  }

  createStorageDirectories();
  storageSdEndSession();
  storageMounted = true;
  storageError = STORAGE_ERR_NONE;
  Serial.println("Storage: SD mounted, MorPhix directories ready");
}

// ------------------------------------------------------------
// FILE HELPERS
// ------------------------------------------------------------
const char* getFileExtension(const char* filename) {
  const char* dot = strrchr(filename, '.');
  if (!dot || dot == filename) return "";
  return dot + 1;
}

bool storageExtEquals(const char* ext, const char* candidate) {
  if (strlen(ext) != strlen(candidate)) return false;
  for (size_t i = 0; i < strlen(ext); i++) {
    if (tolower((unsigned char)ext[i]) != tolower((unsigned char)candidate[i])) return false;
  }
  return true;
}

void formatFileSize(uint32_t bytes, char* out, size_t outLen) {
  if (bytes >= 1024UL * 1024UL) {
    snprintf(out, outLen, "%.1f MB", bytes / (1024.0f * 1024.0f));
  } else if (bytes >= 1024UL) {
    snprintf(out, outLen, "%.1f KB", bytes / 1024.0f);
  } else {
    snprintf(out, outLen, "%lu B", (unsigned long)bytes);
  }
}

enum StorageFileKind : uint8_t {
  SF_FOLDER, SF_TEXT, SF_IMAGE_BMP, SF_IMAGE_JPEG, SF_MP3, SF_MP4,
  SF_VIDEO_AVI /* ===== MORPHIX STORAGE ===== */, SF_PDF, SF_XLS, SF_OTHER
};

StorageFileKind classifyFile(const char* name, bool isDir) {
  if (isDir) return SF_FOLDER;
  const char* ext = getFileExtension(name);
  if (storageExtEquals(ext, "txt") || storageExtEquals(ext, "log") ||
      storageExtEquals(ext, "csv") || storageExtEquals(ext, "json") ||
      storageExtEquals(ext, "md"))                                          return SF_TEXT;
  if (storageExtEquals(ext, "bmp"))                                         return SF_IMAGE_BMP;
  if (storageExtEquals(ext, "jpg") || storageExtEquals(ext, "jpeg"))        return SF_IMAGE_JPEG;
  if (storageExtEquals(ext, "mp3"))                                         return SF_MP3;
  if (storageExtEquals(ext, "mp4"))                                         return SF_MP4;
  if (storageExtEquals(ext, "avi"))                                         return SF_VIDEO_AVI; // ===== MORPHIX STORAGE =====
  if (storageExtEquals(ext, "pdf"))                                         return SF_PDF;
  if (storageExtEquals(ext, "xls") || storageExtEquals(ext, "xlsx"))        return SF_XLS;
  return SF_OTHER;
}

const char* storageKindLabel(StorageFileKind k) {
  switch (k) {
    case SF_FOLDER:      return "FOLDER";
    case SF_TEXT:        return "TEXT";
    case SF_IMAGE_BMP:    return "BMP";
    case SF_IMAGE_JPEG:  return "JPG";
    case SF_MP3:          return "MP3";
    case SF_MP4:          return "MP4";
    case SF_VIDEO_AVI:    return "AVI"; // ===== MORPHIX STORAGE =====
    case SF_PDF:          return "PDF";
    case SF_XLS:          return "XLS/XLSX";
    default:               return "FILE";
  }
}

// Loads (or reloads) the entry list for storageCurrentPath. One SD
// session for the whole listing, not one per entry.
void storageRefreshListing() {
  storageEntryCount = 0;
  storageEntryIndex = 0;
  storageEntryScroll = 0;
  storageListTruncated = false;
  if (!storageAvailable()) return;

  storageSdBeginSession();
  File dir = SD.open(storageCurrentPath);
  if (dir && dir.isDirectory()) {
    File f = dir.openNextFile();
    while (f) {
      const char* nm = f.name();
      // SD.h may return either "name.ext" or "/full/path/name.ext"
      // depending on core version — keep just the leaf name.
      const char* slash = strrchr(nm, '/');
      const char* leaf = slash ? slash + 1 : nm;
      if (leaf[0] != 0 && strcmp(leaf, ".") != 0 && strcmp(leaf, "..") != 0) {
        if (storageEntryCount < STORAGE_MAX_ENTRIES) {
          strncpy(storageEntries[storageEntryCount].name, leaf, STORAGE_NAME_LEN - 1);
          storageEntries[storageEntryCount].name[STORAGE_NAME_LEN - 1] = 0;
          storageEntries[storageEntryCount].isDir = f.isDirectory();
          storageEntries[storageEntryCount].size = f.isDirectory() ? 0 : (uint32_t)f.size();
          storageEntryCount++;
        } else {
          storageListTruncated = true;
        }
      }
      f.close();
      f = dir.openNextFile();
    }
    dir.close();
  }
  storageSdEndSession();
}

// ------------------------------------------------------------
// TEXT VIEWER — chunk/line based, never loads the whole file into RAM
// ------------------------------------------------------------
// Fills storageTextLines[] starting at byte offset storageTextFileOffset,
// advances storageTextFileOffset to just past what it read, and sets
// storageTextAtEnd once the file is exhausted. One SD session per page.
void storageLoadTextPage() {
  storageTextLineCount = 0;
  storageTextAtEnd = true;
  if (!storageAvailable()) return;

  storageSdBeginSession();
  File f = SD.open(storageSelPath, FILE_READ);
  if (f) {
    f.seek(storageTextFileOffset);
    int lineIdx = 0;
    int col = 0;
    while (lineIdx < STORAGE_TEXT_LINES && f.available()) {
      int c = f.read();
      if (c < 0) break;
      if (c == '\r') continue;
      if (c == '\n' || col >= STORAGE_TEXT_LINE_LEN - 1) {
        storageTextLines[lineIdx][col] = 0;
        lineIdx++;
        col = 0;
        if (c != '\n') {
          // wrapped, not a real newline — the char that triggered the
          // wrap still needs to be placed on the next line
          if (lineIdx < STORAGE_TEXT_LINES) storageTextLines[lineIdx][col++] = (char)c;
        }
        continue;
      }
      // Keep it to plain printable ASCII on a 128x160 1-bit font.
      if (c >= 32 && c < 127) storageTextLines[lineIdx][col++] = (char)c;
    }
    if (col > 0 && lineIdx < STORAGE_TEXT_LINES) {
      storageTextLines[lineIdx][col] = 0;
      lineIdx++;
    }
    storageTextLineCount = lineIdx;
    storageTextFileOffset = f.position();
    storageTextAtEnd = !f.available();
    f.close();
  }
  storageSdEndSession();
}

// ===== MORPHIX STORAGE ===== forward/back paging wrappers around
// storageLoadTextPage() — see storageTextPageStack above.
void storageTextGoForward() {
  if (storageTextAtEnd) return;
  if (storageTextPageStackDepth < STORAGE_TEXT_PAGE_STACK) {
    storageTextPageStack[storageTextPageStackDepth++] = storageTextPageStartOffset;
  }
  storageTextPageStartOffset = storageTextFileOffset; // page about to load starts where the last one ended
  storageLoadTextPage();
}

void storageTextGoBack() {
  if (storageTextPageStackDepth == 0) return; // already at the first page
  storageTextPageStartOffset = storageTextPageStack[--storageTextPageStackDepth];
  storageTextFileOffset = storageTextPageStartOffset;
  storageLoadTextPage();
}

// ------------------------------------------------------------
// IMAGE VIEWER — BMP: self-contained parser below, no extra library.
// ===== MORPHIX STORAGE ===== JPEG/JPG is now ALSO decoded and viewable
// (previously detected/listed but not decoded) — see storageDrawJpeg()
// just below, which uses the TJpg_Decoder library added for this.
// ------------------------------------------------------------
uint16_t storageRead16(File &f) {
  uint16_t v = f.read();
  v |= ((uint16_t)f.read()) << 8;
  return v;
}
uint32_t storageRead32(File &f) {
  uint32_t v = f.read();
  v |= ((uint32_t)f.read()) << 8;
  v |= ((uint32_t)f.read()) << 16;
  v |= ((uint32_t)f.read()) << 24;
  return v;
}

// Draws a 24-bit or 8-bit uncompressed BMP, row-by-row (never the whole
// file in RAM), scaled to CONTAIN-fit inside maxW x maxH and centered.
// Unlike the old version — which picked independent stepX/stepY per
// axis, silently stretching non-square images — this uses a single
// uniform scale for both axes so the aspect ratio is always preserved.
// Sets storageImageUnsupported for BMP variants this minimal parser
// doesn't decode (compressed/RLE, palette, unusual bit depths).
// storageImageTooLarge is now only a hard safety cap against absurd or
// corrupt header dimensions that would overflow the row-seek math below
// — not a "doesn't fit the screen" rejection, since that's exactly what
// the contain-fit scaling handles.
bool storageDrawBmp(const char* path, int16_t x0, int16_t y0, int16_t maxW, int16_t maxH) {
  storageImageTooLarge = false;
  storageImageUnsupported = false;
  bool ok = false;

  storageSdBeginSession();
  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.printf("[IMG] %s: could not open\n", path);
  } else if (storageRead16(f) != 0x4D42) { // "BM"
    Serial.printf("[IMG] %s: bad BMP magic\n", path);
  } else {
    storageRead32(f);           // file size
    storageRead32(f);           // reserved
    uint32_t dataOffset = storageRead32(f);
    storageRead32(f);           // DIB header size
    int32_t bmpW = (int32_t)storageRead32(f);
    int32_t bmpHraw = (int32_t)storageRead32(f);
    storageRead16(f);           // planes
    uint16_t bpp = storageRead16(f);
    uint32_t compression = storageRead32(f);

    bool topDown = bmpHraw < 0;
    int32_t bmpH = topDown ? -bmpHraw : bmpHraw;
    Serial.printf("[IMG] %s: %ldx%ld bpp=%u compression=%lu\n",
                   path, (long)bmpW, (long)bmpH, bpp, (unsigned long)compression);

    if (bmpW <= 0 || bmpH <= 0 || bmpW > 4096 || bmpH > 4096) {
      // Sanity cap on the header itself (corrupt/absurd dimensions),
      // not a rejection of ordinary large photos.
      storageImageTooLarge = true;
    } else if (compression != 0 || (bpp != 24 && bpp != 16)) {
      // Compressed (RLE) or palette/other-bit-depth BMP — recognized
      // format, but this minimal parser only understands uncompressed
      // 24/16bpp, same coverage as before the patch.
      storageImageUnsupported = true;
    } else {
      // ---- contain fit: one uniform scale for both axes ----
      float scale = min((float)maxW / (float)bmpW, (float)maxH / (float)bmpH);
      if (scale > 1.0f) scale = 1.0f; // never upscale a small image
      int16_t outW = max((int16_t)1, (int16_t)(bmpW * scale));
      int16_t outH = max((int16_t)1, (int16_t)(bmpH * scale));
      int16_t drawX = x0 + (maxW - outW) / 2;
      int16_t drawY = y0 + (maxH - outH) / 2;

      uint8_t bytesPerPx = bpp / 8;
      uint32_t rowSize = ((uint32_t)bmpW * bytesPerPx + 3) & ~3u; // rows padded to 4 bytes

      for (int16_t oy = 0; oy < outH; oy++) {
        int32_t srcRow = (int32_t)(oy / scale);
        if (srcRow >= bmpH) srcRow = bmpH - 1;
        int32_t fileRow = topDown ? srcRow : (bmpH - 1 - srcRow);
        uint32_t rowBase = dataOffset + (uint32_t)fileRow * rowSize;
        for (int16_t ox = 0; ox < outW; ox++) {
          int32_t srcCol = (int32_t)(ox / scale);
          if (srcCol >= bmpW) srcCol = bmpW - 1;
          f.seek(rowBase + (uint32_t)srcCol * bytesPerPx);
          uint16_t color565;
          if (bpp == 24) {
            uint8_t b = f.read(), g = f.read(), r = f.read();
            color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
          } else { // 16bpp, assume 565 already
            color565 = storageRead16(f);
          }
          tft.drawPixel(drawX + ox, drawY + oy, color565);
        }
      }
      ok = true;
    }
  }
  if (f) f.close();
  storageSdEndSession();
  return ok;
}

// ===== MORPHIX STORAGE =====
// ------------------------------------------------------------
// JPEG IMAGE VIEWER (TJpg_Decoder) — streams the file block-by-block via
// TJpgDec's own filesystem reader (drawSdJpg), the same way tjpgd always
// works: it never holds the whole compressed file, let alone the whole
// decoded bitmap, in RAM at once — only a small internal MCU (up to
// 16x16) buffer per callback.
//
// CONTAIN-FIT SCALING — fixes "IMAGE TOO LARGE" on ordinary large
// photos (e.g. a 58 KB, 1920x1080 phone JPEG). TJpgDec's own built-in
// downscale only offers four fixed factors: 1/1, 1/2, 1/4, 1/8. Even at
// the max 1/8 factor, a 1920x1080 source is still 240x135 decoded —
// bigger than MorPhix's ~148x88 content area — which is exactly why the
// old code gave up at that point and showed "IMAGE TOO LARGE", despite
// the file itself being tiny. The fix does two scaling passes:
//   1. Pick the largest TJpgDec built-in factor (1/2/4/8) that still
//      decodes to AT LEAST the final on-screen size, so the decoder
//      itself does as much of the size reduction as possible and we
//      never ask it to hand us more pixels than we need.
//   2. Finish the remaining reduction ourselves with simple nearest-
//      neighbor decimation, applied pixel-by-pixel as TJpgDec's own
//      callback delivers each small MCU block — so the only extra RAM
//      this needs is two small lookup tables (a few hundred int16_t
//      entries at most: which decoder-space row/column maps to which
//      on-screen row/column, or -1 to skip it), never a framebuffer
//      sized to the photo.
// A single "contain" scale (min of the two axis ratios) drives both
// passes, so the aspect ratio is always preserved and the image is
// centered on any leftover margin rather than stretched. When the
// decoder's own factor already lands exactly on the target size (native
// resolution, or a photo that happens to divide evenly), step 2 is
// skipped entirely and MCU blocks are blitted whole, same as before.
// ------------------------------------------------------------
static int16_t storageJpegDecW = 0, storageJpegDecH = 0;     // decoder-space (post TJpgDec scale) size
static int16_t storageJpegDrawX = 0, storageJpegDrawY = 0;   // top-left of the fitted image on screen
static int16_t *storageJpegOutCol = nullptr;                 // [decoder-space col] -> screen-relative col, or -1 to skip
static int16_t *storageJpegOutRow = nullptr;                 // [decoder-space row] -> screen-relative row, or -1 to skip
static bool storageJpegNeedsResample = false;                // false => decoder output already IS the final size

bool storageJpegRenderCallback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  // storageDrawJpeg() passes (storageJpegDrawX, storageJpegDrawY) into
  // drawSdJpg() itself — same call convention the original, known-working
  // code used — so x,y here arrive already in absolute screen space.
  // Subtract that same offset back out to get 0-based decoder-space
  // coordinates for indexing storageJpegOutCol/Row below.
  int16_t gy0 = y - storageJpegDrawY;
  if (gy0 >= storageJpegDecH) return 0; // fully past the decoded image, stop early

  if (!storageJpegNeedsResample) {
    // Fast path: decoder output already IS the final size — blit each
    // whole MCU block in one call, exactly like the pre-fix code.
    if (y >= tft.height()) return 0;
    tft.drawRGBBitmap(x, y, bitmap, w, h);
    return 1;
  }

  // Resample path: draws only the pixels that survive nearest-neighbor
  // decimation — at most outW*outH draws across the whole image, i.e.
  // never more than the final on-screen pixel count.
  int16_t gx0 = x - storageJpegDrawX;
  for (uint16_t ly = 0; ly < h; ly++) {
    int16_t gy = gy0 + ly;
    if (gy < 0 || gy >= storageJpegDecH) continue;
    int16_t oy = storageJpegOutRow[gy];
    if (oy < 0) continue; // this decoder-space row isn't any output row's sample
    for (uint16_t lx = 0; lx < w; lx++) {
      int16_t gx = gx0 + lx;
      if (gx < 0 || gx >= storageJpegDecW) continue;
      int16_t ox = storageJpegOutCol[gx];
      if (ox < 0) continue;
      tft.drawPixel(storageJpegDrawX + ox, storageJpegDrawY + oy, bitmap[ly * w + lx]);
    }
  }
  return 1;
}

bool storageDrawJpeg(const char* path, int16_t x0, int16_t y0, int16_t maxW, int16_t maxH) {
  storageImageTooLarge = false;
  storageImageUnsupported = false;
  bool ok = false;

  storageSdBeginSession();
  // Using the SD-specific getSdJpgSize()/drawSdJpg() calls (rather than
  // the generic getFsJpgSize()/drawFsJpg(fs::FS&) variants) — both exist
  // in Bodmer's TJpg_Decoder, but the SD-specific ones have been present
  // since earlier library versions, so this is the safer choice given
  // MorPhix only ever decodes from the SD card here.
  uint16_t jw = 0, jh = 0;
  JRESULT hdrRes = TJpgDec.getSdJpgSize(&jw, &jh, path);
  Serial.printf("[IMG] %s hdr=%d %ux%u\n", path, (int)hdrRes, jw, jh);
  if (hdrRes == JDR_OK && jw > 0 && jh > 0) {
    // ---- contain fit: one uniform scale for both axes ----
    float fitScale = min((float)maxW / (float)jw, (float)maxH / (float)jh);
    if (fitScale > 1.0f) fitScale = 1.0f; // never upscale a small image
    int16_t targetW = max((int16_t)1, (int16_t)(jw * fitScale));
    int16_t targetH = max((int16_t)1, (int16_t)(jh * fitScale));

    // Largest built-in TJpgDec factor that still decodes to >= the
    // final target size, so the decimation pass below only ever
    // shrinks further, never has to invent pixels.
    uint8_t decScale = 8;
    while (decScale > 1 && (jw / decScale < targetW || jh / decScale < targetH)) decScale /= 2;
    int16_t decW = jw / decScale, decH = jh / decScale;

    storageJpegOutCol = new (std::nothrow) int16_t[decW];
    storageJpegOutRow = new (std::nothrow) int16_t[decH];
    if (!storageJpegOutCol || !storageJpegOutRow) {
      // Genuinely can't safely allocate even these small (few-hundred-
      // entry) lookup tables — a real "can't process safely" case.
      storageImageTooLarge = true;
    } else {
      for (int16_t i = 0; i < decW; i++) storageJpegOutCol[i] = -1;
      for (int16_t i = 0; i < decH; i++) storageJpegOutRow[i] = -1;
      // For each output column/row, mark which decoder-space column/row
      // is its nearest-neighbor source sample.
      for (int16_t ox = 0; ox < targetW; ox++) {
        int16_t srcCol = (int16_t)((int32_t)ox * decW / targetW);
        if (srcCol >= decW) srcCol = decW - 1;
        storageJpegOutCol[srcCol] = ox;
      }
      for (int16_t oy = 0; oy < targetH; oy++) {
        int16_t srcRow = (int16_t)((int32_t)oy * decH / targetH);
        if (srcRow >= decH) srcRow = decH - 1;
        storageJpegOutRow[srcRow] = oy;
      }

      storageJpegDecW = decW;
      storageJpegDecH = decH;
      storageJpegNeedsResample = (decW != targetW) || (decH != targetH);
      storageJpegDrawX = x0 + max((int16_t)0, (int16_t)((maxW - targetW) / 2));
      storageJpegDrawY = y0 + max((int16_t)0, (int16_t)((maxH - targetH) / 2));

      TJpgDec.setJpgScale(decScale);
      // Same call convention as the pre-fix code: pass the real, already-
      // centered draw position straight into drawSdJpg() rather than
      // (0,0) — the callback derives decoder-space coordinates by
      // subtracting it back out (see storageJpegRenderCallback above).
      JRESULT drawRes = TJpgDec.drawSdJpg(storageJpegDrawX, storageJpegDrawY, path);
      Serial.printf("[IMG] decScale=%u dec=%dx%d target=%dx%d drawRes=%d\n",
                     decScale, decW, decH, targetW, targetH, (int)drawRes);
      ok = (drawRes == JDR_OK);
    }

    delete[] storageJpegOutCol; storageJpegOutCol = nullptr;
    delete[] storageJpegOutRow; storageJpegOutRow = nullptr;
  } else if (hdrRes == JDR_FMT2 || hdrRes == JDR_FMT3) {
    // Header parsed enough to identify the JPEG variant (e.g.
    // progressive) as one tjpgd's baseline-only decoder doesn't support.
    storageImageUnsupported = true;
  }
  // Any other failure (JDR_FMT1 / JDR_INP / etc, or a 0x0 reported size)
  // falls through with ok == false and no flag set -> generic
  // "IMAGE ERROR", for a genuinely corrupt/unreadable file.
  storageSdEndSession();
  return ok;
}

// ------------------------------------------------------------
// MP3 — no audio hardware yet (see FEATURE 3 in the spec), so this
// stays metadata-only. Duration is a CHEAP estimate: read past any
// ID3v2 tag, find the first MPEG audio frame header, read its bitrate
// field, and divide file size by bitrate. Accurate for constant-bitrate
// (CBR) files, which is most consumer MP3s; VBR files will be off, but
// this never scans the whole file to compute an exact value.
// ------------------------------------------------------------
bool storageMp3EstimateDuration(const char* path, uint32_t fileSize, char* out, size_t outLen) {
  static const int MP3_V1L3_KBPS[16] = {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0};
  bool ok = false;
  if (fileSize == 0) return false;

  storageSdBeginSession();
  File f = SD.open(path, FILE_READ);
  if (f) {
    uint32_t dataStart = 0;
    uint8_t hdr[10];
    if (f.read(hdr, 10) == 10 && hdr[0] == 'I' && hdr[1] == 'D' && hdr[2] == '3') {
      uint32_t tagSize = ((uint32_t)(hdr[6] & 0x7F) << 21) | ((uint32_t)(hdr[7] & 0x7F) << 14) |
                          ((uint32_t)(hdr[8] & 0x7F) << 7)  |  (uint32_t)(hdr[9] & 0x7F);
      dataStart = 10 + tagSize;
    }
    f.seek(dataStart);
    // Scan a bounded window for a frame sync (0xFFEx) rather than the
    // whole file — cheap, and enough to find the first real frame.
    for (int tries = 0; tries < 4096 && f.available(); tries++) {
      int b0 = f.read();
      if (b0 != 0xFF) continue;
      int b1 = f.read();
      if (b1 < 0 || (b1 & 0xE0) != 0xE0) continue;
      int b2 = f.read();
      if (b2 < 0) break;
      int bitrateIdx = (b2 >> 4) & 0x0F;
      int kbps = (bitrateIdx > 0 && bitrateIdx < 15) ? MP3_V1L3_KBPS[bitrateIdx] : 0;
      if (kbps > 0) {
        uint32_t durSec = (uint32_t)((float)fileSize * 8.0f / (kbps * 1000.0f));
        snprintf(out, outLen, "%lu:%02lu", (unsigned long)(durSec / 60), (unsigned long)(durSec % 60));
        ok = true;
      }
      break;
    }
    f.close();
  }
  storageSdEndSession();
  return ok;
}

// ------------------------------------------------------------
// VIDEO PLAYER (MJPEG/AVI)
//
// Why not MP4 itself: practical MP4 files are H.264/HEVC — decoding
// those needs either a hardware video decoder block (the ESP32-S3 has
// none) or a full software H.264 decoder, which needs tens of MB of
// working memory and far more CPU than a 240 MHz Xtensa core can spend
// while still running the rest of MorPhix (sensors, BLE, UI). That is
// not realistic here, so MP4 files are correctly reported as
// unsupported (see the SF_MP4 branch in drawStorageInfoScreen()) rather
// than faked.
//
// What IS realistic: Motion-JPEG inside an AVI container. Each "frame"
// is just an independent JPEG image, decoded with the exact same
// TJpg_Decoder already used for the image viewer above — no video-codec
// math at all, just "decode a JPEG, wait, decode the next one." This is
// a well-established lightweight-embedded approach. To play a video on
// MorPhix, convert it once on a computer, e.g. with ffmpeg:
//   ffmpeg -i input.mp4 -vf scale=128:-2 -q:v 5 -c:v mjpeg output.avi
// then copy output.avi to /MORPHIX/FILES/VIDEOS (or upload it wirelessly
// — see the web file manager below).
//
// Parsing approach: a single lightweight scan of the RIFF/AVI header to
// find the "movi" data list (+ dimensions/frame rate from "avih" if
// present), then frame-by-frame forward-only chunk reads during
// playback — the file is never loaded into RAM, only one compressed
// frame at a time (capped, see storageVideoFrameCap). This does not
// handle nested "rec " LIST sub-chunks some interleaved AVIs use inside
// movi (those are simply skipped as opaque chunks) — the common
// ffmpeg-style single-stream MJPEG AVI produced by the command above
// does not use them.
// ------------------------------------------------------------
bool storageParseAviHeader(File &f, AviInfo &info) {
  info.valid = false;
  info.width = 128; info.height = 160; // safe fallback, overwritten by avih if present
  info.frameDelayMs = 66; // ~15 fps fallback

  uint8_t tag[4];
  if (f.read(tag, 4) != 4 || memcmp(tag, "RIFF", 4) != 0) return false;
  f.seek(f.position() + 4); // whole-file size, unused
  if (f.read(tag, 4) != 4 || memcmp(tag, "AVI ", 4) != 0) return false;

  for (int guard = 0; guard < 64 && f.available(); guard++) { // bounded: never loop forever on a malformed file
    uint8_t chunkId[4]; uint8_t sizeBytes[4];
    if (f.read(chunkId, 4) != 4 || f.read(sizeBytes, 4) != 4) break;
    uint32_t chunkSize = sizeBytes[0] | ((uint32_t)sizeBytes[1] << 8) | ((uint32_t)sizeBytes[2] << 16) | ((uint32_t)sizeBytes[3] << 24);
    uint32_t chunkDataPos = f.position();

    if (memcmp(chunkId, "LIST", 4) == 0) {
      uint8_t listType[4];
      f.read(listType, 4);
      if (memcmp(listType, "movi", 4) == 0) {
        info.moviStart = f.position();
        info.moviEnd = chunkDataPos + chunkSize;
        info.valid = true;
        return true; // found what we need — playback reads frames from here forward
      }
      if (memcmp(listType, "hdrl", 4) == 0) {
        // Look inside hdrl (bounded to this list's own size) for "avih".
        uint32_t hdrlEnd = chunkDataPos + chunkSize;
        while (f.position() + 8 <= hdrlEnd) {
          uint8_t innerId[4]; uint8_t innerSizeBytes[4];
          if (f.read(innerId, 4) != 4 || f.read(innerSizeBytes, 4) != 4) break;
          uint32_t innerSize = innerSizeBytes[0] | ((uint32_t)innerSizeBytes[1] << 8) |
                                ((uint32_t)innerSizeBytes[2] << 16) | ((uint32_t)innerSizeBytes[3] << 24);
          uint32_t innerDataPos = f.position();
          if (memcmp(innerId, "avih", 4) == 0 && innerSize >= 40) {
            uint8_t avih[40];
            f.read(avih, 40);
            uint32_t usecPerFrame = avih[0] | ((uint32_t)avih[1] << 8) | ((uint32_t)avih[2] << 16) | ((uint32_t)avih[3] << 24);
            uint32_t w = avih[32] | ((uint32_t)avih[33] << 8) | ((uint32_t)avih[34] << 16) | ((uint32_t)avih[35] << 24);
            uint32_t h = avih[36] | ((uint32_t)avih[37] << 8) | ((uint32_t)avih[38] << 16) | ((uint32_t)avih[39] << 24);
            if (usecPerFrame > 0) info.frameDelayMs = max((uint32_t)20, usecPerFrame / 1000); // clamp so a bad header can't spin at 0ms/frame
            if (w > 0 && w < 2000) info.width = (uint16_t)w;
            if (h > 0 && h < 2000) info.height = (uint16_t)h;
          }
          f.seek(innerDataPos + innerSize + (innerSize & 1));
        }
        f.seek(hdrlEnd); // continue the outer scan right after hdrl regardless of what was found inside
        continue;
      }
      // Some other LIST (e.g. "INFO") — skip past it and keep scanning.
      f.seek(chunkDataPos + chunkSize + (chunkSize & 1));
    } else {
      f.seek(chunkDataPos + chunkSize + (chunkSize & 1));
    }
  }
  return false; // never found a movi list — not a usable AVI for this player
}

bool storageOpenVideo(const char* path) {
  storageVideoState = VIDEO_ERROR;
  storageAviInfo.valid = false;

  storageSdBeginSession(); // held open for the WHOLE playback session, see storageCloseVideo()
  storageVideoFile = SD.open(path, FILE_READ);
  if (!storageVideoFile) { storageSdEndSession(); return false; }

  if (!storageParseAviHeader(storageVideoFile, storageAviInfo)) {
    storageVideoFile.close();
    storageSdEndSession();
    return false; // not a RIFF/AVI file, or no movi data — reported as VIDEO ERROR on screen
  }

  int16_t maxW = SCR_W - 2 * MARGIN, maxH = CONTENT_H - 4;
  uint8_t scale = 1;
  while ((storageAviInfo.width / scale > (uint16_t)maxW || storageAviInfo.height / scale > (uint16_t)maxH) && scale < 8) scale *= 2;
  TJpgDec.setJpgScale(scale);
  int16_t outW = storageAviInfo.width / scale, outH = storageAviInfo.height / scale;
  storageVideoDrawX = MARGIN + max((int16_t)0, (int16_t)((maxW - outW) / 2));
  storageVideoDrawY = CONTENT_Y + 2 + max((int16_t)0, (int16_t)((maxH - outH) / 2));

  // Frame buffer: one compressed MJPEG frame at a time, never the whole
  // video. Prefer PSRAM (the XIAO ESP32-S3 ships with 8MB PSRAM) so a
  // generous per-frame cap doesn't compete with the rest of MorPhix's
  // heap; fall back to a smaller heap-only cap if PSRAM isn't present.
  storageVideoFrameCap = psramFound() ? 24576 : 8192;
  if (!storageVideoFrameBuf) {
    storageVideoFrameBuf = (uint8_t*)(psramFound() ? ps_malloc(storageVideoFrameCap) : malloc(storageVideoFrameCap));
  }
  if (!storageVideoFrameBuf) {
    storageVideoFile.close();
    storageSdEndSession();
    return false;
  }

  storageVideoPos = storageAviInfo.moviStart;
  tft.fillRect(0, CONTENT_Y, SCR_W, CONTENT_H, COL_BG);
  storageVideoState = VIDEO_PAUSED; // opened successfully, waiting for PLAY (SELECT)
  return true;
}

// Decodes and draws exactly one frame, advancing storageVideoPos past it.
// Returns false at end-of-stream, on a read error, or if a chunk is too
// big for storageVideoFrameCap (skipped safely rather than crashing).
bool storageVideoNextFrame() {
  while (storageVideoPos + 8 <= storageAviInfo.moviEnd) {
    storageVideoFile.seek(storageVideoPos);
    uint8_t hdr[8];
    if (storageVideoFile.read(hdr, 8) != 8) return false;
    uint32_t chunkSize = hdr[4] | ((uint32_t)hdr[5] << 8) | ((uint32_t)hdr[6] << 16) | ((uint32_t)hdr[7] << 24);
    uint32_t advance = 8 + chunkSize + (chunkSize & 1);

    if (memcmp(hdr, "LIST", 4) == 0) {
      storageVideoPos += 8; // step past the LIST wrapper; its inner chunks are read as normal chunks next
      continue;
    }
    bool isVideoFrame = (hdr[2] == 'd' && hdr[3] == 'c'); // "00dc"/"01dc" = compressed video frame chunk
    if (isVideoFrame && chunkSize > 0 && chunkSize <= storageVideoFrameCap) {
      size_t got = storageVideoFile.read(storageVideoFrameBuf, chunkSize);
      storageVideoPos += advance;
      if (got != chunkSize) return false; // short read: truncated/corrupted file
      return (TJpgDec.drawJpg(storageVideoDrawX, storageVideoDrawY, storageVideoFrameBuf, chunkSize) == JDR_OK);
    }
    // Not a usable video frame (audio chunk, index chunk, or an
    // oversized/corrupt frame) — skip it and keep scanning forward.
    storageVideoPos += advance;
  }
  return false; // reached the end of the movi data
}

void storageCloseVideo() {
  if (storageVideoFile) storageVideoFile.close();
  if (storageVideoFrameBuf) { free(storageVideoFrameBuf); storageVideoFrameBuf = nullptr; }
  storageSdEndSession();
  storageVideoState = VIDEO_IDLE;
}

// ------------------------------------------------------------
// WIRELESS WEB FILE MANAGER
//
// A small local web server (WiFi Access Point + WebServer.h — both
// already built into the esp32 core, no cloud/internet involved at any
// point) so files can be added to the SD card over Wi-Fi instead of
// pulling the microSD card out. See startStorageServer()/
// stopStorageServer(), wired to STORAGE_SCR_WIRELESS below.
//
// Uploads/downloads are both streamed directly to/from the SD card
// (WebServer's own upload handler + streamFile()) — never buffered
// whole in RAM, so file size is limited only by the SD card itself.
// ------------------------------------------------------------
const char STORAGE_WEB_INDEX_HTML[] PROGMEM = R"HTMLPAGE(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MorPhix Storage</title>
<style>
body{font-family:sans-serif;background:#101418;color:#e6e6e6;margin:0;padding:16px}
h1{font-size:18px;margin:0 0 4px}
#path{color:#8fa;font-size:13px;margin-bottom:12px;word-break:break-all}
.row{display:flex;justify-content:space-between;align-items:center;padding:8px 6px;border-bottom:1px solid #2a2f36}
.row a{color:#7ad;text-decoration:none;flex:1}
.sz{color:#888;font-size:12px;margin:0 10px}
button{background:#2a2f36;color:#eee;border:1px solid #444;border-radius:4px;padding:4px 8px;cursor:pointer}
button.del{color:#f66}
#bar{display:flex;gap:8px;margin-bottom:12px;flex-wrap:wrap}
#msg{margin-top:10px;font-size:13px;color:#8fa}
</style></head><body>
<h1>MORPHIX STORAGE</h1>
<div id="path">/FILES</div>
<div id="bar">
  <input type="file" id="fileInput">
  <button onclick="doUpload()">Upload File</button>
  <button onclick="doMkdir()">Create Folder</button>
</div>
<div id="list"></div>
<div id="msg"></div>
<script>
let cur = "/MORPHIX/FILES";
function load(p){
  cur = p;
  document.getElementById('path').textContent = p;
  fetch('/api/list?path=' + encodeURIComponent(p)).then(r=>r.json()).then(j=>{
    let html = '';
    if (p !== '/MORPHIX/FILES') html += '<div class="row"><a href="#" onclick="load(cur.substring(0,cur.lastIndexOf(\'/\'))||\'/\');return false;">.. (up)</a></div>';
    j.entries.forEach(e=>{
      if (e.dir){
        html += '<div class="row"><a href="#" onclick="load(cur+\'/\'+\''+e.name+'\');return false;">'+e.name+'/</a><button class="del" onclick="doDelete(\''+e.name+'\',true)">DELETE</button></div>';
      } else {
        let kb=(e.size/1024).toFixed(1);
        html += '<div class="row"><a href="/api/download?path='+encodeURIComponent(cur+'/'+e.name)+'">'+e.name+'</a><span class="sz">'+kb+' KB</span><button class="del" onclick="doDelete(\''+e.name+'\',false)">DELETE</button></div>';
      }
    });
    document.getElementById('list').innerHTML = html || '<p>Empty folder</p>';
  });
}
function doUpload(){
  const f = document.getElementById('fileInput').files[0];
  if (!f){ msg('Choose a file first'); return; }
  const fd = new FormData(); fd.append('file', f);
  msg('Uploading ' + f.name + '...');
  fetch('/api/upload?path=' + encodeURIComponent(cur), {method:'POST', body:fd})
    .then(r=>r.text()).then(t=>{ msg(t); load(cur); })
    .catch(()=>msg('Upload failed (disconnected?)'));
}
function doMkdir(){
  const name = prompt('New folder name:');
  if (!name) return;
  fetch('/api/mkdir', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'path='+encodeURIComponent(cur+'/'+name)})
    .then(r=>r.text()).then(t=>{ msg(t); load(cur); });
}
function doDelete(name, isDir){
  const full = cur + '/' + name;
  if (!confirm('DELETE?\n\n' + name + (isDir ? ' (folder)' : ''))) return;
  fetch('/api/delete', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'path='+encodeURIComponent(full)+'&confirm=yes'})
    .then(r=>r.text()).then(t=>{ msg(t); load(cur); });
}
function msg(t){ document.getElementById('msg').textContent = t; }
load(cur);
</script></body></html>)HTMLPAGE";

const char* storageWebMimeType(const String &path) {
  String p = path; p.toLowerCase();
  if (p.endsWith(".html") || p.endsWith(".htm")) return "text/html";
  if (p.endsWith(".txt") || p.endsWith(".log") || p.endsWith(".md")) return "text/plain";
  if (p.endsWith(".json")) return "application/json";
  if (p.endsWith(".csv")) return "text/csv";
  if (p.endsWith(".jpg") || p.endsWith(".jpeg")) return "image/jpeg";
  if (p.endsWith(".bmp")) return "image/bmp";
  if (p.endsWith(".mp3")) return "audio/mpeg";
  if (p.endsWith(".mp4")) return "video/mp4";
  if (p.endsWith(".avi")) return "video/x-msvideo";
  if (p.endsWith(".pdf")) return "application/pdf";
  return "application/octet-stream";
}

// If path/name already exists, append " (1)", " (2)", ... before the
// extension (up to a bound) rather than silently overwriting — see the
// FEATURE 7 "existing filename" requirement. Never corrupts/overwrites
// an existing file.
void storageUniqueFilePath(const String &dir, const String &name, char* out, size_t outLen) {
  String base = name, ext = "";
  int dot = name.lastIndexOf('.');
  if (dot > 0) { base = name.substring(0, dot); ext = name.substring(dot); }

  snprintf(out, outLen, "%s/%s", dir.c_str(), name.c_str());
  for (int i = 1; i < 100 && SD.exists(out); i++) {
    snprintf(out, outLen, "%s/%s (%d)%s", dir.c_str(), base.c_str(), i, ext.c_str());
  }
}

void storageHandleWebRoot() {
  storageWebServer.send_P(200, "text/html", STORAGE_WEB_INDEX_HTML);
}

void storageHandleWebList() {
  String dir = storageWebServer.hasArg("path") ? storageWebServer.arg("path") : "/MORPHIX/FILES";
  String json = "{\"path\":\"" + dir + "\",\"entries\":[";
  storageSdBeginSession();
  File d = SD.open(dir);
  bool first = true;
  if (d && d.isDirectory()) {
    File f = d.openNextFile();
    while (f) {
      const char* nm = f.name();
      const char* slash = strrchr(nm, '/');
      const char* leaf = slash ? slash + 1 : nm;
      if (leaf[0] && strcmp(leaf, ".") != 0 && strcmp(leaf, "..") != 0) {
        if (!first) json += ",";
        first = false;
        json += "{\"name\":\"" + String(leaf) + "\",\"dir\":" + (f.isDirectory() ? "true" : "false") +
                ",\"size\":" + String((uint32_t)f.size()) + "}";
      }
      f.close();
      f = d.openNextFile();
    }
    d.close();
  }
  storageSdEndSession();
  json += "]}";
  storageWebServer.send(200, "application/json", json);
}

void storageHandleWebMkdir() {
  if (!storageWebServer.hasArg("path")) { storageWebServer.send(400, "text/plain", "Missing path"); return; }
  String path = storageWebServer.arg("path");
  if (path.indexOf("..") >= 0 || path.length() == 0) { storageWebServer.send(400, "text/plain", "Invalid folder name"); return; }
  storageSdBeginSession();
  bool ok = SD.exists(path) ? true : SD.mkdir(path); // creating an already-existing folder is not an error
  storageSdEndSession();
  storageWebServer.send(ok ? 200 : 500, "text/plain", ok ? "Folder created" : "Could not create folder");
}

void storageHandleWebDelete() {
  if (!storageWebServer.hasArg("path")) { storageWebServer.send(400, "text/plain", "Missing path"); return; }
  if (!storageWebServer.hasArg("confirm") || storageWebServer.arg("confirm") != "yes") {
    storageWebServer.send(400, "text/plain", "Confirmation required"); return; // never deletes without an explicit confirm
  }
  String path = storageWebServer.arg("path");
  storageSdBeginSession();
  File chk = SD.open(path);
  bool isDir = chk && chk.isDirectory();
  if (chk) chk.close();
  bool ok = isDir ? SD.rmdir(path) : SD.remove(path);
  storageSdEndSession();
  storageWebServer.send(ok ? 200 : 500, "text/plain", ok ? "Deleted" : "Delete failed (folder not empty?)");
}

void storageHandleWebDownload() {
  if (!storageWebServer.hasArg("path")) { storageWebServer.send(400, "text/plain", "Missing path"); return; }
  String path = storageWebServer.arg("path");
  storageSdBeginSession();
  File f = SD.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    storageSdEndSession();
    storageWebServer.send(404, "text/plain", "Not found");
    return;
  }
  storageWebServer.sendHeader("Content-Disposition", "attachment; filename=\"" + String(f.name()) + "\"");
  storageWebServer.streamFile(f, storageWebMimeType(path)); // streams in chunks internally — never whole-file-in-RAM
  f.close();
  storageSdEndSession();
}

// Called once per HTTP request lifecycle by WebServer's upload machinery
// (registered as the upload-callback of "/api/upload" in
// startStorageServer()) — writes each chunk straight to the SD card as
// it arrives, so the uploaded file is never buffered whole in RAM.
void storageHandleWebUploadChunk() {
  HTTPUpload &up = storageWebServer.upload();

  if (up.status == UPLOAD_FILE_START) {
    String dir = storageWebServer.hasArg("path") ? storageWebServer.arg("path") : "/MORPHIX/RECEIVED";
    String fname = up.filename;
    fname.replace("/", "_"); fname.replace("\\", "_"); fname.replace("..", "_"); // no path traversal via filename
    if (fname.length() == 0) fname = "upload.bin";

    storageSdBeginSession(); // held open for the whole upload — see the class comment above
    uint64_t freeBytes = (uint64_t)SD.totalBytes() - (uint64_t)SD.usedBytes();
    if (freeBytes < 8192) { // insufficient storage — refuse before writing anything
      storageUploadOk = false;
      storageUploadPath[0] = 0;
      storageSdEndSession(); // ===== MORPHIX STORAGE ===== must not leave the SD session open on this early-out
      return;
    }
    storageUniqueFilePath(dir, fname, storageUploadPath, sizeof(storageUploadPath));
    storageUploadFile = SD.open(storageUploadPath, FILE_WRITE);
    storageUploadOk = (bool)storageUploadFile;

  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (storageUploadOk && storageUploadFile) {
      size_t written = storageUploadFile.write(up.buf, up.currentSize);
      if (written != up.currentSize) storageUploadOk = false; // disk full / write error mid-transfer
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (storageUploadFile) storageUploadFile.close();
    storageSdEndSession();
    if (currentScreen == SCR_STORAGE) needsRedraw = true; // Received/etc. listing will show it next time it's opened

  } else if (up.status == UPLOAD_FILE_ABORTED) { // client disconnected mid-transfer
    if (storageUploadFile) storageUploadFile.close();
    if (!storageUploadOk && storageUploadPath[0]) SD.remove(storageUploadPath); // never leave a corrupt partial file behind
    storageSdEndSession();
  }
}

void storageHandleWebUploadDone() {
  storageWebServer.send(storageUploadOk ? 200 : 500, "text/plain", storageUploadOk ? "Upload complete" : "Upload failed");
}

void startStorageServer() {
  if (storageServerActive || !storageAvailable()) return;

  uint64_t chipId = ESP.getEfuseMac();
  snprintf(storageApSsid, sizeof(storageApSsid), "MORPHIX-%04X", (uint16_t)(chipId & 0xFFFF));

  // AP_STA (not AP-only) so the existing background Wi-Fi station
  // connection/auto-reconnect (see serviceWifiAutoConnect() in loop())
  // keeps working exactly as before — this only ADDS the local AP.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(storageApSsid);
  storageApIp = WiFi.softAPIP();

  storageWebServer.on("/", HTTP_GET, storageHandleWebRoot);
  storageWebServer.on("/api/list", HTTP_GET, storageHandleWebList);
  storageWebServer.on("/api/mkdir", HTTP_POST, storageHandleWebMkdir);
  storageWebServer.on("/api/delete", HTTP_POST, storageHandleWebDelete);
  storageWebServer.on("/api/download", HTTP_GET, storageHandleWebDownload);
  storageWebServer.on("/api/upload", HTTP_POST, storageHandleWebUploadDone, storageHandleWebUploadChunk);
  storageWebServer.begin();
  storageServerActive = true;
}

void stopStorageServer() {
  if (!storageServerActive) return;
  storageWebServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA); // restore normal station-only mode used by the rest of MorPhix
  storageServerActive = false;
}

// Pumps the web server — call every loop() iteration (see loop() below).
// A no-op (single boolean check) whenever the server isn't running, so
// this never costs anything while Wi-Fi transfer is off (the default).
void handleStorageWebRequest() {
  if (storageServerActive) storageWebServer.handleClient();
}

// ------------------------------------------------------------
// DRAWING
// ------------------------------------------------------------
void drawStorageHomeScreen() {
  drawHeader("STORAGE");

  if (storageError == STORAGE_ERR_NO_CARD) {
    drawTextC("SD CARD", 0, SCR_W, CONTENT_Y + 30, 2, COL_CRIT);
    drawTextC("NOT FOUND", 0, SCR_W, CONTENT_Y + 54, 2, COL_CRIT);
    drawFooter("", COL_SECONDARY);
    drawScreenFrame();
    return;
  }
  if (storageError == STORAGE_ERR_INIT_FAILED) {
    drawTextC("SD INIT FAILED", 0, SCR_W, CONTENT_Y + 40, 1, COL_CRIT);
    drawFooter("", COL_SECONDARY);
    drawScreenFrame();
    return;
  }

  // ===== MORPHIX STORAGE ===== rowH trimmed from 15->13 so all 7 entries
  // (the new WIRELESS TRANSFER row included) still fit inside CONTENT_H
  // without needing to scroll this short, fixed-length menu.
  const int rowH = 13;
  for (int i = 0; i < STORAGE_HOME_COUNT; i++) {
    int16_t y = CONTENT_Y + 2 + i * rowH;
    drawRowHighlight(MARGIN - 2, y, SCR_W - 2 * (MARGIN - 2), rowH - 1, i == storageHomeIndex);
    drawTextL(STORAGE_HOME_LABELS[i], MARGIN + 4, y + 2, 1, i == storageHomeIndex ? COL_PRIMARY : COL_TEXT);
  }

  drawFooter("OPEN");
  drawScreenFrame();
}

void drawStorageBrowserScreen() {
  // Title: last path segment of storageCurrentPath.
  const char* leaf = strrchr(storageCurrentPath, '/');
  leaf = leaf ? leaf + 1 : storageCurrentPath;
  drawHeader(leaf[0] ? leaf : "FILES");

  if (storageEntryCount == 0) {
    drawTextC("EMPTY FOLDER", 0, SCR_W, CONTENT_Y + 40, 1, COL_SECONDARY);
    drawFooter("", COL_SECONDARY);
    drawScreenFrame();
    return;
  }

  char posBuf[10];
  snprintf(posBuf, sizeof(posBuf), "%d/%d", storageEntryIndex + 1, storageEntryCount);
  drawTextR(posBuf, SCR_W - MARGIN, CONTENT_Y + 2, 1, COL_SECONDARY);

  int visible = min(storageEntryCount - storageEntryScroll, LIST_VISIBLE_ROWS);
  const int rowH = 20;
  for (int row = 0; row < visible; row++) {
    int i = storageEntryScroll + row;
    int16_t y = CONTENT_Y + 14 + row * rowH;
    bool sel = (i == storageEntryIndex);
    StorageEntry &e = storageEntries[i];

    drawTextL(sel ? ">" : " ", MARGIN, y, 1, COL_PRIMARY);
    char nameBuf[STORAGE_NAME_LEN + 2];
    if (e.isDir) snprintf(nameBuf, sizeof(nameBuf), "%s/", e.name);
    else         snprintf(nameBuf, sizeof(nameBuf), "%s", e.name);
    drawTextL(nameBuf, MARGIN + 10, y, 1, sel ? COL_PRIMARY : (e.isDir ? COL_PRIMARY_DIM : COL_TEXT));

    if (!e.isDir) {
      char szBuf[12];
      formatFileSize(e.size, szBuf, sizeof(szBuf));
      drawTextR(szBuf, SCR_W - MARGIN, y, 1, COL_SECONDARY);
    }
  }

  drawFooter("OPEN");
  drawScreenFrame();
}

void drawStorageInfoScreen() {
  drawHeader("FILE INFO");

  drawTextC(storageSelName, 0, SCR_W, CONTENT_Y + 20, 1, COL_TEXT);

  StorageFileKind kind = classifyFile(storageSelName, false);
  drawTextC(storageKindLabel(kind), 0, SCR_W, CONTENT_Y + 42, 2, COL_PRIMARY);

  char szBuf[16];
  formatFileSize(storageSelSize, szBuf, sizeof(szBuf));
  drawTextC(szBuf, 0, SCR_W, CONTENT_Y + 66, 1, COL_SECONDARY);

  if (kind == SF_MP3) {
    // ===== MORPHIX STORAGE ===== duration (cheap CBR estimate, computed
    // once in openSelectedFile() — see storageMp3EstimateDuration()) and
    // the exact "AUDIO HARDWARE / NOT AVAILABLE" wording from the spec.
    if (storageMp3HasDuration) {
      drawTextC(storageMp3DurationStr, 0, SCR_W, CONTENT_Y + 80, 1, COL_SECONDARY);
    }
    drawTextC("AUDIO HARDWARE", 0, SCR_W, CONTENT_Y + 92, 1, COL_WARN);
    drawTextC("NOT AVAILABLE", 0, SCR_W, CONTENT_Y + 102, 1, COL_WARN);
  } else if (kind == SF_MP4) {
    // ===== MORPHIX STORAGE ===== MP4/H.264 genuinely isn't decodable on
    // this hardware (see the VIDEO PLAYER section above) — point at the
    // practical alternative (.avi/MJPEG) instead of pretending it works.
    drawTextC("MP4 NOT SUPPORTED", 0, SCR_W, CONTENT_Y + 84, 1, COL_WARN);
    drawTextC("TRY MJPEG (.AVI)", 0, SCR_W, CONTENT_Y + 96, 1, COL_WARN);
  }

  drawFooter("BACK", COL_SECONDARY);
  drawScreenFrame();
}

void drawStorageTextScreen() {
  drawHeader(storageSelName);

  int16_t y = CONTENT_Y + 4;
  for (int i = 0; i < storageTextLineCount; i++) {
    drawTextL(storageTextLines[i], MARGIN, y, 1, COL_TEXT);
    y += 14;
  }

  drawFooter(storageTextAtEnd ? "END" : "MORE", storageTextAtEnd ? COL_SECONDARY : COL_PRIMARY);
  drawScreenFrame();
}

void drawStorageImageScreen() {
  drawHeader(storageSelName);

  // ===== MORPHIX STORAGE ===== storagePendingImageDraw == true means the
  // actual decode/draw (storageDrawJpeg()/storageDrawBmp(), called from
  // the deferred-draw block in loop()) hasn't run yet for this file —
  // storageImageLoaded/storageImageTooLarge/storageImageUnsupported are
  // still just their reset-to-false defaults from openSelectedFile() and
  // don't reflect a real outcome yet. Painting a status message off of
  // those defaults here (on the chrome-only render pass that happens
  // before the deferred draw) previously showed a premature "IMAGE
  // ERROR" that could be left as stray text once the real draw succeeded
  // and only painted over the fitted image rectangle. Skip status text
  // entirely while pending; the deferred-draw block repaints this screen
  // once the real result is known (see loop()).
  if (!storagePendingImageDraw) {
    if (storageImageTooLarge) {
      drawTextC("IMAGE TOO LARGE", 0, SCR_W, CONTENT_Y + 40, 1, COL_CRIT);
    } else if (storageImageUnsupported) {
      // ===== MORPHIX STORAGE ===== recognized-but-unhandled image variant
      // (compressed/palette BMP, progressive JPEG, etc) — distinct from a
      // corrupt file (IMAGE ERROR) and from one that's simply too big to
      // process safely (IMAGE TOO LARGE, now a rare case since ordinary
      // large photos are auto-fitted instead of rejected).
      drawTextC("UNSUPPORTED IMAGE", 0, SCR_W, CONTENT_Y + 40, 1, COL_WARN);
    } else if (!storageImageLoaded) {
      // ===== MORPHIX STORAGE ===== matches the spec's exact wording
      drawTextC("IMAGE ERROR", 0, SCR_W, CONTENT_Y + 34, 2, COL_CRIT);
      drawTextC("Unable to display", 0, SCR_W, CONTENT_Y + 56, 1, COL_CRIT);
    }
  }
  // Successful loads paint pixels directly onto the content area from
  // storageDrawBmp(), called once from openSelectedFile() below —
  // nothing further to draw here in that case.

  drawFooter("BACK", COL_SECONDARY);
  drawScreenFrame();
}

// ===== MORPHIX STORAGE =====
// Only draws chrome (header/footer/frame) + status text — the decoded
// video frame itself is painted directly to the content rect by
// storageVideoNextFrame()/storageOpenVideo() and must never be touched
// here, or it would be wiped on every same-screen redraw (state changes,
// PLAY/PAUSE footer updates, etc.) instead of just once per real frame.
void drawStorageVideoScreen() {
  drawHeader(storageSelName);

  if (storageVideoState == VIDEO_ERROR) {
    drawTextC("VIDEO ERROR", 0, SCR_W, CONTENT_Y + 34, 2, COL_CRIT);
    drawTextC("Unsupported/corrupt AVI", 0, SCR_W, CONTENT_Y + 56, 1, COL_CRIT);
  }

  const char* footerLabel = "BACK";
  uint16_t footerColor = COL_SECONDARY;
  switch (storageVideoState) {
    case VIDEO_PLAYING: footerLabel = "PAUSE";  footerColor = COL_PRIMARY; break;
    case VIDEO_PAUSED:  footerLabel = "PLAY";   footerColor = COL_PRIMARY; break;
    case VIDEO_ENDED:   footerLabel = "REPLAY"; footerColor = COL_PRIMARY; break;
    default: break;
  }
  drawFooter(footerLabel, footerColor);
  drawScreenFrame();
}

void drawStorageWirelessScreen() {
  drawHeader("WIRELESS");

  drawTextL("WI-FI", MARGIN, CONTENT_Y + 6, 1, COL_SECONDARY);
  drawTextR(storageServerActive ? "ON" : "OFF", SCR_W - MARGIN, CONTENT_Y + 6, 1,
            storageServerActive ? COL_GOOD : COL_SECONDARY);

  if (storageServerActive) {
    drawTextC(storageApSsid, 0, SCR_W, CONTENT_Y + 26, 1, COL_PRIMARY);
    char ipBuf[24];
    snprintf(ipBuf, sizeof(ipBuf), "http://%s", storageApIp.toString().c_str());
    drawTextC(ipBuf, 0, SCR_W, CONTENT_Y + 40, 1, COL_TEXT);
    drawTextC("CONNECT TO WI-FI ABOVE", 0, SCR_W, CONTENT_Y + 58, 1, COL_SECONDARY);
    drawTextC("THEN OPEN THAT ADDRESS", 0, SCR_W, CONTENT_Y + 70, 1, COL_SECONDARY);
    drawFooter("STOP", COL_CRIT);
  } else if (!storageAvailable()) {
    drawTextC("SD CARD REQUIRED", 0, SCR_W, CONTENT_Y + 40, 1, COL_CRIT);
    drawFooter("", COL_SECONDARY);
  } else {
    drawTextC("PRESS SELECT", 0, SCR_W, CONTENT_Y + 36, 1, COL_SECONDARY);
    drawTextC("TO START SERVER", 0, SCR_W, CONTENT_Y + 50, 1, COL_SECONDARY);
    drawFooter("START");
  }
  drawScreenFrame();
}

void drawStorageScreen() {
  switch (storageScreenState) {
    case STORAGE_SCR_HOME:      drawStorageHomeScreen();      break;
    case STORAGE_SCR_BROWSER:   drawStorageBrowserScreen();   break;
    case STORAGE_SCR_INFO:      drawStorageInfoScreen();      break;
    case STORAGE_SCR_TEXT:      drawStorageTextScreen();      break;
    case STORAGE_SCR_IMAGE:     drawStorageImageScreen();     break;
    case STORAGE_SCR_VIDEO:     drawStorageVideoScreen();     break; // ===== MORPHIX STORAGE =====
    case STORAGE_SCR_WIRELESS:  drawStorageWirelessScreen();  break; // ===== MORPHIX STORAGE =====
  }
}

// ------------------------------------------------------------
// OPEN / CLOSE
// ------------------------------------------------------------
// Called when leaving SCR_STORAGE for any other top-level screen —
// resets viewer state so re-entering Storage always starts clean at
// STORAGE_SCR_HOME, and nothing is left "open" while the user is
// elsewhere in MorPhix.
void closeStorageFiles() {
  storageScreenState = STORAGE_SCR_HOME;
  storageHomeIndex = 0;
  strcpy(storageCurrentPath, "/MORPHIX/FILES");
  storageEntryCount = 0;
  storageEntryIndex = 0;
  storageEntryScroll = 0;
  storageTextLineCount = 0;
  storageTextFileOffset = 0;
  storageTextPageStartOffset = 0; // ===== MORPHIX STORAGE =====
  storageTextPageStackDepth = 0;  // ===== MORPHIX STORAGE =====
  storageImageLoaded = false;
  storageImageTooLarge = false;
  // ===== MORPHIX STORAGE ===== if a video was open/playing, close it
  // properly (frees the frame buffer, closes the file, restores the SD/
  // TFT SPI arbitration) rather than leaving it dangling.
  if (storageVideoState != VIDEO_IDLE) storageCloseVideo();
  storagePendingVideoOpen = false;
  // Deliberately NOT stopping the wireless web server here — it's a
  // backgrounded service the user explicitly turned on (see the class
  // comment on storageServerActive above) and should keep serving files
  // even while the user is elsewhere in MorPhix, exactly like BLE.
}

// SELECT on a browser row: enters a folder, or opens a file according
// to its kind (text viewer / BMP viewer / metadata-only info screen).
void openSelectedFile() {
  if (storageEntryIndex < 0 || storageEntryIndex >= storageEntryCount) return;
  StorageEntry &e = storageEntries[storageEntryIndex];

  char childPath[STORAGE_PATH_LEN];
  snprintf(childPath, sizeof(childPath), "%s/%s", storageCurrentPath, e.name);

  if (e.isDir) {
    strncpy(storageCurrentPath, childPath, STORAGE_PATH_LEN - 1);
    storageCurrentPath[STORAGE_PATH_LEN - 1] = 0;
    storageRefreshListing();
    return;
  }

  strncpy(storageSelName, e.name, STORAGE_NAME_LEN - 1);
  storageSelName[STORAGE_NAME_LEN - 1] = 0;
  strncpy(storageSelPath, childPath, STORAGE_PATH_LEN - 1);
  storageSelPath[STORAGE_PATH_LEN - 1] = 0;
  storageSelSize = e.size;

  StorageFileKind kind = classifyFile(e.name, false);
  if (kind == SF_TEXT) {
    storageTextFileOffset = 0;
    storageTextPageStartOffset = 0;      // ===== MORPHIX STORAGE =====
    storageTextPageStackDepth = 0;       // ===== MORPHIX STORAGE =====
    storageLoadTextPage();
    storageScreenState = STORAGE_SCR_TEXT;
  } else if (kind == SF_IMAGE_BMP || kind == SF_IMAGE_JPEG) { // ===== MORPHIX STORAGE ===== JPEG now viewable too
    storageScreenState = STORAGE_SCR_IMAGE;
    storageImageLoaded = false;
    storageImageTooLarge = false;
    storageImageUnsupported = false;
    // Actual pixel draw happens once the screen transition below has
    // fillScreen()'d + drawn the frame/header, so it isn't immediately
    // overwritten — see the STORAGE_SCR_IMAGE branch in loop()'s
    // redraw block via storagePendingImageDraw.
    storagePendingImageDraw = true;
  } else if (kind == SF_VIDEO_AVI) { // ===== MORPHIX STORAGE =====
    storageScreenState = STORAGE_SCR_VIDEO;
    storageVideoState = VIDEO_IDLE;
    // Same deferred-open pattern as the image viewer above — the actual
    // SD open + header parse happens once the VIDEO screen's chrome has
    // been drawn, via storagePendingVideoOpen in loop()'s redraw block.
    storagePendingVideoOpen = true;
  } else {
    if (kind == SF_MP3) { // ===== MORPHIX STORAGE ===== cheap CBR duration estimate
      storageMp3HasDuration = storageMp3EstimateDuration(childPath, e.size, storageMp3DurationStr, sizeof(storageMp3DurationStr));
    } else {
      storageMp3HasDuration = false;
    }
    storageScreenState = STORAGE_SCR_INFO;
  }
}

// Consumes UP/DOWN/BACK/SELECT while SCR_STORAGE is current. Returns
// true if BACK should climb out of Storage entirely (back to HOME via
// the normal parentOf[] path in handleButtons()); false if BACK was
// fully handled internally (e.g. stepping up one folder level).
bool handleStorageInput(char btn) {
  switch (storageScreenState) {
    case STORAGE_SCR_HOME:
      // ===== MORPHIX STORAGE ===== modulo/count bumped 6->STORAGE_HOME_COUNT (7) for the new WIRELESS TRANSFER row
      if (btn == 'U') { storageHomeIndex = (storageHomeIndex - 1 + STORAGE_HOME_COUNT) % STORAGE_HOME_COUNT; needsRedraw = true; }
      else if (btn == 'D') { storageHomeIndex = (storageHomeIndex + 1) % STORAGE_HOME_COUNT; needsRedraw = true; }
      else if (btn == 'S' && storageHomeIndex == STORAGE_HOME_WIRELESS_INDEX) {
        // ===== MORPHIX STORAGE ===== WIRELESS TRANSFER has no folder to browse — opens its own control screen instead
        storageScreenState = STORAGE_SCR_WIRELESS;
        needsRedraw = true;
      } else if (btn == 'S' && storageAvailable()) {
        strncpy(storageCurrentPath, STORAGE_HOME_PATHS[storageHomeIndex], STORAGE_PATH_LEN - 1);
        storageCurrentPath[STORAGE_PATH_LEN - 1] = 0;
        storageRefreshListing();
        storageScreenState = STORAGE_SCR_BROWSER;
        needsRedraw = true;
      } else if (btn == 'S' && !storageAvailable()) {
        initStorage(); // manual retry — rate-limited internally, safe to spam
        needsRedraw = true;
      } else if (btn == 'B') {
        return true; // climb out to HOME
      }
      return false;

    case STORAGE_SCR_BROWSER:
      if (btn == 'U' && storageEntryCount > 0) {
        storageEntryIndex = (storageEntryIndex - 1 + storageEntryCount) % storageEntryCount;
        clampListScroll(storageEntryScroll, storageEntryIndex, storageEntryCount);
        needsRedraw = true;
      } else if (btn == 'D' && storageEntryCount > 0) {
        storageEntryIndex = (storageEntryIndex + 1) % storageEntryCount;
        clampListScroll(storageEntryScroll, storageEntryIndex, storageEntryCount);
        needsRedraw = true;
      } else if (btn == 'S' && storageEntryCount > 0) {
        openSelectedFile();
        needsRedraw = true;
      } else if (btn == 'B') {
        // Step up one path segment; if that lands above the six
        // category roots, go back to STORAGE_SCR_HOME instead.
        char* slash = strrchr(storageCurrentPath, '/');
        if (slash && slash != storageCurrentPath) {
          *slash = 0;
          // Any path at or above /MORPHIX/FILES (or /MORPHIX/RECEIVED)
          // that is no longer a category root just re-lists as a
          // browser view of the parent folder, UNLESS it's shorter
          // than the shortest category root, in which case bounce home.
          if (strlen(storageCurrentPath) < strlen("/MORPHIX/FILES")) {
            storageScreenState = STORAGE_SCR_HOME;
          } else {
            storageRefreshListing();
          }
        } else {
          storageScreenState = STORAGE_SCR_HOME;
        }
        needsRedraw = true;
      }
      return false;

    case STORAGE_SCR_INFO:
      if (btn == 'B') { storageScreenState = STORAGE_SCR_BROWSER; needsRedraw = true; }
      return false;

    case STORAGE_SCR_TEXT:
      if (btn == 'D' || btn == 'S') {
        storageTextGoForward(); needsRedraw = true; // ===== MORPHIX STORAGE =====
      } else if (btn == 'U') {
        storageTextGoBack(); needsRedraw = true; // ===== MORPHIX STORAGE =====
      } else if (btn == 'B') {
        storageScreenState = STORAGE_SCR_BROWSER;
        needsRedraw = true;
      }
      return false;

    case STORAGE_SCR_IMAGE:
      if (btn == 'B') { storageScreenState = STORAGE_SCR_BROWSER; needsRedraw = true; }
      return false;

    // ===== MORPHIX STORAGE =====
    case STORAGE_SCR_VIDEO:
      if (btn == 'S') {
        if (storageVideoState == VIDEO_PLAYING) {
          storageVideoState = VIDEO_PAUSED;
          needsRedraw = true;
        } else if (storageVideoState == VIDEO_PAUSED) {
          storageVideoState = VIDEO_PLAYING;
          storageVideoLastFrameMs = millis();
          needsRedraw = true;
        } else if (storageVideoState == VIDEO_ENDED) {
          storageVideoPos = storageAviInfo.moviStart; // replay from the start
          tft.fillRect(0, CONTENT_Y, SCR_W, CONTENT_H, COL_BG);
          storageVideoState = VIDEO_PLAYING;
          storageVideoLastFrameMs = millis();
          needsRedraw = true;
        }
      } else if (btn == 'B') {
        storageCloseVideo(); // closes file, frees frame buffer, ends the SD session
        storageScreenState = STORAGE_SCR_BROWSER;
        needsRedraw = true;
      }
      return false;

    // ===== MORPHIX STORAGE =====
    case STORAGE_SCR_WIRELESS:
      if (btn == 'S') {
        if (storageServerActive) stopStorageServer(); else startStorageServer();
        needsRedraw = true;
      } else if (btn == 'B') {
        storageScreenState = STORAGE_SCR_HOME;
        needsRedraw = true;
      }
      return false;
  }
  return false;
}

// ============================================================
// 12. SCREEN RENDERING
// ============================================================

// ---------- HOME ----------
// Time lives in a fixed rectangle so it can be redrawn on its own every
// second without flickering the rest of the screen (see updateHomeClock()).
#define HOME_TIME_Y 34
#define HOME_TIME_H 24

void drawHomeScreen() {
  drawHeader("MORPHIX");

  char buf[16];

  // Row: day/date (left) + status dot + battery (right)
  if (clockHasValidTime) {
    snprintf(buf, sizeof(buf), "%s %s", mockDay, mockDate);
  } else {
    snprintf(buf, sizeof(buf), "SYNC");
  }
  drawTextL(buf, MARGIN, 20, 1, COL_SECONDARY);

  // Real connectivity — lit if ANY real link is up (WiFi, a paired
  // generic BLE device, or the phone link), never a hard-coded true.
  drawStatusDot(120, 24, connData.wifiConnected || connData.bleConnected || connData.phoneConnected);

  snprintf(buf, sizeof(buf), "%d%%", mockBatteryPct);
  uint16_t battTextW = textWidthOf(buf, 1);
  int16_t battTextX = SCR_W - MARGIN - battTextW;
  drawTextL(buf, battTextX, 20, 1, COL_TEXT);
  drawBatteryIcon(battTextX - 16, 20, mockBatteryPct);

  // Big time, size 3 (18x24 px per char)
  snprintf(buf, sizeof(buf), "%02d:%02d", mockHour, mockMinute);
  drawTextBoldC(buf, 0, SCR_W, HOME_TIME_Y, 3, COL_TEXT);

  drawDivider(62);

  // Mini stats: TEMP (left) / HR (right)
  drawTextL("TEMP", MARGIN, 66, 1, COL_SECONDARY);
  drawTextR("HR", SCR_W / 2 + 30, 66, 1, COL_SECONDARY);

  char tempBuf[10];
  if (bmp280Ready) {
    dtostrf(realTemperatureC, 0, 1, tempBuf);
    strcat(tempBuf, "C");
  } else {
    snprintf(tempBuf, sizeof(tempBuf), "--C");
  }
  drawTextL(tempBuf, MARGIN, 76, 1, bmp280Ready ? COL_PRIMARY : COL_SECONDARY);

  int homeHR = getStableHeartRate();
  if (homeHR > 0) {
    snprintf(buf, sizeof(buf), "%d BPM", homeHR);
  } else {
    snprintf(buf, sizeof(buf), "-- BPM");
  }
  drawTextR(buf, SCR_W / 2 + 30, 76, 1, homeHR > 0 ? COL_PRIMARY : COL_SECONDARY);

  // Activity summary line
  // Uses its own larger buffer — "buf" above is sized 16 for the short
  // time/battery strings, but "NNNNN STEPS - 100%" needs up to 19 chars
  // plus the null terminator. With buf[16], snprintf was silently
  // truncating this line once step count grew past a few digits (or once
  // the percentage hit 100%), which is what was showing a cut-off/wrong
  // goal percentage on the home screen.
  {
    char actBuf[24];
    long homeSteps = getStepCount();
    int homePct = (int)(100L * homeSteps / dailyStepGoal);
    if (homePct > 100) homePct = 100;
    snprintf(actBuf, sizeof(actBuf), "%ld STEPS - %d%%", homeSteps, homePct);
    drawTextC(actBuf, 0, SCR_W, 96, 1, COL_SECONDARY);
  }

  drawFooter("MENU");
  drawScreenFrame();
}

// Redraws ONLY the clock digits — used on the once-per-second tick so the
// rest of the Home screen (header, footer, stats) never flickers.
void updateHomeClock() {
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d:%02d", mockHour, mockMinute);
  tft.fillRect(0, HOME_TIME_Y, SCR_W, HOME_TIME_H, COL_BG);
  drawTextBoldC(buf, 0, SCR_W, HOME_TIME_Y, 3, COL_TEXT);
}

// Redraws ONLY the TEMP mini-stat on Home (left of the divider at y=66),
// and only when the displayed value actually changed. Region matches
// exactly where drawHomeScreen() paints it, so this is a no-op visually
// unless the text itself is different.
void updateHomeTemp() {
  static char lastShown[10] = "";
  char tempBuf[10];
  if (bmp280Ready) {
    dtostrf(realTemperatureC, 0, 1, tempBuf);
    strcat(tempBuf, "C");
  } else {
    snprintf(tempBuf, sizeof(tempBuf), "--C");
  }
  if (!displayValueChanged(lastShown, sizeof(lastShown), tempBuf)) return;
  tft.fillRect(MARGIN, 76, 50, 9, COL_BG);
  drawTextL(tempBuf, MARGIN, 76, 1, bmp280Ready ? COL_PRIMARY : COL_SECONDARY);
}

// Redraws ONLY the HR mini-stat on Home (right of the divider at y=66),
// same change-detection approach as updateHomeTemp().
void updateHomeHR() {
  static char lastShown[10] = "";
  char buf[10];
  int homeHR = getStableHeartRate();
  if (homeHR > 0) {
    snprintf(buf, sizeof(buf), "%d BPM", homeHR);
  } else {
    snprintf(buf, sizeof(buf), "-- BPM");
  }
  if (!displayValueChanged(lastShown, sizeof(lastShown), buf)) return;
  tft.fillRect(SCR_W / 2 - 10, 76, SCR_W / 2 + 40 - (SCR_W / 2 - 10), 9, COL_BG);
  drawTextR(buf, SCR_W / 2 + 30, 76, 1, homeHR > 0 ? COL_PRIMARY : COL_SECONDARY);
}

// ---------- ENVIRONMENT ----------
void drawEnvironmentScreen() {
  drawHeader("ENVIRONMENT");

  const int rowH = 30;
  int y = CONTENT_Y;
  char buf[16];

  drawTextL("TEMP", MARGIN, y + 4, 1, COL_SECONDARY);
  if (bmp280Ready) {
    dtostrf(realTemperatureC, 0, 1, buf);
    strcat(buf, " C");
  } else {
    snprintf(buf, sizeof(buf), "--");
  }
  drawTextR(buf, SCR_W - MARGIN, y + 8, 2, bmp280Ready ? COL_PRIMARY : COL_SECONDARY);
  drawDivider(y + rowH - 1);
  y += rowH;

  drawTextL("STATUS", MARGIN, y + 4, 1, COL_SECONDARY);
  drawTextR(bmp280Ready ? "LIVE" : "OFFLINE", SCR_W - MARGIN, y + 8, 2, bmp280Ready ? COL_GOOD : COL_CRIT);
  drawDivider(y + rowH - 1);
  y += rowH;

  drawTextL("PRESSURE", MARGIN, y + 3, 1, COL_SECONDARY);
  if (bmp280Ready) {
    snprintf(buf, sizeof(buf), "%.0f hPa", realPressureHpa);
  } else {
    snprintf(buf, sizeof(buf), "--");
  }
  drawTextR(buf, SCR_W - MARGIN, y + 7, 2, bmp280Ready ? COL_PRIMARY : COL_SECONDARY);

  drawFooter("SYNC");
  drawScreenFrame();
}

// Redraws only the three value fields on ENVIRONMENT (TEMP / STATUS /
// PRESSURE), each only when its own displayed value changed. Labels,
// dividers, header and footer are untouched — they were already painted
// once by drawEnvironmentScreen() on screen entry.
void updateEnvironmentDynamic() {
  const int rowH = 30;
  int y = CONTENT_Y;
  char buf[16];

  static char lastTemp[16] = "";
  if (bmp280Ready) { dtostrf(realTemperatureC, 0, 1, buf); strcat(buf, " C"); }
  else              snprintf(buf, sizeof(buf), "--");
  if (displayValueChanged(lastTemp, sizeof(lastTemp), buf)) {
    tft.fillRect(70, y + 8 - 12, SCR_W - MARGIN - 70, 16, COL_BG);
    drawTextR(buf, SCR_W - MARGIN, y + 8, 2, bmp280Ready ? COL_PRIMARY : COL_SECONDARY);
  }
  y += rowH;

  static int lastReady = -1;
  if (lastReady != (int)bmp280Ready) {
    lastReady = (int)bmp280Ready;
    tft.fillRect(70, y + 8 - 12, SCR_W - MARGIN - 70, 16, COL_BG);
    drawTextR(bmp280Ready ? "LIVE" : "OFFLINE", SCR_W - MARGIN, y + 8, 2, bmp280Ready ? COL_GOOD : COL_CRIT);
  }
  y += rowH;

  static char lastPressure[16] = "";
  if (bmp280Ready) snprintf(buf, sizeof(buf), "%.0f hPa", realPressureHpa);
  else              snprintf(buf, sizeof(buf), "--");
  if (displayValueChanged(lastPressure, sizeof(lastPressure), buf)) {
    tft.fillRect(60, y + 7 - 12, SCR_W - MARGIN - 60, 16, COL_BG);
    drawTextR(buf, SCR_W - MARGIN, y + 7, 2, bmp280Ready ? COL_PRIMARY : COL_SECONDARY);
  }
}

// ---------- HEALTH ----------
void drawHealthScreen() {
  drawHeader("HEALTH");

  int hr   = getStableHeartRate();
  int spo2 = getStableSpO2();

  char buf[12];
  if (hr > 0) {
    snprintf(buf, sizeof(buf), "%d", hr);
  } else {
    snprintf(buf, sizeof(buf), "--");
  }
  drawTextBoldC(buf, 0, SCR_W, 24, 3, hr > 0 ? COL_TEXT : COL_SECONDARY);
  drawTextC("BPM", 0, SCR_W, 50, 1, COL_SECONDARY);

  drawDivider(62);

  drawTextL("SpO2", MARGIN, 68, 1, COL_SECONDARY);
  if (spo2 > 0) {
    snprintf(buf, sizeof(buf), "%d %%", spo2);
  } else {
    snprintf(buf, sizeof(buf), "--");
  }
  drawTextR(buf, SCR_W - MARGIN, 66, 2, spo2 > 0 ? COL_PRIMARY : COL_SECONDARY);

  // Status reflects real sensor/signal/motion state — never a fabricated
  // medical read (see hrQuality, gated by contact + optical signal +
  // MPU6050 motion in updateMAX30102()/processMAX30102Window()).
  const char* status;
  uint16_t statusColor;
  if (!max30102Ready) {
    status = "OFFLINE";
    statusColor = COL_CRIT;
  } else if (!max30102Contact) {
    status = "PLACE ON SKIN";
    statusColor = COL_WARN;
  } else if (hr < 0 || spo2 < 0) {
    status = "MEASURING";
    statusColor = COL_PRIMARY;
  } else {
    switch (hrQuality) {
      case HRQ_MOTION:      status = "MOTION";      statusColor = COL_WARN; break;
      case HRQ_POOR_SIGNAL: status = "POOR SIGNAL";  statusColor = COL_WARN; break;
      case HRQ_GOOD:        status = "GOOD";         statusColor = COL_GOOD; break;
      default:               status = "MEASURING";    statusColor = COL_PRIMARY; break;
    }
  }
  drawBadge(status, SCR_W / 2 - 40, SCR_W / 2 + 40, 92, 14, statusColor);

  drawFooter("SYNC");
  drawScreenFrame();
}

// Redraws only HEALTH's dynamic fields (big HR number, SpO2 value, and
// the status badge), each gated on its own displayed value changing.
// Header, "BPM"/"SpO2" labels, divider and footer stay untouched.
void updateHealthDynamic() {
  int hr   = getStableHeartRate();
  int spo2 = getStableSpO2();
  char buf[12];

  static char lastHR[12] = "";
  if (hr > 0) snprintf(buf, sizeof(buf), "%d", hr);
  else         snprintf(buf, sizeof(buf), "--");
  if (displayValueChanged(lastHR, sizeof(lastHR), buf)) {
    tft.fillRect(0, 24 - 4, SCR_W, 28, COL_BG);
    drawTextBoldC(buf, 0, SCR_W, 24, 3, hr > 0 ? COL_TEXT : COL_SECONDARY);
  }

  static char lastSpO2[12] = "";
  if (spo2 > 0) snprintf(buf, sizeof(buf), "%d %%", spo2);
  else           snprintf(buf, sizeof(buf), "--");
  if (displayValueChanged(lastSpO2, sizeof(lastSpO2), buf)) {
    tft.fillRect(SCR_W - MARGIN - 50, 66 - 12, 50, 18, COL_BG);
    drawTextR(buf, SCR_W - MARGIN, 66, 2, spo2 > 0 ? COL_PRIMARY : COL_SECONDARY);
  }

  const char* status; uint16_t statusColor;
  if (!max30102Ready)          { status = "OFFLINE";      statusColor = COL_CRIT; }
  else if (!max30102Contact)   { status = "PLACE ON SKIN"; statusColor = COL_WARN; }
  else if (hr < 0 || spo2 < 0) { status = "MEASURING";     statusColor = COL_PRIMARY; }
  else {
    switch (hrQuality) {
      case HRQ_MOTION:      status = "MOTION";     statusColor = COL_WARN; break;
      case HRQ_POOR_SIGNAL: status = "POOR SIGNAL"; statusColor = COL_WARN; break;
      case HRQ_GOOD:        status = "GOOD";        statusColor = COL_GOOD; break;
      default:               status = "MEASURING";   statusColor = COL_PRIMARY; break;
    }
  }
  static char lastStatus[16] = "";
  if (displayValueChanged(lastStatus, sizeof(lastStatus), status)) {
    tft.fillRect(SCR_W / 2 - 40, 92, 80, 14, COL_BG);
    drawBadge(status, SCR_W / 2 - 40, SCR_W / 2 + 40, 92, 14, statusColor);
  }
}

// ---------- ACTIVITY ----------
void drawActivityScreen() {
  drawHeader("ACTIVITY");

  char buf[16];
  long steps = getStepCount();
  snprintf(buf, sizeof(buf), "%ld", steps);
  drawTextBoldC(buf, 0, SCR_W, 20, 2, COL_TEXT);
  drawTextC("STEPS", 0, SCR_W, 38, 1, COL_SECONDARY);

  snprintf(buf, sizeof(buf), "GOAL %ld", dailyStepGoal);
  drawTextC(buf, 0, SCR_W, 50, 1, COL_SECONDARY);

  int pct = (int)(100L * steps / dailyStepGoal);
  if (pct > 100) pct = 100;
  drawProgressBar(MARGIN, 62, SCR_W - 2 * MARGIN, 10, pct, COL_PRIMARY);

  snprintf(buf, sizeof(buf), "%d%%", pct);
  drawTextC(buf, 0, SCR_W, 76, 1, COL_PRIMARY);

  const char* motionText = !mpu6050Ready ? "N/A" : (motionIsMoving ? "MOVING" : "STABLE");
  uint16_t motionColor = !mpu6050Ready ? COL_SECONDARY : (motionIsMoving ? COL_WARN : COL_GOOD);
  drawTextL("MOTION", MARGIN, 90, 1, COL_SECONDARY);
  drawTextR(motionText, SCR_W - MARGIN, 90, 1, motionColor);

  drawFooter("SYNC");
  drawScreenFrame();
}

// Redraws only ACTIVITY's dynamic fields (step count, progress bar +
// percent, motion state), each gated on its own displayed value
// changing. Header, "STEPS"/"GOAL"/"MOTION" labels and footer stay
// untouched — they were already painted once by drawActivityScreen()
// on screen entry.
void updateActivityDynamic() {
  char buf[16];

  static long lastSteps = -1;
  long steps = getStepCount();
  if (steps != lastSteps) {
    lastSteps = steps;

    snprintf(buf, sizeof(buf), "%ld", steps);
    tft.fillRect(0, 20, SCR_W, 18, COL_BG);
    drawTextBoldC(buf, 0, SCR_W, 20, 2, COL_TEXT);

    int pct = (int)(100L * steps / dailyStepGoal);
    if (pct > 100) pct = 100;
    drawProgressBar(MARGIN, 62, SCR_W - 2 * MARGIN, 10, pct, COL_PRIMARY);

    snprintf(buf, sizeof(buf), "%d%%", pct);
    tft.fillRect(0, 76, SCR_W, 10, COL_BG);
    drawTextC(buf, 0, SCR_W, 76, 1, COL_PRIMARY);
  }

  static int lastMotionState = -1;
  int motionState = !mpu6050Ready ? 0 : (motionIsMoving ? 1 : 2);
  if (motionState != lastMotionState) {
    lastMotionState = motionState;
    const char* motionText = !mpu6050Ready ? "N/A" : (motionIsMoving ? "MOVING" : "STABLE");
    uint16_t motionColor = !mpu6050Ready ? COL_SECONDARY : (motionIsMoving ? COL_WARN : COL_GOOD);
    // Right-aligned value only — "MOTION" label on the left is static
    // and untouched.
    tft.fillRect(70, 90, SCR_W - MARGIN - 70, 10, COL_BG);
    drawTextR(motionText, SCR_W - MARGIN, 90, 1, motionColor);
  }
}

// ---------- NOTIFICATIONS ----------
// Copies up to (outLen-1) chars of src into out, appending ".." if it had
// to cut src short — used so a long title/text from the phone never runs
// off the edge of the 160px panel instead of silently overflowing.
void truncateForDisplay(const char* src, char* out, size_t outLen) {
  size_t srcLen = strlen(src);
  if (srcLen < outLen) {
    strcpy(out, src);
    return;
  }
  size_t keep = (outLen > 3) ? outLen - 3 : 0;
  strncpy(out, src, keep);
  out[keep] = 0;
  strcat(out, "..");
}

void drawNotificationsScreen() {
  char title[20];
  snprintf(title, sizeof(title), "NOTIFS (%d)", notifCount);
  drawHeader(title);

  if (notifCount == 0) {
    // Real empty state now — no phone linked yet, or the phone hasn't
    // forwarded anything. Never a mock/demo placeholder value.
    drawTextC(connData.phoneConnected ? "NO NOTIFICATIONS" : "PHONE NOT LINKED",
               0, SCR_W, 56, 1, COL_SECONDARY);
  } else {
    if (notifIndex >= notifCount) notifIndex = notifCount - 1;
    QueuedNotification &n = notifQueue[notifIndex];

    // Unread dot to the left of the app name; read notifications just get
    // a blank space there so the app-name column stays aligned either way.
    if (n.unread) drawStatusDot(MARGIN, 26, true);
    char appBuf[NOTIF_APP_LEN];
    truncateForDisplay(n.app, appBuf, sizeof(appBuf));
    drawTextL(appBuf, MARGIN + 8, 26, 1, COL_PRIMARY);

    // Second line combines title + body (matches the two-line layout this
    // screen has always used) — "Title: body text", or just the body if
    // the phone didn't supply a title for this notification.
    char combined[NOTIF_TITLE_LEN + NOTIF_TEXT_LEN + 4];
    if (n.title[0] != 0) {
      snprintf(combined, sizeof(combined), "%s: %s", n.title, n.text);
    } else {
      strncpy(combined, n.text, sizeof(combined) - 1);
      combined[sizeof(combined) - 1] = 0;
    }
    char previewBuf[26];
    truncateForDisplay(combined, previewBuf, sizeof(previewBuf));
    drawTextL(previewBuf, MARGIN, 42, 1, COL_TEXT);

    // Real timestamp when we have a phone time sync; otherwise an honest
    // "--:--" rather than a fabricated time.
    char timeBuf[9];
    if (n.timestampSec > 0) {
      time_t t = (time_t)n.timestampSec;
      struct tm tmVal;
      gmtime_r(&t, &tmVal);
      snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", tmVal.tm_hour, tmVal.tm_min);
    } else {
      strcpy(timeBuf, "--:--");
    }
    drawTextL(timeBuf, MARGIN, 60, 1, COL_SECONDARY);

    char pager[8];
    snprintf(pager, sizeof(pager), "%d/%d", notifIndex + 1, notifCount);
    drawTextR(pager, SCR_W - MARGIN, 96, 1, COL_SECONDARY);
  }

  drawFooter(notifCount > 0 ? "DISMISS" : "BACK", COL_WARN);
  drawScreenFrame();
}

// ---------- CONNECTIVITY (hub) ----------
const char* connMenuLabels[3] = {"WIFI", "BLUETOOTH", "PHONE LINK"};
int connIndex = 0;

void drawConnectivityScreen() {
  drawHeader("CONNECTIVITY");

  const int rowH = 28;
  for (int i = 0; i < 3; i++) {
    int16_t y = CONTENT_Y + i * rowH;
    bool selected = (i == connIndex);

    const char* statusText; uint16_t statusColor;
    if (i == 0) {
      statusText = connData.wifiConnected ? "CONNECTED" : "OFFLINE";
      statusColor = connData.wifiConnected ? COL_GOOD : COL_CRIT;
    } else if (i == 1) {
      statusText = connData.bleConnected ? "CONNECTED" : (connData.bleReady ? "READY" : "OFF");
      statusColor = connData.bleConnected ? COL_GOOD : (connData.bleReady ? COL_PRIMARY : COL_CRIT);
    } else {
      statusText = connData.phoneConnected ? "CONNECTED" : "DISCONNECTED";
      statusColor = connData.phoneConnected ? COL_GOOD : COL_CRIT;
    }
    drawMenuRow(y, rowH, connMenuLabels[i], selected, statusText, statusColor);
  }

  drawFooter("SELECT");
  drawScreenFrame();
}

// ---------- WIFI ----------
// A small shared row-list renderer used by both the WiFi and BLE result
// screens: draws a "i/count" position label, up to LIST_VISIBLE_ROWS
// rows from a scrolled window, a signal indicator per row, and an empty
// state if there's nothing to show. label1/label2/rssi come from a
// per-row callback-free layout since WifiNetwork and BLEDevice aren't a
// shared type — see the two thin wrappers below.
void drawDiscoveryList(const char* emptyText, int count, int selIdx, int scrollIdx,
                        int rowH, const char* names[], int rssis[]) {
  if (count == 0) {
    drawTextC(emptyText, 0, SCR_W, CONTENT_Y + 40, 1, COL_CRIT);
    drawFooter("", COL_SECONDARY);
    return;
  }

  char posBuf[8];
  snprintf(posBuf, sizeof(posBuf), "%d/%d", selIdx + 1, count);
  drawTextR(posBuf, SCR_W - MARGIN, CONTENT_Y + 2, 1, COL_SECONDARY);

  int visible = min(count - scrollIdx, LIST_VISIBLE_ROWS);
  for (int row = 0; row < visible; row++) {
    int i = scrollIdx + row;
    int16_t y = CONTENT_Y + 14 + row * rowH;
    bool sel = (i == selIdx);

    drawTextL(sel ? ">" : " ", MARGIN, y, 1, COL_PRIMARY);
    drawTextL(names[i], MARGIN + 10, y, 1, sel ? COL_PRIMARY : COL_TEXT);

    char rssiBuf[8];
    snprintf(rssiBuf, sizeof(rssiBuf), "%d", rssis[i]);
    drawSignalBars(SCR_W - MARGIN - 11, y, rssis[i]);
    drawTextR(rssiBuf, SCR_W - MARGIN - 14, y, 1, COL_SECONDARY);
  }
  drawFooter("OPEN");
}

void drawWifiScreen() {
  drawHeader("WIFI");

  switch (wifiScreenState) {
    case WIFI_SCR_STATUS: {
      drawTextL("STATUS", MARGIN, CONTENT_Y + 6, 1, COL_SECONDARY);
      if (connData.wifiConnected) {
        drawTextR("CONNECTED", SCR_W - MARGIN, CONTENT_Y + 6, 1, COL_GOOD);

        drawTextL("NETWORK", MARGIN, CONTENT_Y + 26, 1, COL_SECONDARY);
        drawTextR(connData.wifiSSID, SCR_W - MARGIN, CONTENT_Y + 26, 1, COL_PRIMARY);

        char rssiBuf[16];
        snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", connData.wifiRSSI);
        drawTextL("RSSI", MARGIN, CONTENT_Y + 46, 1, COL_SECONDARY);
        drawTextR(rssiBuf, SCR_W - MARGIN, CONTENT_Y + 46, 1, COL_TEXT);

        drawFooter("DISCONNECT", COL_WARN);
      } else {
        drawTextR("OFFLINE", SCR_W - MARGIN, CONTENT_Y + 6, 1, COL_CRIT);
        drawTextC("PRESS SELECT TO SCAN", 0, SCR_W, CONTENT_Y + 40, 1, COL_SECONDARY);
        drawFooter("SCAN");
      }
      break;
    }
    case WIFI_SCR_SCANNING:
      drawTextC("SCANNING...", 0, SCR_W, CONTENT_Y + 36, 1, COL_PRIMARY);
      drawFooter("", COL_SECONDARY);
      break;
    case WIFI_SCR_LIST: {
      const char* names[MAX_WIFI_NETWORKS];
      int rssis[MAX_WIFI_NETWORKS];
      for (int i = 0; i < wifiNetworkCount; i++) { names[i] = wifiNetworks[i].ssid; rssis[i] = wifiNetworks[i].rssi; }
      drawDiscoveryList("NO NETWORKS FOUND", wifiNetworkCount, wifiListIndex, wifiListScroll, 16, names, rssis);
      break;
    }
    case WIFI_SCR_DETAILS: {
      WifiNetwork &n = wifiNetworks[selectedWifiNetwork];
      drawTextC("NETWORK DETAILS", 0, SCR_W, CONTENT_Y + 4, 1, COL_SECONDARY);

      drawTextL("SSID", MARGIN, CONTENT_Y + 20, 1, COL_SECONDARY);
      drawTextR(n.ssid, SCR_W - MARGIN, CONTENT_Y + 20, 1, COL_PRIMARY);

      char rssiBuf[16];
      snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", n.rssi);
      drawTextL("SIGNAL", MARGIN, CONTENT_Y + 36, 1, COL_SECONDARY);
      drawSignalBars(SCR_W - MARGIN - 11, CONTENT_Y + 34, n.rssi);
      drawTextR(rssiBuf, SCR_W - MARGIN - 15, CONTENT_Y + 36, 1, COL_TEXT);

      drawTextL("SECURITY", MARGIN, CONTENT_Y + 52, 1, COL_SECONDARY);
      drawTextR(n.security, SCR_W - MARGIN, CONTENT_Y + 52, 1, COL_TEXT);

      drawTextL("STATUS", MARGIN, CONTENT_Y + 68, 1, COL_SECONDARY);
      drawTextR("AVAILABLE", SCR_W - MARGIN, CONTENT_Y + 68, 1, COL_GOOD);

      drawFooter("CONNECT");
      break;
    }
    case WIFI_SCR_NOCRED: {
      // TEST_WIFI_SSID left blank entirely -> nothing has been configured
      // at all; otherwise this network just doesn't match what IS
      // configured, i.e. we'd need its password.
      bool blank = (strlen(TEST_WIFI_SSID) == 0);
      drawTextC(blank ? "NOT CONFIGURED" : "PASSWORD REQUIRED", 0, SCR_W, CONTENT_Y + 28, 1, COL_WARN);
      drawTextC(wifiNetworks[selectedWifiNetwork].ssid, 0, SCR_W, CONTENT_Y + 46, 1, COL_TEXT);
      drawFooter("BACK", COL_SECONDARY);
      break;
    }
    case WIFI_SCR_CONNECTING:
      drawTextC("CONNECTING...", 0, SCR_W, CONTENT_Y + 28, 1, COL_PRIMARY);
      drawTextC(wifiNetworks[selectedWifiNetwork].ssid, 0, SCR_W, CONTENT_Y + 46, 1, COL_TEXT);
      drawFooter("", COL_SECONDARY);
      break;
    case WIFI_SCR_CONNECTED:
      drawTextC("CONNECTED", 0, SCR_W, CONTENT_Y + 28, 1, COL_GOOD);
      drawTextC(wifiNetworks[selectedWifiNetwork].ssid, 0, SCR_W, CONTENT_Y + 46, 1, COL_TEXT);
      drawFooter("", COL_SECONDARY);
      break;
    case WIFI_SCR_FAILED:
      drawTextC("CONNECTION FAILED", 0, SCR_W, CONTENT_Y + 28, 1, COL_CRIT);
      drawTextC(wifiNetworks[selectedWifiNetwork].ssid, 0, SCR_W, CONTENT_Y + 46, 1, COL_TEXT);
      drawFooter("RETRY", COL_WARN);
      break;
  }
  drawScreenFrame();
}

// ---------- BLUETOOTH ----------
void drawBluetoothScreen() {
  drawHeader("BLUETOOTH");

  switch (bleScreenState) {
    case BLE_SCR_STATUS: {
      drawTextL("STATUS", MARGIN, CONTENT_Y + 6, 1, COL_SECONDARY);
      if (connData.bleConnected) {
        drawTextR("CONNECTED", SCR_W - MARGIN, CONTENT_Y + 6, 1, COL_GOOD);

        drawTextL("DEVICE", MARGIN, CONTENT_Y + 26, 1, COL_SECONDARY);
        drawTextR(connData.bleDeviceName, SCR_W - MARGIN, CONTENT_Y + 26, 1, COL_PRIMARY);

        char rssiBuf[16];
        snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", connData.bleRSSI);
        drawTextL("RSSI", MARGIN, CONTENT_Y + 46, 1, COL_SECONDARY);
        drawTextR(rssiBuf, SCR_W - MARGIN, CONTENT_Y + 46, 1, COL_TEXT);

        drawFooter("DISCONNECT", COL_WARN);
      } else {
        drawTextR(connData.bleReady ? "READY" : "OFF", SCR_W - MARGIN, CONTENT_Y + 6, 1,
                  connData.bleReady ? COL_PRIMARY : COL_CRIT);
        drawTextC("PRESS SELECT TO SCAN", 0, SCR_W, CONTENT_Y + 40, 1, COL_SECONDARY);
        drawFooter("SCAN");
      }
      break;
    }
    case BLE_SCR_SCANNING:
      drawTextC("SCANNING...", 0, SCR_W, CONTENT_Y + 36, 1, COL_PRIMARY);
      drawFooter("", COL_SECONDARY);
      break;
    case BLE_SCR_LIST: {
      const char* names[MAX_BLE_DEVICES];
      int rssis[MAX_BLE_DEVICES];
      for (int i = 0; i < bleDeviceCount; i++) { names[i] = bleDevices[i].name; rssis[i] = bleDevices[i].rssi; }
      drawDiscoveryList("NO DEVICES FOUND", bleDeviceCount, bleListIndex, bleListScroll, 18, names, rssis);
      break;
    }
    case BLE_SCR_DETAILS: {
      BleDeviceInfo &d = bleDevices[selectedBLEDevice];
      drawTextC("DEVICE", 0, SCR_W, CONTENT_Y + 4, 1, COL_SECONDARY);
      drawTextC(d.name, 0, SCR_W, CONTENT_Y + 18, 1, COL_PRIMARY);

      // Never claims a specific device class (phone/laptop/etc.) — a BLE
      // advertisement alone isn't reliable evidence of what a device is.
      drawTextL("TYPE", MARGIN, CONTENT_Y + 38, 1, COL_SECONDARY);
      drawTextR("BLE DEVICE", SCR_W - MARGIN, CONTENT_Y + 38, 1, COL_TEXT);

      char rssiBuf[16];
      snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", d.rssi);
      drawTextL("SIGNAL", MARGIN, CONTENT_Y + 54, 1, COL_SECONDARY);
      drawSignalBars(SCR_W - MARGIN - 11, CONTENT_Y + 52, d.rssi);
      drawTextR(rssiBuf, SCR_W - MARGIN - 15, CONTENT_Y + 54, 1, COL_TEXT);

      drawTextL("STATUS", MARGIN, CONTENT_Y + 70, 1, COL_SECONDARY);
      drawTextR("AVAILABLE", SCR_W - MARGIN, CONTENT_Y + 70, 1, COL_GOOD);

      drawFooter("PAIR");
      break;
    }
    case BLE_SCR_PAIRING:
      drawTextC("PAIRING...", 0, SCR_W, CONTENT_Y + 28, 1, COL_PRIMARY);
      drawTextC(bleDevices[selectedBLEDevice].name, 0, SCR_W, CONTENT_Y + 46, 1, COL_TEXT);
      drawFooter("", COL_SECONDARY);
      break;
    case BLE_SCR_CONNECTED:
      drawTextC("CONNECTED", 0, SCR_W, CONTENT_Y + 28, 1, COL_GOOD);
      drawTextC(bleDevices[selectedBLEDevice].name, 0, SCR_W, CONTENT_Y + 46, 1, COL_TEXT);
      drawFooter("", COL_SECONDARY);
      break;
    case BLE_SCR_FAILED:
      // bleFailReason distinguishes NOT CONNECTABLE from a generic
      // CONNECTION FAILED — see connectBLEDevice().
      drawTextC(bleFailReason, 0, SCR_W, CONTENT_Y + 28, 1, COL_CRIT);
      drawTextC(bleDevices[selectedBLEDevice].name, 0, SCR_W, CONTENT_Y + 46, 1, COL_TEXT);
      drawFooter("RETRY", COL_WARN);
      break;
  }
  drawScreenFrame();
}

// ---------- PHONE LINK ----------
void drawPhoneLinkScreen() {
  drawHeader("PHONE LINK");

  const char* statusText; uint16_t statusColor;
  switch (phoneState) {
    case PHONE_NOT_PAIRED: statusText = "DISCONNECTED"; statusColor = COL_CRIT; break;
    case PHONE_PAIRING:    statusText = "PAIRING";       statusColor = COL_WARN; break;
    case PHONE_CONNECTED:  statusText = "CONNECTED";     statusColor = COL_GOOD; break;
    case PHONE_SYNCING:    statusText = "SYNCING";       statusColor = COL_WARN; break;
    default:               statusText = "SYNC COMPLETE"; statusColor = COL_GOOD; break;
  }

  drawTextL("BLUETOOTH", MARGIN, CONTENT_Y + 4, 1, COL_SECONDARY);
  drawTextR(connData.phoneConnected ? "CONNECTED" : "DISCONNECTED",
            SCR_W - MARGIN, CONTENT_Y + 4, 1, connData.phoneConnected ? COL_GOOD : COL_CRIT);

  // PHONE mirrors the real MORPHIX BLE GATT server link state (a phone is
  // "connected" only once the Android app's GATT client has subscribed) —
  // never assumes any arbitrary BLE peer is a phone.
  drawTextL("PHONE", MARGIN, CONTENT_Y + 20, 1, COL_SECONDARY);
  drawTextR(statusText, SCR_W - MARGIN, CONTENT_Y + 20, 1, statusColor);

  drawTextL("NOTIFICATIONS", MARGIN, CONTENT_Y + 36, 1, COL_SECONDARY);
  drawTextR(connData.notifAccessEnabled ? "ENABLED" : "DISABLED",
            SCR_W - MARGIN, CONTENT_Y + 36, 1, connData.notifAccessEnabled ? COL_GOOD : COL_CRIT);

  drawTextL("LAST SYNC", MARGIN, CONTENT_Y + 52, 1, COL_SECONDARY);
  drawTextR(connData.phoneLastSync, SCR_W - MARGIN, CONTENT_Y + 52, 1, COL_TEXT);

  // BATTERY only shown once the phone has actually reported one — no
  // fabricated percentage while phoneBatteryPct is still -1 (unknown).
  if (connData.phoneBatteryPct >= 0) {
    char battBuf[6];
    snprintf(battBuf, sizeof(battBuf), "%d%%", connData.phoneBatteryPct);
    drawTextL("BATTERY", MARGIN, CONTENT_Y + 68, 1, COL_SECONDARY);
    drawTextR(battBuf, SCR_W - MARGIN, CONTENT_Y + 68, 1, COL_TEXT);
  }

  if (phoneState == PHONE_CONNECTED) {
    drawFooter("SYNC");
  } else {
    // NOT_PAIRED has no action here — pairing happens by opening the
    // MORPHIX companion app on the phone and connecting from there; this
    // screen just reflects the real BLE server link state.
    drawFooter("", COL_SECONDARY);
  }
  drawScreenFrame();
}

// ---------- MOTION (hub) ----------
const char* motionMenuLabels[3] = {"MOTION DATA", "GESTURES", "MOTION SETTINGS"};
int motionMenuIndex = 0;

void drawMotionScreen() {
  drawHeader("MOTION");

  const int rowH = 28;
  for (int i = 0; i < 3; i++) {
    int16_t y = CONTENT_Y + i * rowH;
    bool selected = (i == motionMenuIndex);

    const char* statusText; uint16_t statusColor;
    if (i == 0) {
      statusText = "LIVE"; statusColor = COL_PRIMARY;
    } else if (i == 1) {
      statusText = gestureControlEnabled ? "ENABLED" : "DISABLED";
      statusColor = gestureControlEnabled ? COL_GOOD : COL_SECONDARY;
    } else {
      statusText = sensitivityLabel(gestureSensitivity);
      statusColor = COL_SECONDARY;
    }
    drawMenuRow(y, rowH, motionMenuLabels[i], selected, statusText, statusColor);
  }

  drawFooter("SELECT");
  drawScreenFrame();
}

// ---------- MOTION DATA ----------
void drawMotionDataScreen() {
  drawHeader("MOTION");

  char sbuf[8];
  drawTextL("ACCEL", MARGIN, CONTENT_Y + 4, 1, COL_SECONDARY);
  fmtSigned(motionData.accelX, sbuf, 2);
  { char line[24]; snprintf(line, sizeof(line), "X%-7s", sbuf);
    drawTextL(line, MARGIN, CONTENT_Y + 16, 1, COL_TEXT); }
  fmtSigned(motionData.accelY, sbuf, 2);
  { char line[24]; snprintf(line, sizeof(line), "Y%-7s", sbuf);
    drawTextC(line, 0, SCR_W, CONTENT_Y + 16, 1, COL_TEXT); }
  fmtSigned(motionData.accelZ, sbuf, 2);
  { char line[24]; snprintf(line, sizeof(line), "Z%-7s", sbuf);
    drawTextR(line, SCR_W - MARGIN, CONTENT_Y + 16, 1, COL_TEXT); }

  drawDivider(CONTENT_Y + 30);

  drawTextL("GYRO", MARGIN, CONTENT_Y + 36, 1, COL_SECONDARY);
  fmtSigned(motionData.gyroX, sbuf, 1);
  { char line[24]; snprintf(line, sizeof(line), "X%-6s", sbuf);
    drawTextL(line, MARGIN, CONTENT_Y + 48, 1, COL_TEXT); }
  fmtSigned(motionData.gyroY, sbuf, 1);
  { char line[24]; snprintf(line, sizeof(line), "Y%-6s", sbuf);
    drawTextC(line, 0, SCR_W, CONTENT_Y + 48, 1, COL_TEXT); }
  fmtSigned(motionData.gyroZ, sbuf, 1);
  { char line[24]; snprintf(line, sizeof(line), "Z%-6s", sbuf);
    drawTextR(line, SCR_W - MARGIN, CONTENT_Y + 48, 1, COL_TEXT); }

  drawDivider(CONTENT_Y + 62);

  char pitchBuf[10], rollBuf[10], line2[28];
  fmtSigned(motionData.pitch, pitchBuf, 1);
  fmtSigned(motionData.roll, rollBuf, 1);
  snprintf(line2, sizeof(line2), "PITCH %s  ROLL %s", pitchBuf, rollBuf);
  drawTextC(line2, 0, SCR_W, CONTENT_Y + 70, 1, COL_PRIMARY);

  const char* motionStateText = !mpu6050Ready ? "OFFLINE" : (motionIsMoving ? "MOVING" : "STABLE");
  uint16_t motionStateColor = !mpu6050Ready ? COL_CRIT : (motionIsMoving ? COL_WARN : COL_GOOD);
  drawTextL("MOTION", MARGIN, CONTENT_Y + 82, 1, COL_SECONDARY);
  drawTextR(motionStateText, SCR_W / 2 - 4, CONTENT_Y + 82, 1, motionStateColor);
  drawTextL("GESTURE", SCR_W / 2 + 4, CONTENT_Y + 82, 1, COL_SECONDARY);
  drawTextR(gestureLabel(lastGesture), SCR_W - MARGIN, CONTENT_Y + 82, 1, COL_PRIMARY);

  drawFooter("", COL_SECONDARY);
  drawScreenFrame();
}

// Redraws only MOTION DATA's live value fields (accel XYZ, gyro XYZ,
// pitch/roll, motion state, gesture), each gated on its own displayed
// value changing. Header, "ACCEL"/"GYRO"/"MOTION"/"GESTURE" labels
// and dividers stay untouched. This is what the 250 ms motion tick in
// loop() actually calls — previously that tick set needsRedraw and
// forced the whole screen (including these labels) to repaint every
// 250 ms, which is exactly the kind of high-frequency full redraw
// that causes ghosting on fast-changing digits like accel/gyro.
void updateMotionDataDynamic() {
  char sbuf[8], line[24];

  static char lastAccelX[10] = "", lastAccelY[10] = "", lastAccelZ[10] = "";
  fmtSigned(motionData.accelX, sbuf, 2);
  snprintf(line, sizeof(line), "X%-7s", sbuf);
  if (displayValueChanged(lastAccelX, sizeof(lastAccelX), line)) {
    tft.fillRect(MARGIN, CONTENT_Y + 16, 50, 9, COL_BG);
    drawTextL(line, MARGIN, CONTENT_Y + 16, 1, COL_TEXT);
  }
  fmtSigned(motionData.accelY, sbuf, 2);
  snprintf(line, sizeof(line), "Y%-7s", sbuf);
  if (displayValueChanged(lastAccelY, sizeof(lastAccelY), line)) {
    tft.fillRect(SCR_W / 2 - 30, CONTENT_Y + 16, 60, 9, COL_BG);
    drawTextC(line, 0, SCR_W, CONTENT_Y + 16, 1, COL_TEXT);
  }
  fmtSigned(motionData.accelZ, sbuf, 2);
  snprintf(line, sizeof(line), "Z%-7s", sbuf);
  if (displayValueChanged(lastAccelZ, sizeof(lastAccelZ), line)) {
    tft.fillRect(SCR_W - MARGIN - 50, CONTENT_Y + 16, 50, 9, COL_BG);
    drawTextR(line, SCR_W - MARGIN, CONTENT_Y + 16, 1, COL_TEXT);
  }

  static char lastGyroX[10] = "", lastGyroY[10] = "", lastGyroZ[10] = "";
  fmtSigned(motionData.gyroX, sbuf, 1);
  snprintf(line, sizeof(line), "X%-6s", sbuf);
  if (displayValueChanged(lastGyroX, sizeof(lastGyroX), line)) {
    tft.fillRect(MARGIN, CONTENT_Y + 48, 46, 9, COL_BG);
    drawTextL(line, MARGIN, CONTENT_Y + 48, 1, COL_TEXT);
  }
  fmtSigned(motionData.gyroY, sbuf, 1);
  snprintf(line, sizeof(line), "Y%-6s", sbuf);
  if (displayValueChanged(lastGyroY, sizeof(lastGyroY), line)) {
    tft.fillRect(SCR_W / 2 - 26, CONTENT_Y + 48, 52, 9, COL_BG);
    drawTextC(line, 0, SCR_W, CONTENT_Y + 48, 1, COL_TEXT);
  }
  fmtSigned(motionData.gyroZ, sbuf, 1);
  snprintf(line, sizeof(line), "Z%-6s", sbuf);
  if (displayValueChanged(lastGyroZ, sizeof(lastGyroZ), line)) {
    tft.fillRect(SCR_W - MARGIN - 46, CONTENT_Y + 48, 46, 9, COL_BG);
    drawTextR(line, SCR_W - MARGIN, CONTENT_Y + 48, 1, COL_TEXT);
  }

  static char lastPitchRoll[28] = "";
  char pitchBuf[10], rollBuf[10], line2[28];
  fmtSigned(motionData.pitch, pitchBuf, 1);
  fmtSigned(motionData.roll, rollBuf, 1);
  snprintf(line2, sizeof(line2), "PITCH %s  ROLL %s", pitchBuf, rollBuf);
  if (displayValueChanged(lastPitchRoll, sizeof(lastPitchRoll), line2)) {
    tft.fillRect(0, CONTENT_Y + 70, SCR_W, 9, COL_BG);
    drawTextC(line2, 0, SCR_W, CONTENT_Y + 70, 1, COL_PRIMARY);
  }

  static int lastMotionState = -1;
  int motionState = !mpu6050Ready ? 0 : (motionIsMoving ? 1 : 2);
  if (motionState != lastMotionState) {
    lastMotionState = motionState;
    const char* motionStateText = !mpu6050Ready ? "OFFLINE" : (motionIsMoving ? "MOVING" : "STABLE");
    uint16_t motionStateColor = !mpu6050Ready ? COL_CRIT : (motionIsMoving ? COL_WARN : COL_GOOD);
    tft.fillRect(MARGIN + 40, CONTENT_Y + 82, SCR_W / 2 - 4 - (MARGIN + 40), 9, COL_BG);
    drawTextR(motionStateText, SCR_W / 2 - 4, CONTENT_Y + 82, 1, motionStateColor);
  }

  static char lastGestureShown[16] = "";
  const char* gestureText = gestureLabel(lastGesture);
  if (displayValueChanged(lastGestureShown, sizeof(lastGestureShown), gestureText)) {
    tft.fillRect(SCR_W / 2 + 40, CONTENT_Y + 82, SCR_W - MARGIN - (SCR_W / 2 + 40), 9, COL_BG);
    drawTextR(gestureText, SCR_W - MARGIN, CONTENT_Y + 82, 1, COL_PRIMARY);
  }
}

// ---------- GESTURES ----------
void drawGesturesScreen() {
  drawHeader("GESTURES");

  drawTextL("STATUS", MARGIN, CONTENT_Y + 6, 1, COL_SECONDARY);
  drawTextR(gestureControlEnabled ? "ENABLED" : "DISABLED", SCR_W - MARGIN, CONTENT_Y + 6, 1,
            gestureControlEnabled ? COL_GOOD : COL_SECONDARY);

  drawTextL("GESTURE", MARGIN, CONTENT_Y + 30, 1, COL_SECONDARY);
  drawTextR(gestureLabel(lastGesture), SCR_W - MARGIN, CONTENT_Y + 30, 1, COL_PRIMARY);

  drawTextL("ACTION", MARGIN, CONTENT_Y + 54, 1, COL_SECONDARY);
  drawTextR(lastActionLabel, SCR_W - MARGIN, CONTENT_Y + 54, 1, COL_TEXT);

  drawFooter(gestureControlEnabled ? "DISABLE" : "ENABLE",
             gestureControlEnabled ? COL_WARN : COL_GOOD);
  drawScreenFrame();
}

// ---------- MOTION SETTINGS ----------
const char* motionSettingsItems[] = {"GESTURE CONTROL", "MOTION WAKE", "SENSITIVITY", "CALIBRATION"};
const int MOTION_SETTINGS_COUNT = 4;
int motionSettingsIndex = 0;

bool calibrating = false;
bool calibrationDone = false;
unsigned long calibrationStartMs = 0;
const unsigned long CALIBRATION_MS = 1500;

void drawMotionSettingsScreen() {
  drawHeader("MOTION SETTINGS");

  const int rowH = 22;
  for (int i = 0; i < MOTION_SETTINGS_COUNT; i++) {
    int16_t y = CONTENT_Y + i * rowH;
    bool selected = (i == motionSettingsIndex);
    drawRowHighlight(MARGIN - 2, y + 2, SCR_W - 2 * (MARGIN - 2), rowH - 4, selected);
    drawTextL(motionSettingsItems[i], MARGIN + 6, y + (rowH - 8) / 2, 1,
              selected ? COL_PRIMARY : COL_TEXT);

    const char* valText;
    switch (i) {
      case 0: valText = gestureControlEnabled ? "ON" : "OFF"; break;
      case 1: valText = motionWakeEnabled ? "ON" : "OFF"; break;
      case 2: valText = sensitivityLabel(gestureSensitivity); break;
      default: valText = calibrating ? "..." : (calibrationDone ? "DONE" : "START"); break;
    }
    drawTextR(valText, SCR_W - MARGIN - 2, y + (rowH - 8) / 2, 1,
              selected ? COL_PRIMARY : COL_SECONDARY);
  }

  drawFooter("SELECT");
  drawScreenFrame();
}

// ---------- SETTINGS ----------
const char* settingsItems[] = {"DISPLAY", "SENSORS", "TIME", "DEVICE STATUS", "ABOUT"};
const int SETTINGS_COUNT = 5;
int settingsIndex = 0;

void drawSettingsScreen() {
  drawHeader("SETTINGS");

  const int rowH = 18;
  for (int i = 0; i < SETTINGS_COUNT; i++) {
    int16_t y = CONTENT_Y + i * rowH;
    bool selected = (i == settingsIndex);
    drawRowHighlight(MARGIN - 2, y + 1, SCR_W - 2 * (MARGIN - 2), rowH - 2, selected);
    drawTextL(settingsItems[i], MARGIN + 6, y + (rowH - 8) / 2, 1,
              selected ? COL_PRIMARY : COL_TEXT);
    if (selected) drawTextR(">", SCR_W - MARGIN - 2, y + (rowH - 8) / 2, 1, COL_PRIMARY);
  }

  drawFooter("SELECT");
  drawScreenFrame();
}

// ---------- DEVICE STATUS ----------
void drawDeviceStatusScreen() {
  drawHeader("DEVICE");

  unsigned long upSec = millis() / 1000;
  unsigned long h = upSec / 3600, m = (upSec % 3600) / 60, s = upSec % 60;
  char buf[20];

  const int rowH = 15;
  int y = CONTENT_Y + 2;

  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  drawTextL("UPTIME", MARGIN, y, 1, COL_SECONDARY);
  drawTextR(buf, SCR_W - MARGIN, y, 1, COL_TEXT); y += rowH;

  snprintf(buf, sizeof(buf), "%lu KB", (unsigned long)(ESP.getFreeHeap() / 1024));
  drawTextL("RAM FREE", MARGIN, y, 1, COL_SECONDARY);
  drawTextR(buf, SCR_W - MARGIN, y, 1, COL_TEXT); y += rowH;

  {
    unsigned long used = ESP.getSketchSize();
    unsigned long total = used + ESP.getFreeSketchSpace();
    int pct = total > 0 ? (int)(100UL * used / total) : 0;
    snprintf(buf, sizeof(buf), "%d%%", pct);
  }
  drawTextL("FLASH USED", MARGIN, y, 1, COL_SECONDARY);
  drawTextR(buf, SCR_W - MARGIN, y, 1, COL_TEXT); y += rowH;

  drawTextL("WIFI", MARGIN, y, 1, COL_SECONDARY);
  drawTextR(connData.wifiConnected ? "CONNECTED" : "READY", SCR_W - MARGIN, y, 1,
            connData.wifiConnected ? COL_GOOD : COL_PRIMARY); y += rowH;

  drawTextL("BLE", MARGIN, y, 1, COL_SECONDARY);
  drawTextR(connData.bleReady ? "READY" : "OFF", SCR_W - MARGIN, y, 1,
            connData.bleReady ? COL_PRIMARY : COL_CRIT); y += rowH;

  snprintf(buf, sizeof(buf), "%d%%", mockBatteryPct);
  drawTextL("BATTERY", MARGIN, y, 1, COL_SECONDARY);
  drawTextR(buf, SCR_W - MARGIN, y, 1, COL_TEXT);

  drawFooter("", COL_SECONDARY);
  drawScreenFrame();
}

// Redraws only DEVICE STATUS's per-row value fields (right-hand side
// of each row), each gated on its own displayed value changing. Row
// labels (UPTIME/RAM FREE/etc.), header and footer stay untouched.
// This is what the once-a-second device-status tick in loop() calls
// instead of forcing a full drawDeviceStatusScreen() repaint.
void updateDeviceStatusDynamic() {
  const int rowH = 15;
  int y = CONTENT_Y + 2;
  char buf[20];

  auto updateRow = [&](char* cache, size_t cacheSize, const char* val, int16_t rowY) {
    if (displayValueChanged(cache, cacheSize, val)) {
      tft.fillRect(SCR_W / 2, rowY, SCR_W - MARGIN - SCR_W / 2, 9, COL_BG);
      drawTextR(val, SCR_W - MARGIN, rowY, 1, COL_TEXT);
    }
  };

  static char lastUptime[20] = "";
  unsigned long upSec = millis() / 1000;
  unsigned long h = upSec / 3600, m = (upSec % 3600) / 60, s = upSec % 60;
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  updateRow(lastUptime, sizeof(lastUptime), buf, y); y += rowH;

  static char lastRam[20] = "";
  snprintf(buf, sizeof(buf), "%lu KB", (unsigned long)(ESP.getFreeHeap() / 1024));
  updateRow(lastRam, sizeof(lastRam), buf, y); y += rowH;

  static char lastFlash[20] = "";
  {
    unsigned long used = ESP.getSketchSize();
    unsigned long total = used + ESP.getFreeSketchSpace();
    int pct = total > 0 ? (int)(100UL * used / total) : 0;
    snprintf(buf, sizeof(buf), "%d%%", pct);
  }
  updateRow(lastFlash, sizeof(lastFlash), buf, y); y += rowH;

  // WIFI/BLE rows change color as well as text, so those two are
  // handled by the dedicated state-change redraws elsewhere in loop()
  // (see the WiFi-status and BLE sections) rather than here — this
  // keeps a single source of truth for each row instead of two
  // competing redraw paths for the same region.
  y += rowH; y += rowH;

  static char lastBattery[20] = "";
  snprintf(buf, sizeof(buf), "%d%%", mockBatteryPct);
  updateRow(lastBattery, sizeof(lastBattery), buf, y);
}

// ---------- DISPLAY / SENSORS / TIME / ABOUT (settings detail pages) ----------
void drawDisplaySettingsScreen() {
  drawHeader("DISPLAY");
  drawTextL("BRIGHTNESS", MARGIN, CONTENT_Y + 6, 1, COL_SECONDARY);
  drawProgressBar(MARGIN, CONTENT_Y + 18, SCR_W - 2 * MARGIN, 12, 80, COL_PRIMARY);
  drawTextC("AUTO-DIM: OFF", 0, SCR_W, CONTENT_Y + 44, 1, COL_SECONDARY);
  drawTextC("MORE OPTIONS COMING SOON", 0, SCR_W, CONTENT_Y + 64, 1, COL_SECONDARY);
  drawFooter("", COL_SECONDARY);
  drawScreenFrame();
}

void drawSensorsSettingsScreen() {
  drawHeader("SENSORS");
  const char* names[4]  = {"BMP280", "MAX30102", "MPU6050", "CONNECTIVITY"};
  char detailBuf[28];
  for (int i = 0; i < 4; i++) {
    int16_t y = CONTENT_Y + 2 + i * 21;
    drawTextL(names[i], MARGIN, y, 1, COL_TEXT);

    const char* status;
    uint16_t color;
    const char* detail = NULL;
    switch (i) {
      case 0: // BMP280 -> temperature/pressure
        status = bmp280Ready ? "LIVE" : "OFFLINE";
        color  = bmp280Ready ? COL_GOOD : COL_CRIT;
        if (bmp280Ready) {
          snprintf(detailBuf, sizeof(detailBuf), "%.1fC  %.0fhPa", realTemperatureC, realPressureHpa);
          detail = detailBuf;
        }
        break;
      case 1: // MAX30102 -> HR/SpO2/contact/signal
        if (!max30102Ready) {
          status = "OFFLINE";
          color  = COL_CRIT;
        } else if (!isMAX30102SignalGood()) {
          status = "NO SIGNAL";
          color  = COL_WARN;
        } else {
          status = "LIVE";
          color  = COL_GOOD;
        }
        if (max30102Ready) {
          int hr = getStableHeartRate();
          char hrBuf[6];
          if (hr > 0) snprintf(hrBuf, sizeof(hrBuf), "%d", hr);
          else        snprintf(hrBuf, sizeof(hrBuf), "--");
          snprintf(detailBuf, sizeof(detailBuf), "%s %s  %s",
                    hrBuf,
                    max30102Contact ? "CONTACT" : "NO SKIN",
                    max30102SignalGood ? "GOOD SIG" : "WEAK SIG");
          detail = detailBuf;
        }
        break;
      case 2: // MPU6050 -> motion/orientation/gesture
        status = mpu6050Ready ? "LIVE" : "OFFLINE";
        color  = mpu6050Ready ? COL_GOOD : COL_CRIT;
        if (mpu6050Ready) {
          snprintf(detailBuf, sizeof(detailBuf), "%s  %s",
                    motionIsMoving ? "MOVING" : "STABLE",
                    gestureControlEnabled ? "GESTURES ON" : "GESTURES OFF");
          detail = detailBuf;
        }
        break;
      default: // CONNECTIVITY -> real WiFi/BLE link state
        status = (connData.wifiConnected || connData.bleConnected) ? "LIVE" : "READY";
        color  = (connData.wifiConnected || connData.bleConnected) ? COL_GOOD : COL_PRIMARY;
        break;
    }
    drawTextR(status, SCR_W - MARGIN, y, 1, color);
    if (detail != NULL) {
      drawTextL(detail, MARGIN + 8, y + 9, 1, COL_SECONDARY);
    }
  }
  drawFooter("", COL_SECONDARY);
  drawScreenFrame();
}

// Redraws only SENSORS's per-row status/detail fields (right-hand
// status word + the optional detail line under each row name), each
// gated on its own displayed text changing. Row name labels (BMP280/
// MAX30102/MPU6050/CONNECTIVITY), header and footer stay untouched —
// they were already painted once by drawSensorsSettingsScreen() on
// screen entry. This is what the once-a-second sensors-settings tick
// in loop() calls instead of forcing a full drawSensorsSettingsScreen()
// repaint (which, since a same-screen redraw never fillScreen()s,
// would otherwise leave stale text ghosted behind shorter new text).
void updateSensorsSettingsDynamic() {
  static char lastStatus[4][12] = {"", "", "", ""};
  static char lastDetail[4][28] = {"", "", "", ""};

  char detailBuf[28];

  for (int i = 0; i < 4; i++) {
    int16_t y = CONTENT_Y + 2 + i * 21;

    const char* status;
    uint16_t color;
    const char* detail = NULL;
    switch (i) {
      case 0:
        status = bmp280Ready ? "LIVE" : "OFFLINE";
        color  = bmp280Ready ? COL_GOOD : COL_CRIT;
        if (bmp280Ready) {
          snprintf(detailBuf, sizeof(detailBuf), "%.1fC  %.0fhPa", realTemperatureC, realPressureHpa);
          detail = detailBuf;
        }
        break;
      case 1:
        if (!max30102Ready) {
          status = "OFFLINE";
          color  = COL_CRIT;
        } else if (!isMAX30102SignalGood()) {
          status = "NO SIGNAL";
          color  = COL_WARN;
        } else {
          status = "LIVE";
          color  = COL_GOOD;
        }
        if (max30102Ready) {
          int hr = getStableHeartRate();
          char hrBuf[6];
          if (hr > 0) snprintf(hrBuf, sizeof(hrBuf), "%d", hr);
          else        snprintf(hrBuf, sizeof(hrBuf), "--");
          snprintf(detailBuf, sizeof(detailBuf), "%s %s  %s",
                    hrBuf,
                    max30102Contact ? "CONTACT" : "NO SKIN",
                    max30102SignalGood ? "GOOD SIG" : "WEAK SIG");
          detail = detailBuf;
        }
        break;
      case 2:
        status = mpu6050Ready ? "LIVE" : "OFFLINE";
        color  = mpu6050Ready ? COL_GOOD : COL_CRIT;
        if (mpu6050Ready) {
          snprintf(detailBuf, sizeof(detailBuf), "%s  %s",
                    motionIsMoving ? "MOVING" : "STABLE",
                    gestureControlEnabled ? "GESTURES ON" : "GESTURES OFF");
          detail = detailBuf;
        }
        break;
      default:
        status = (connData.wifiConnected || connData.bleConnected) ? "LIVE" : "READY";
        color  = (connData.wifiConnected || connData.bleConnected) ? COL_GOOD : COL_PRIMARY;
        break;
    }

    if (displayValueChanged(lastStatus[i], sizeof(lastStatus[i]), status)) {
      // Status word only, right-aligned on the row's first line —
      // name label to the left stays untouched.
      tft.fillRect(SCR_W / 2, y, SCR_W - MARGIN - SCR_W / 2, 9, COL_BG);
      drawTextR(status, SCR_W - MARGIN, y, 1, color);
    }

    const char* detailToShow = (detail != NULL) ? detail : "";
    if (displayValueChanged(lastDetail[i], sizeof(lastDetail[i]), detailToShow)) {
      tft.fillRect(MARGIN + 8, y + 9, SCR_W - MARGIN - (MARGIN + 8), 9, COL_BG);
      if (detail != NULL) {
        drawTextL(detail, MARGIN + 8, y + 9, 1, COL_SECONDARY);
      }
    }
  }
}

void drawTimeSettingsScreen() {
  drawHeader("TIME");
  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", mockHour, mockMinute, mockSecond);
  drawTextC(buf, 0, SCR_W, CONTENT_Y + 16, 2, COL_TEXT);
  snprintf(buf, sizeof(buf), "%s %s", mockDay, mockDate);
  drawTextC(buf, 0, SCR_W, CONTENT_Y + 44, 1, COL_SECONDARY);
  drawTextC("SYNC SOURCE: MANUAL", 0, SCR_W, CONTENT_Y + 64, 1, COL_SECONDARY);
  drawFooter("", COL_SECONDARY);
  drawScreenFrame();
}

void drawAboutScreen() {
  drawHeader("ABOUT");
  drawTextC("MORPHIX", 0, SCR_W, CONTENT_Y + 14, 2, COL_PRIMARY);
  drawTextC("WEARABLE OS v0.2", 0, SCR_W, CONTENT_Y + 42, 1, COL_SECONDARY);
  drawTextC("BUILT BY REHAN KHAN", 0, SCR_W, CONTENT_Y + 58, 1, COL_SECONDARY);
  drawTextC("SEEED XIAO ESP32-S3", 0, SCR_W, CONTENT_Y + 74, 1, COL_SECONDARY);
  drawFooter("", COL_SECONDARY);
  drawScreenFrame();
}

// ============================================================
// 13. NAVIGATION
// ============================================================
// Top-level screens (indices 0..SCR_TOP_COUNT-1) are the ones cycled by
// UP/DOWN and by DEMO_AUTOPLAY when the user is at the top level. Every
// screen after SCR_TOP_COUNT is a sub-screen reached via SELECT from a
// hub screen (CONNECTIVITY, MOTION, SETTINGS) and returns to that hub
// on BACK — see parentOf[] below.
// Where BACK sends you from each screen. Top-level screens all return
// to HOME (matches the original app's behavior); sub-screens return to
// the hub they were opened from.
const ScreenID parentOf[SCR_COUNT] = {
  SCR_HOME, SCR_HOME, SCR_HOME, SCR_HOME, SCR_HOME, SCR_HOME, SCR_HOME, SCR_HOME, SCR_HOME,  // top level (incl. STORAGE)
  SCR_CONNECTIVITY, SCR_CONNECTIVITY, SCR_CONNECTIVITY,                             // wifi, ble, phone link
  SCR_MOTION, SCR_MOTION, SCR_MOTION,                                               // motion data, gestures, motion settings
  SCR_SETTINGS, SCR_SETTINGS, SCR_SETTINGS, SCR_SETTINGS, SCR_SETTINGS              // display, sensors, time, device status, about
};

// What SELECT opens from each hub screen, indexed by that hub's own
// selection index.
const ScreenID connMenuTargets[3]   = {SCR_WIFI, SCR_BLUETOOTH, SCR_PHONELINK};
const ScreenID motionMenuTargets[3] = {SCR_MOTION_DATA, SCR_GESTURES, SCR_MOTION_SETTINGS};
const ScreenID settingsTargets[SETTINGS_COUNT] = {
  SCR_DISPLAY_SETTINGS, SCR_SENSORS_SETTINGS, SCR_TIME_SETTINGS, SCR_DEVICE_STATUS, SCR_ABOUT
};

// needsRedraw, currentScreen, and lastRenderedScreen are all declared
// earlier (near the top globals) since the WiFi backend functions
// above (and, for lastRenderedScreen, the Storage subsystem's
// storageSdEndSession()) need them and are defined before this point
// in the file. lastRenderedScreen tracks which screen is actually on
// the glass right now — compared against currentScreen at redraw time
// to tell a real screen TRANSITION (which legitimately needs one
// fillScreen() + one full draw of that screen's static chrome) apart
// from an in-place VALUE UPDATE on the screen already being shown
// (which must never fillScreen() or repaint static chrome — only the
// specific dynamic region that changed). Its sentinel -1 forces the
// very first redraw after boot to be treated as a transition.

// --- DEMO AUTOPLAY ---
// Previously used to self-demo the UI while no buttons were wired: it
// automatically cycled through every TOP-LEVEL screen on a timer. Real
// buttons are now wired and verified, so autoplay is switched off per
// the plan noted above — manual UP/DOWN/BACK/SELECT navigation in
// handleButtons() works either way and always takes priority for the
// screen it's on. Autoplay pauses while the user is inside a sub-screen
// (WiFi, Bluetooth, Motion Data, etc.) so it never yanks them out of
// something they navigated into on purpose — see loop(). Flip this back
// to true if you ever want the self-demo behavior again.
const bool DEMO_AUTOPLAY = false;
const unsigned long SCREEN_HOLD_MS  = 3000;  // how long each top-level screen stays up
const unsigned long SUBITEM_HOLD_MS = 900;   // settings highlight / notif paging rate

unsigned long lastScreenChangeMs = 0;
unsigned long lastSubItemMs = 0;

// Central place to switch screens so the demo timer and manual buttons
// both reset state (selection indices, sub-item timer) the same way.
// NOTE: takes a plain int (not ScreenID) and casts internally — this
// sidesteps a known Arduino IDE quirk where its auto-generated function
// prototypes are inserted above custom enum/type definitions, which
// would otherwise cause a "type not declared in this scope" error.
void goToScreen(int s) {
  ScreenID previousScreen = currentScreen;
  currentScreen = (ScreenID)s;
  settingsIndex = 0;
  notifIndex = 0;
  connIndex = 0;
  motionMenuIndex = 0;
  motionSettingsIndex = 0;
  if (currentScreen == SCR_WIFI) {
    wifiScreenState = WIFI_SCR_STATUS;
    wifiListIndex = 0; wifiListScroll = 0; selectedWifiNetwork = 0;
  }
  if (currentScreen == SCR_BLUETOOTH) {
    bleScreenState = BLE_SCR_STATUS;
    bleListIndex = 0; bleListScroll = 0; selectedBLEDevice = 0;
  }
  if (currentScreen == SCR_STORAGE) {
    closeStorageFiles(); // always enter fresh at STORAGE_SCR_HOME
    if (!storageMounted) initStorage(); // rate-limited inside initStorage(); safe to call every entry
  } else if (previousScreen == SCR_STORAGE) {
    closeStorageFiles(); // leaving Storage for another top-level screen: close anything open
  }
  needsRedraw = true;
  lastScreenChangeMs = millis();
  lastSubItemMs = millis();
}

// Draws the full static+dynamic content of whichever screen is current.
// Called once per real screen ENTRY (see loop()'s redraw block) — never
// on every loop() pass — so headers/footers/dividers/labels are painted
// exactly once per visit instead of being continuously repainted.
void renderCurrentScreen() {
  switch (currentScreen) {
    case SCR_HOME:              drawHomeScreen(); break;
    case SCR_ENV:                drawEnvironmentScreen(); break;
    case SCR_HEALTH:             drawHealthScreen(); break;
    case SCR_ACTIVITY:           drawActivityScreen(); break;
    case SCR_NOTIF:              drawNotificationsScreen(); break;
    case SCR_CONNECTIVITY:       drawConnectivityScreen(); break;
    case SCR_MOTION:             drawMotionScreen(); break;
    case SCR_SETTINGS:           drawSettingsScreen(); break;
    case SCR_STORAGE:             drawStorageScreen(); break;
    case SCR_WIFI:                drawWifiScreen(); break;
    case SCR_BLUETOOTH:           drawBluetoothScreen(); break;
    case SCR_PHONELINK:           drawPhoneLinkScreen(); break;
    case SCR_MOTION_DATA:         drawMotionDataScreen(); break;
    case SCR_GESTURES:            drawGesturesScreen(); break;
    case SCR_MOTION_SETTINGS:     drawMotionSettingsScreen(); break;
    case SCR_DISPLAY_SETTINGS:    drawDisplaySettingsScreen(); break;
    case SCR_SENSORS_SETTINGS:    drawSensorsSettingsScreen(); break;
    case SCR_TIME_SETTINGS:       drawTimeSettingsScreen(); break;
    case SCR_DEVICE_STATUS:       drawDeviceStatusScreen(); break;
    case SCR_ABOUT:               drawAboutScreen(); break;
    default: break;
  }
}

// ============================================================
// 14. MOTION & GESTURE ENGINE (UI-side half)
// ============================================================
// Clean pipeline, kept out of the drawing code entirely:
//   updateMPU6050() -> detectGesture() -> handleGesture() -> navigation
// updateMPU6050()/detectGesture() live in section 6c (no navigation
// dependency needed for detection itself); handleGesture() lives here
// because it needs ScreenID/goToScreen(), which section 6c predates.
// detectGesture() itself (the real threshold-based engine, gated by
// gestureControlEnabled and fully debounced/cooldown/re-armed) is
// defined in section 6c — see GESTURE ENGINE there.

// Maps each real gesture to a UI action, per the requested mapping:
// TILT UP/DOWN -> navigate, TILT LEFT -> back, TILT RIGHT -> select/open,
// TWIST CW/CCW -> next/prev item, SHAKE -> return home.
void handleGesture(int gArg) {
  GestureType g = (GestureType)gArg;
  if (g == GESTURE_NONE) return;
  lastGesture = g;

  switch (g) {
    case GESTURE_TILT_UP:
      strcpy(lastActionLabel, "NAV UP");
      if (currentScreen < SCR_TOP_COUNT) goToScreen((currentScreen - 1 + SCR_TOP_COUNT) % SCR_TOP_COUNT);
      break;
    case GESTURE_TILT_DOWN:
      strcpy(lastActionLabel, "NAV DOWN");
      if (currentScreen < SCR_TOP_COUNT) goToScreen((currentScreen + 1) % SCR_TOP_COUNT);
      break;
    case GESTURE_TILT_LEFT:
      strcpy(lastActionLabel, "BACK");
      goToScreen(parentOf[currentScreen]);
      break;
    case GESTURE_TILT_RIGHT:
      strcpy(lastActionLabel, "SELECT/OPEN");
      // Reuses the same per-screen SELECT behavior as the physical
      // button for hub screens; elsewhere it's a no-op beyond the label.
      if (currentScreen == SCR_CONNECTIVITY) goToScreen(connMenuTargets[connIndex]);
      else if (currentScreen == SCR_MOTION)   goToScreen(motionMenuTargets[motionMenuIndex]);
      else if (currentScreen == SCR_SETTINGS) goToScreen(settingsTargets[settingsIndex]);
      break;
    case GESTURE_TWIST_CW:
      strcpy(lastActionLabel, "NEXT ITEM");
      break;
    case GESTURE_TWIST_CCW:
      strcpy(lastActionLabel, "PREV ITEM");
      break;
    case GESTURE_SHAKE:
      strcpy(lastActionLabel, "HOME");
      goToScreen(SCR_HOME);
      break;
    default:
      break;
  }

  if (currentScreen == SCR_GESTURES) needsRedraw = true;
}

// ============================================================
// 15. BUTTON HANDLING (navigation logic)
// ============================================================
void handleButtons() {
  if (buttonPressed(btnUp)) {
    Serial.println("BUTTON: UP");
    if (currentScreen == SCR_SETTINGS) {
      settingsIndex = (settingsIndex - 1 + SETTINGS_COUNT) % SETTINGS_COUNT;
      needsRedraw = true;
    } else if (currentScreen == SCR_NOTIF && notifCount > 0) {
      notifIndex = (notifIndex - 1 + notifCount) % notifCount;
      needsRedraw = true;
    } else if (currentScreen == SCR_CONNECTIVITY) {
      connIndex = (connIndex - 1 + 3) % 3;
      needsRedraw = true;
    } else if (currentScreen == SCR_MOTION) {
      motionMenuIndex = (motionMenuIndex - 1 + 3) % 3;
      needsRedraw = true;
    } else if (currentScreen == SCR_MOTION_SETTINGS) {
      motionSettingsIndex = (motionSettingsIndex - 1 + MOTION_SETTINGS_COUNT) % MOTION_SETTINGS_COUNT;
      needsRedraw = true;
    } else if (currentScreen == SCR_WIFI && wifiScreenState == WIFI_SCR_LIST && wifiNetworkCount > 0) {
      wifiListIndex = (wifiListIndex - 1 + wifiNetworkCount) % wifiNetworkCount;
      clampListScroll(wifiListScroll, wifiListIndex, wifiNetworkCount);
      needsRedraw = true;
    } else if (currentScreen == SCR_BLUETOOTH && bleScreenState == BLE_SCR_LIST && bleDeviceCount > 0) {
      bleListIndex = (bleListIndex - 1 + bleDeviceCount) % bleDeviceCount;
      clampListScroll(bleListScroll, bleListIndex, bleDeviceCount);
      needsRedraw = true;
    } else if (currentScreen == SCR_STORAGE) {
      handleStorageInput('U');
    } else if (currentScreen < SCR_TOP_COUNT) {
      goToScreen((currentScreen - 1 + SCR_TOP_COUNT) % SCR_TOP_COUNT);
    }
  }

  if (buttonPressed(btnDown)) {
    Serial.println("BUTTON: DOWN");
    if (currentScreen == SCR_SETTINGS) {
      settingsIndex = (settingsIndex + 1) % SETTINGS_COUNT;
      needsRedraw = true;
    } else if (currentScreen == SCR_NOTIF && notifCount > 0) {
      notifIndex = (notifIndex + 1) % notifCount;
      needsRedraw = true;
    } else if (currentScreen == SCR_CONNECTIVITY) {
      connIndex = (connIndex + 1) % 3;
      needsRedraw = true;
    } else if (currentScreen == SCR_MOTION) {
      motionMenuIndex = (motionMenuIndex + 1) % 3;
      needsRedraw = true;
    } else if (currentScreen == SCR_MOTION_SETTINGS) {
      motionSettingsIndex = (motionSettingsIndex + 1) % MOTION_SETTINGS_COUNT;
      needsRedraw = true;
    } else if (currentScreen == SCR_WIFI && wifiScreenState == WIFI_SCR_LIST && wifiNetworkCount > 0) {
      wifiListIndex = (wifiListIndex + 1) % wifiNetworkCount;
      clampListScroll(wifiListScroll, wifiListIndex, wifiNetworkCount);
      needsRedraw = true;
    } else if (currentScreen == SCR_BLUETOOTH && bleScreenState == BLE_SCR_LIST && bleDeviceCount > 0) {
      bleListIndex = (bleListIndex + 1) % bleDeviceCount;
      clampListScroll(bleListScroll, bleListIndex, bleDeviceCount);
      needsRedraw = true;
    } else if (currentScreen == SCR_STORAGE) {
      handleStorageInput('D');
    } else if (currentScreen < SCR_TOP_COUNT) {
      goToScreen((currentScreen + 1) % SCR_TOP_COUNT);
    }
  }

  if (buttonPressed(btnBack)) {
    Serial.println("BUTTON: BACK");
    if (currentScreen == SCR_WIFI && wifiScreenState != WIFI_SCR_STATUS) {
      switch (wifiScreenState) {
        case WIFI_SCR_SCANNING:
          // The background scan keeps running; pollWifiScan() will just
          // discard the results since we're no longer in SCANNING state.
          wifiScreenState = WIFI_SCR_STATUS;
          break;
        case WIFI_SCR_LIST:    wifiScreenState = WIFI_SCR_STATUS;  break;
        case WIFI_SCR_DETAILS: wifiScreenState = WIFI_SCR_LIST;    break;
        case WIFI_SCR_NOCRED:  wifiScreenState = WIFI_SCR_DETAILS; break;
        case WIFI_SCR_CONNECTING:
          WiFi.disconnect(true); // actually abort the in-progress attempt
          wifiScreenState = WIFI_SCR_DETAILS;
          break;
        case WIFI_SCR_CONNECTED: wifiScreenState = WIFI_SCR_STATUS; break; // skip the hold
        case WIFI_SCR_FAILED:    wifiScreenState = WIFI_SCR_LIST;   break; // try a different network
        default: break;
      }
      needsRedraw = true;
    } else if (currentScreen == SCR_BLUETOOTH && bleScreenState != BLE_SCR_STATUS) {
      switch (bleScreenState) {
        case BLE_SCR_SCANNING:  bleScreenState = BLE_SCR_STATUS;  break; // cancel scan
        case BLE_SCR_LIST:      bleScreenState = BLE_SCR_STATUS;  break;
        case BLE_SCR_DETAILS:   bleScreenState = BLE_SCR_LIST;    break;
        case BLE_SCR_PAIRING:   bleScreenState = BLE_SCR_DETAILS; break; // cancel pairing
        case BLE_SCR_CONNECTED: bleScreenState = BLE_SCR_STATUS;  break; // skip the hold
        case BLE_SCR_FAILED:    bleScreenState = BLE_SCR_LIST;    break; // try a different device
        default: break;
      }
      needsRedraw = true;
    } else if (currentScreen == SCR_PHONELINK && phoneState == PHONE_SYNCING) {
      phoneState = PHONE_CONNECTED; // cancel the (mock) sync, real link stays up
      needsRedraw = true;
    } else if (currentScreen == SCR_STORAGE) {
      if (handleStorageInput('B')) goToScreen(parentOf[currentScreen]); // only climbs out of Storage from STORAGE_SCR_HOME
    } else {
      goToScreen(parentOf[currentScreen]);
    }
  }

  if (buttonPressed(btnSelect)) {
    Serial.println("BUTTON: SELECT");
    switch (currentScreen) {
      case SCR_HOME:     goToScreen(SCR_SETTINGS); break;                          // MENU
      case SCR_ENV:
      case SCR_ACTIVITY: manualSensorSync(); needsRedraw = true; break;            // SYNC
      case SCR_HEALTH:   manualHealthSync(); needsRedraw = true; break;            // SYNC (MAX30102)
      case SCR_NOTIF:    dismissNotification(); needsRedraw = true; break;         // DISMISS

      case SCR_CONNECTIVITY: goToScreen(connMenuTargets[connIndex]); break;
      case SCR_MOTION:        goToScreen(motionMenuTargets[motionMenuIndex]); break;
      case SCR_SETTINGS:      goToScreen(settingsTargets[settingsIndex]); break;
      case SCR_STORAGE:        handleStorageInput('S'); break;

      case SCR_WIFI:
        switch (wifiScreenState) {
          case WIFI_SCR_STATUS:
            if (connData.wifiConnected) {
              disconnectWiFi();
            } else {
              wifiScreenState = WIFI_SCR_SCANNING;
              wifiStateTimerMs = millis();
              startWifiScan(); // real, non-blocking WiFi.scanNetworks(true)
            }
            break;
          case WIFI_SCR_LIST:
            if (wifiNetworkCount > 0) {
              selectedWifiNetwork = wifiListIndex;
              wifiScreenState = WIFI_SCR_DETAILS;
            }
            break;
          case WIFI_SCR_DETAILS:
            attemptWifiConnect(selectedWifiNetwork); // moves to CONNECTING or NOCRED
            break;
          case WIFI_SCR_FAILED: // RETRY
            attemptWifiConnect(selectedWifiNetwork);
            break;
          default: break; // scanning/connecting/connected/nocred auto-transition or BACK-only, ignore SELECT
        }
        needsRedraw = true;
        break;

      case SCR_BLUETOOTH:
        switch (bleScreenState) {
          case BLE_SCR_STATUS:
            if (connData.bleConnected) {
              disconnectBLEDevice();
            } else {
              bleScreenState = BLE_SCR_SCANNING;
              startBLEScan(); // real, non-blocking BLE scan
            }
            break;
          case BLE_SCR_LIST:
            if (bleDeviceCount > 0) {
              selectedBLEDevice = bleListIndex;
              bleScreenState = BLE_SCR_DETAILS;
            }
            break;
          case BLE_SCR_DETAILS:
            bleScreenState = BLE_SCR_PAIRING;
            bleStateTimerMs = millis(); // brief delay so "PAIRING..." paints before the connect() call blocks
            break;
          case BLE_SCR_FAILED: // RETRY
            bleScreenState = BLE_SCR_PAIRING;
            bleStateTimerMs = millis();
            break;
          default: break;
        }
        needsRedraw = true;
        break;

      case SCR_PHONELINK:
        // Pairing itself happens by opening the MORPHIX app on the phone
        // and connecting from there — this screen just reflects the real
        // BLE server link. SELECT here asks the phone for a real
        // resync (WCMD_REQUEST_SYNC over CHAR_COMMAND_RX); the
        // SYNCING/SYNC_DONE hold is just UI feedback timing.
        if (phoneState == PHONE_CONNECTED) {
          phoneState = PHONE_SYNCING;
          phoneStateTimerMs = millis();
          sendCommandToPhone(WCMD_REQUEST_SYNC, nullptr, 0);
        }
        needsRedraw = true;
        break;

      case SCR_MOTION_DATA:
        needsRedraw = true; // motion data already updates continuously; SELECT just forces a redraw
        break;

      case SCR_GESTURES:
        gestureControlEnabled = !gestureControlEnabled;
        needsRedraw = true;
        break;

      case SCR_MOTION_SETTINGS:
        switch (motionSettingsIndex) {
          case 0: gestureControlEnabled = !gestureControlEnabled; break;
          case 1: motionWakeEnabled = !motionWakeEnabled; break;
          case 2: gestureSensitivity = (GestureSensitivity)(((int)gestureSensitivity + 1) % 3); break;
          case 3:
            calibrating = true;
            calibrationDone = false;
            calibrationStartMs = millis();
            break;
        }
        needsRedraw = true;
        break;

      default: needsRedraw = true; break; // informational screens: SELECT just re-renders
    }
  }
}

// ============================================================
// 16. setup()
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BACK, INPUT_PULLUP);
  pinMode(PIN_SELECT, INPUT_PULLUP);

  // Seed each Button's raw state from the pin's current level (idle =
  // HIGH = released) before interrupts start, so there's no false edge
  // the instant attachInterrupt() below goes live.
  btnUp.lastReading     = (digitalRead(PIN_UP)     == LOW);
  btnDown.lastReading   = (digitalRead(PIN_DOWN)   == LOW);
  btnBack.lastReading   = (digitalRead(PIN_BACK)   == LOW);
  btnSelect.lastReading = (digitalRead(PIN_SELECT) == LOW);

  // CHANGE interrupts so both the press edge and the release edge are
  // timestamped the instant they happen, independent of how long
  // anything else in loop() (TFT redraw, WiFi/BLE work, sensor I2C)
  // takes to run. See isrButtonUp()/etc. above for why this matters.
  attachInterrupt(digitalPinToInterrupt(PIN_UP),     isrButtonUp,     CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_DOWN),   isrButtonDown,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BACK),   isrButtonBack,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SELECT), isrButtonSelect, CHANGE);

  // ---------- Wi-Fi ----------
  // Built into the ESP32-S3, no extra pins/hardware. Station mode, start
  // disconnected — the user can still drive manual scanning/connecting from
  // the WIFI screen; the background auto-connect/auto-reconnect state
  // machine (started below) independently attaches to WIFI_SSID.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  connData.wifiConnected = false;

  // ---------- Real-time clock: restore last-known time from NVS ----------
  // Opens the "time" NVS namespace used by serviceNtpSync() to persist the
  // last successful sync. If a previous boot ever synced, seed the system
  // clock from that stored value now so the UI has a continuing (if stale)
  // clock immediately instead of blanking on every reboot. This does NOT
  // set ntpSynced — that only ever becomes true after a real NTP sync this
  // boot — so nothing here falsely claims a fresh synchronization.
  timePrefs.begin("time", false);
  uint32_t savedEpoch = timePrefs.getULong("lastEpoch", 0);
  if (savedEpoch >= (uint32_t)NTP_VALID_EPOCH_THRESHOLD) {
    struct timeval tv = { (time_t)savedEpoch, 0 };
    settimeofday(&tv, nullptr);
    setenv("TZ", MORPHIX_TZ, 1);
    tzset();
    clockHasValidTime = true;
  }

  // Kick off the non-blocking background Wi-Fi auto-connect/auto-reconnect
  // state machine once, here. From this point on it runs entirely on its
  // own via serviceWifiAutoConnect(now) in loop() — no delay(), no
  // while(WiFi.status() != WL_CONNECTED) blocking loop.
  startWifiAutoConnect();

  // ---------- Bluetooth LE ----------
  // Also built into the ESP32-S3. Uses the ESP32 Arduino core's bundled
  // BLE library (BLEDevice/BLEScan/BLEAdvertisedDevice) — no separate
  // library install needed beyond the "esp32" board package itself.
  BLEDevice::init("MORPHIX");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MorphixBleScanCallbacks(), false);
  pBLEScan->setActiveScan(true); // active scan requests get names, not just addresses
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  connData.bleReady = true;
  Serial.println("BLE stack ready");

  // ---------- MORPHIX BLE GATT server (phone bridge) ----------
  // Runs alongside the BLEScan/BLEClient central role above — the ESP32
  // BLE stack supports peripheral + central concurrently. This is what
  // the Android companion app scans for and connects to.
  initMorphixBleServer();

  // ---------- Sensors (BMP280 + MAX30102 + MPU6050, shared I2C bus D4/D5,
  // addresses 0x76 / 0x57 / 0x68) ----------
  // All three are brought up exactly once here; loop() only ever calls
  // the non-blocking update*() functions afterward — see section 6/6b/6c.
  initSensors();

  // Explicit SPI init BEFORE tft.initR(). Without this, Adafruit_ST7735's
  // hardware-SPI constructor calls SPI.begin() with the board's DEFAULT
  // pins on first use — and on the XIAO ESP32-S3, default MISO is GPIO8,
  // which is D9, i.e. PIN_UP. That silently reclaims D9 through the GPIO
  // matrix and overrides the INPUT_PULLUP set above, so every SPI
  // transaction to the display toggles the pin — UP then reads as
  // "pressed" on its own, with or without a wire connected. The ST7735
  // is write-only (no MISO line exists on the wiring), so MISO is passed
  // as -1 here to keep the pin permanently free for the button.
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  pinMode(PIN_UP, INPUT_PULLUP); // re-assert in case SPI.begin() touched it

  tft.initR(INITR_GREENTAB);
  tft.setRotation(1);          // 160x128 landscape — do not change
  tft.setSPISpeed(16000000);   // 16 MHz — safe fast speed for hardware SPI on ST7735
  tft.fillScreen(COL_BG);

  // ===== MORPHIX STORAGE ===== JPEG decoder (image viewer + MJPEG/AVI
  // video player) — configured once here, after the TFT is up since the
  // render callback draws directly to it. setJpgScale() is changed
  // per-image/per-video at open time (see storageDrawJpeg()/
  // storageOpenVideo()), this is just the one-time callback wiring.
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true); // ST7735 expects big-endian 565, tjpgd outputs little-endian
  TJpgDec.setCallback(storageJpegRenderCallback);

  // ---------- Storage (SD card) ----------
  // Must come after the TFT is fully initialized above, since they
  // share the same physical SPI bus. initStorage() itself is a single
  // bounded attempt (SD.begin() + directory creation) — if no card is
  // present it just sets storageError and returns; it never blocks or
  // retries in a loop here. The rest of MorPhix boots normally either way.
  initStorage();

  needsRedraw = true;
}

// ============================================================
// 17. loop()
// ============================================================
unsigned long lastClockTickMs = 0;
unsigned long lastMotionRedrawMs = 0;
unsigned long lastDeviceStatusMs = 0;

void loop() {
  handleButtons();

  unsigned long now = millis();

  // Update real BMP280 readings every 1 second
  static unsigned long lastBMP280Ms = 0;

  if (now - lastBMP280Ms >= 1000) {
    lastBMP280Ms = now;

    updateBMP280();

    // Update only the affected region on Home/Environment — a new
    // BMP280 sample never triggers a full-screen repaint by itself.
    // (If a full redraw is already pending this tick — e.g. a screen
    // transition just happened — skip the partial update: the full
    // renderCurrentScreen() below will paint the fresh value anyway.)
    if (!needsRedraw) {
      if (currentScreen == SCR_HOME) {
        updateHomeTemp();
      } else if (currentScreen == SCR_ENV) {
        updateEnvironmentDynamic();
      }
    }
  }

  // Pump the MAX30102 every loop iteration — non-blocking, it only drains
  // whatever FIFO samples are already available and otherwise returns
  // immediately. Only trigger a redraw when the exposed HR/SpO2 validity
  // or value actually changes, so this doesn't force a full-speed redraw
  // loop on screens that show heart-rate data.
  {
    bool hrValidBefore   = heartRateValid;
    bool spo2ValidBefore = spo2Valid;
    int  hrBefore        = getStableHeartRate();
    int  spo2Before      = getStableSpO2();

    updateMAX30102();

    bool healthChanged = (heartRateValid != hrValidBefore) ||
                          (spo2Valid != spo2ValidBefore) ||
                          (getStableHeartRate() != hrBefore) ||
                          (getStableSpO2() != spo2Before);

    // Same principle as BMP280 above: update only the HR/SpO2 region
    // that actually changed, never the whole screen — and only if a
    // full redraw isn't already pending this tick.
    if (healthChanged && !needsRedraw) {
      if (currentScreen == SCR_HOME) {
        updateHomeHR();
      } else if (currentScreen == SCR_HEALTH) {
        updateHealthDynamic();
      }
    }
  }

  // SENSORS settings page shows live BMP280/MAX30102/connectivity status —
  // tick it once a second so a signal-quality or link-state change is
  // reflected while the user is looking at it. Updates only the
  // per-row status/detail regions that actually changed (see
  // updateSensorsSettingsDynamic()) rather than forcing a full-screen
  // repaint every second.
  static unsigned long lastSensorsSettingsMs = 0;
  if (currentScreen == SCR_SENSORS_SETTINGS && now - lastSensorsSettingsMs >= 1000) {
    lastSensorsSettingsMs = now;
    if (!needsRedraw) updateSensorsSettingsDynamic();
  }

  // Pause the demo-autoplay clock while the user is inside a sub-screen
  // (WiFi, Bluetooth, Motion Data, Settings detail pages, etc.) so
  // autoplay never yanks them back to the top-level carousel mid-interaction.
  if (currentScreen >= SCR_TOP_COUNT) {
    lastScreenChangeMs = now;
  }

  // --- Demo autoplay: cycles every top-level screen automatically ---
  if (DEMO_AUTOPLAY && currentScreen < SCR_TOP_COUNT) {
    if (now - lastScreenChangeMs >= SCREEN_HOLD_MS) {
      goToScreen((currentScreen + 1) % SCR_TOP_COUNT);
    } else if (currentScreen == SCR_SETTINGS &&
               now - lastSubItemMs >= SUBITEM_HOLD_MS) {
      lastSubItemMs = now;
      settingsIndex = (settingsIndex + 1) % SETTINGS_COUNT;
      needsRedraw = true;
    } else if (currentScreen == SCR_NOTIF && notifCount > 0 &&
               now - lastSubItemMs >= SUBITEM_HOLD_MS) {
      lastSubItemMs = now;
      notifIndex = (notifIndex + 1) % notifCount;
      needsRedraw = true;
    }
  }

  // Real clock tick — reads the ESP32 system clock (NTP-synced, or
  // restored from NVS at boot; see updateClockData()) once a second.
  // Never manually increments seconds/minutes/hours itself, so the clock
  // keeps running off the real system time even if Wi-Fi later drops.
  if (now - lastClockTickMs >= 1000) {
    lastClockTickMs = now;
    updateClockData();
    if (currentScreen == SCR_HOME && !needsRedraw) {
      // Only the clock digits change — update just that rectangle so
      // the header/footer/stats never flash or redraw needlessly.
      updateHomeClock();
    }
  }

  // --- Motion / gesture / step / raise-to-wake pipeline (section 6c/14) ---
  // MPU6050 is sampled on its own fixed cadence (~66 Hz, inside the
  // requested 50-100 Hz band) rather than every loop() iteration, so its
  // rate doesn't depend on how fast the rest of loop() happens to run.
  static unsigned long lastMpuSampleMs = 0;
  bool motionWasMoving = motionIsMoving;
  long stepsBefore = stepCount;

  if (now - lastMpuSampleMs >= MPU_SAMPLE_INTERVAL_MS) {
    lastMpuSampleMs = now;

    updateMPU6050();       // serviceMPU6050(): read + filter, non-blocking

    int g = detectGesture(); // processGestures()
    if (g != GESTURE_NONE) handleGesture(g);

    updateStepCounter();    // processSteps()

    if (raiseToWakeEvent()) { // processRaiseToWake()
      strcpy(lastActionLabel, "WAKE (RAISE)");
      lastGesture = GESTURE_NONE; // distinct from a hand gesture; label already set
      if (currentScreen != SCR_HOME) goToScreen(SCR_HOME);
      else needsRedraw = true;
    }
  }

  if (currentScreen == SCR_MOTION_DATA && now - lastMotionRedrawMs >= 250) {
    lastMotionRedrawMs = now;
    // Region-only update (accel/gyro/pitch/roll/motion/gesture), each
    // gated on its own displayed value changing — see
    // updateMotionDataDynamic(). A full drawMotionDataScreen() repaint
    // at this 250 ms cadence would ghost fast-changing digits, since a
    // same-screen redraw never fillScreen()s first.
    if (!needsRedraw) updateMotionDataDynamic();
  }
  if (motionIsMoving != motionWasMoving &&
      (currentScreen == SCR_ACTIVITY || currentScreen == SCR_SENSORS_SETTINGS)) {
    needsRedraw = true;
  }
  if (stepCount != stepsBefore && currentScreen == SCR_ACTIVITY) {
    needsRedraw = true;
  }

  // --- Background Wi-Fi auto-connect/auto-reconnect + NTP sync ---
  // Both are pure millis()-gated state machines: they never call delay()
  // and never attempt a connection/sync every single loop iteration. This
  // is what actually attaches to the phone hotspot in the background,
  // reconnects automatically if it disappears and comes back, and starts
  // NTP synchronization once Wi-Fi is up.
  serviceWifiAutoConnect(now);
  serviceNtpSync(now);

  // --- Real WiFi status sync: runs regardless of which screen is shown,
  // so a drop (or a connection completing) is reflected on CONNECTIVITY /
  // DEVICE STATUS even when the user isn't sitting on the WIFI screen. ---
  static unsigned long lastWifiSyncMs = 0;
  if (now - lastWifiSyncMs >= 1000) {
    lastWifiSyncMs = now;
    bool nowConnected = (WiFi.status() == WL_CONNECTED);
    if (nowConnected != connData.wifiConnected) {
      connData.wifiConnected = nowConnected;
      if (currentScreen == SCR_WIFI || currentScreen == SCR_CONNECTIVITY || currentScreen == SCR_DEVICE_STATUS) {
        needsRedraw = true;
      }
    }
    if (nowConnected) {
      strncpy(connData.wifiSSID, WiFi.SSID().c_str(), sizeof(connData.wifiSSID) - 1);
      connData.wifiSSID[sizeof(connData.wifiSSID) - 1] = 0;
      connData.wifiRSSI = WiFi.RSSI();
    }
  }

  // --- WiFi state machine: scan/connect are real and asynchronous
  // (WiFi.scanNetworks(true) + WiFi.scanComplete(); WiFi.begin() +
  // WiFi.status() polling) so this just polls/times them out. ---
  if (currentScreen == SCR_WIFI) {
    if (wifiScreenState == WIFI_SCR_SCANNING) {
      pollWifiScan();
      if (wifiScreenState == WIFI_SCR_SCANNING && now - wifiStateTimerMs >= WIFI_SCAN_TIMEOUT_MS) {
        WiFi.scanDelete();
        Serial.println("WiFi scan timed out");
        wifiScreenState = WIFI_SCR_STATUS;
        needsRedraw = true;
      }
    } else if (wifiScreenState == WIFI_SCR_CONNECTING) {
      if (WiFi.status() == WL_CONNECTED) {
        connData.wifiConnected = true;
        strncpy(connData.wifiSSID, WiFi.SSID().c_str(), sizeof(connData.wifiSSID) - 1);
        connData.wifiSSID[sizeof(connData.wifiSSID) - 1] = 0;
        connData.wifiRSSI = WiFi.RSSI();
        wifiScreenState = WIFI_SCR_CONNECTED;
        wifiStateTimerMs = now;
        Serial.println("Connection result: SUCCESS");
        needsRedraw = true;
      } else if (now - wifiStateTimerMs >= WIFI_CONNECT_TIMEOUT_MS) {
        WiFi.disconnect(true);
        wifiScreenState = WIFI_SCR_FAILED;
        wifiStateTimerMs = now;
        Serial.println("Connection result: FAILED (timeout)");
        needsRedraw = true;
      }
    } else if (wifiScreenState == WIFI_SCR_CONNECTED && now - wifiStateTimerMs >= WIFI_CONNECTED_HOLD_MS) {
      wifiScreenState = WIFI_SCR_STATUS;
      needsRedraw = true;
    }
  }

  // --- BLE state machine: scanning is real and asynchronous (results
  // stream in live via MorphixBleScanCallbacks::onResult(), completion
  // flagged by bleScanCompleteCB()). Pairing calls the real (briefly
  // blocking) connectBLEDevice() once the short PAIRING... paint delay
  // has elapsed — see BLE_PAIR_MS / connectBLEDevice() for why. ---
  if (currentScreen == SCR_BLUETOOTH) {
    if (bleScreenState == BLE_SCR_SCANNING) {
      if (bleScanDone) {
        pBLEScan->clearResults();
        Serial.print("BLE scan complete. Found ");
        Serial.println(bleDeviceCount);
        bleListIndex = 0; bleListScroll = 0;
        bleScreenState = (bleDeviceCount > 0) ? BLE_SCR_LIST : BLE_SCR_STATUS;
        needsRedraw = true;
      } else if (now - bleScanStartMs >= BLE_SCAN_TIMEOUT_MS) {
        pBLEScan->stop();
        pBLEScan->clearResults();
        Serial.println("BLE scan failed (timeout)");
        bleScreenState = BLE_SCR_STATUS;
        needsRedraw = true;
      }
    } else if (bleScreenState == BLE_SCR_PAIRING && now - bleStateTimerMs >= BLE_PAIR_MS) {
      bool ok = connectBLEDevice(selectedBLEDevice); // real, briefly-blocking BLE connect
      bleScreenState = ok ? BLE_SCR_CONNECTED : BLE_SCR_FAILED;
      bleStateTimerMs = now;
      needsRedraw = true;
    } else if (bleScreenState == BLE_SCR_CONNECTED && now - bleStateTimerMs >= BLE_CONNECTED_HOLD_MS) {
      bleScreenState = BLE_SCR_STATUS;
      needsRedraw = true;
    }
  }

  // --- MORPHIX BLE GATT server (the phone bridge): connData.phoneConnected
  // and phoneState are now set directly by MorphixServerCallbacks
  // onConnect()/onDisconnect() (see the PHONE BLE BRIDGE section) — this
  // block just watches for that edge to trigger a redraw, and drives the
  // reassembly-timeout / periodic status-notify housekeeping that has to
  // happen every loop() without ever blocking it. ---
  static bool lastPhoneConnectedFlag = false;
  if (connData.phoneConnected != lastPhoneConnectedFlag) {
    lastPhoneConnectedFlag = connData.phoneConnected;
    if (currentScreen == SCR_PHONELINK || currentScreen == SCR_CONNECTIVITY) needsRedraw = true;
  }
  serviceMorphixBle(); // reassembly timeout check + periodic STATUS_TX notify, always non-blocking

  if (currentScreen == SCR_PHONELINK) {
    if (phoneState == PHONE_SYNCING && now - phoneStateTimerMs >= PHONE_SYNC_MS) {
      phoneState = PHONE_SYNC_DONE;
      // Real synced time when the phone has sent one via TIME_SYNC;
      // otherwise an honest "--:--:--" — never the mock on-screen clock.
      if (morphixTimeSynced) {
        time_t t = (time_t)currentUnixSeconds();
        struct tm tmVal;
        gmtime_r(&t, &tmVal);
        snprintf(connData.phoneLastSync, sizeof(connData.phoneLastSync), "%02d:%02d:%02d",
                 tmVal.tm_hour, tmVal.tm_min, tmVal.tm_sec);
      } else {
        strcpy(connData.phoneLastSync, "--:--:--");
      }
      phoneStateTimerMs = now;
      needsRedraw = true;
    } else if (phoneState == PHONE_SYNC_DONE && now - phoneStateTimerMs >= PHONE_SYNC_DONE_HOLD_MS) {
      phoneState = PHONE_CONNECTED;
      needsRedraw = true;
    }
  }

  // --- Motion settings: calibration timer. The 1.5s UI hold gives the
  // user a moment to lay the watch flat/still before the real gyro
  // recalibration (calibrateGyro(), ~200ms) actually runs — a bounded,
  // user-initiated exception to the "no blocking in loop()" rule, not
  // something that happens on a timer or automatically. ---
  if (calibrating && now - calibrationStartMs >= CALIBRATION_MS) {
    calibrating = false;
    calibrationDone = true;
    if (mpu6050Ready) calibrateGyro();
    if (currentScreen == SCR_MOTION_SETTINGS) needsRedraw = true;
  }

  // --- Device status: tick the uptime display once a second while visible.
  // Updates only the per-row value regions that changed (see
  // updateDeviceStatusDynamic()) instead of forcing a full-screen repaint
  // every second. ---
  if (currentScreen == SCR_DEVICE_STATUS && now - lastDeviceStatusMs >= 1000) {
    lastDeviceStatusMs = now;
    if (!needsRedraw) updateDeviceStatusDynamic();
  }

  if (needsRedraw) {
    // A full-screen fill only ever happens when we're actually entering
    // a different screen than what's on the glass right now — never
    // just because some sensor value changed while staying put.
    if (currentScreen != lastRenderedScreen) {
      tft.fillScreen(COL_BG);
    }
    renderCurrentScreen();
    lastRenderedScreen = currentScreen;
    needsRedraw = false;

    // The BMP viewer's actual pixel data is painted here, once, right
    // after drawStorageImageScreen() has drawn the header/footer/frame
    // chrome above — painting it inside drawStorageScreen() itself
    // would risk the pixels being wiped by a later fillScreen() if
    // this redraw pass isn't a real screen transition next time.
    if (currentScreen == SCR_STORAGE && storageScreenState == STORAGE_SCR_IMAGE && storagePendingImageDraw) {
      storagePendingImageDraw = false;
      // ===== MORPHIX STORAGE ===== BMP and JPEG now share this same
      // deferred-draw slot — pick the right decoder for this file.
      StorageFileKind imgKind = classifyFile(storageSelName, false);
      if (imgKind == SF_IMAGE_JPEG) {
        storageImageLoaded = storageDrawJpeg(storageSelPath, MARGIN, CONTENT_Y + 2,
                                              SCR_W - 2 * MARGIN, CONTENT_H - 4);
      } else {
        storageImageLoaded = storageDrawBmp(storageSelPath, MARGIN, CONTENT_Y + 2,
                                             SCR_W - 2 * MARGIN, CONTENT_H - 4);
      }
      if (!storageImageLoaded) {
        // Re-paint the content area with the error message now that we
        // know the load failed (drawStorageImageScreen() already ran
        // once above, before we knew the outcome).
        tft.fillRect(0, CONTENT_Y, SCR_W, CONTENT_H, COL_BG);
        drawStorageImageScreen();
      }
      // storageDrawJpeg()/storageDrawBmp() just ended with
      // storageSdEndSession(), whose defensive TFT re-init unconditionally
      // sets lastRenderedScreen = (ScreenID)-1 and needsRedraw = true (see
      // the SD/TFT SPI arbitration comment above storageSdEndSession()).
      // Left as -1, the next loop() pass would treat this as a fresh
      // screen transition and tft.fillScreen() over the pixels we just
      // painted, wiping a successfully loaded image with no error shown.
      // We're still legitimately on this same image screen, so restore
      // lastRenderedScreen here; needsRedraw stays true for one harmless
      // extra chrome-only repaint (header/footer/frame), which never
      // touches the content area.
      lastRenderedScreen = currentScreen;
    }

    // ===== MORPHIX STORAGE ===== deferred video open — same reasoning
    // as the image case above: wait until the VIDEO screen's chrome has
    // actually been painted before touching the SD card.
    if (currentScreen == SCR_STORAGE && storageScreenState == STORAGE_SCR_VIDEO && storagePendingVideoOpen) {
      storagePendingVideoOpen = false;
      if (!storageOpenVideo(storageSelPath)) {
        storageVideoState = VIDEO_ERROR;
        tft.fillRect(0, CONTENT_Y, SCR_W, CONTENT_H, COL_BG);
        drawStorageVideoScreen();
      }
    }
  }

  // ===== MORPHIX STORAGE ===== video frame pump — paced by the AVI's
  // own frame rate (storageAviInfo.frameDelayMs), non-blocking, only
  // active while actually on the VIDEO screen and PLAYING.
  if (currentScreen == SCR_STORAGE && storageScreenState == STORAGE_SCR_VIDEO && storageVideoState == VIDEO_PLAYING) {
    if (now - storageVideoLastFrameMs >= storageAviInfo.frameDelayMs) {
      storageVideoLastFrameMs = now;
      if (!storageVideoNextFrame()) {
        storageVideoState = VIDEO_ENDED;
        needsRedraw = true; // updates the footer label (PLAY/PAUSE -> REPLAY) without touching the video frame area
      }
    }
  }

  // ===== MORPHIX STORAGE ===== pumps the wireless file-transfer web
  // server (no-op whenever it isn't running — see handleStorageWebRequest()).
  handleStorageWebRequest();
}
