#include <Arduino.h>

#include "ice_melody_data.h"
#include <WouoUiLiteGeneralBridge.h>
#include <master_business.h>

namespace {
constexpr uint8_t PIN_LED_ERR = 42;
constexpr uint8_t PIN_LED_STAT = 41;
constexpr uint8_t PIN_BUZZER = 2;
constexpr uint8_t BUZZER_PWM_RES_BITS = 8;
constexpr uint8_t BUZZER_PWM_DUTY_BOOT = 51;  // about 30% on 8-bit PWM
constexpr uint8_t BUZZER_PWM_DUTY_ERROR = 51; // about 20% on 8-bit PWM
constexpr uint8_t BUZZER_PWM_DUTY_LINK_OK = 64;
constexpr uint8_t BUZZER_PWM_DUTY_KEY_PRESS = 64;
constexpr uint8_t BUZZER_PWM_DUTY_KEY5 = 16;
constexpr uint16_t BOOT_MELODY_BASE_BPM = 120;
constexpr uint16_t BOOT_MELODY_TARGET_BPM = 195;
constexpr uint32_t BOOT_MELODY_STOP_GUARD_MS = 1500;

constexpr uint8_t BUZZER_PWM_CHANNEL = 0;
constexpr uint8_t KEY5_FLAG_MASK = 0x10;

enum BuzzerSoundKind : uint8_t {
  BUZZER_SOUND_NONE = 0,
  BUZZER_SOUND_BOOT,
  BUZZER_SOUND_ERROR,
  BUZZER_SOUND_LINK_OK,
  BUZZER_SOUND_KEY_PRESS,
  BUZZER_SOUND_KEY5_PRESS,
  BUZZER_SOUND_KEY5_RELEASE,
};

struct BuzzerPlaybackState {
  const BuzzerPwmStep *steps = nullptr;
  size_t count = 0;
  size_t index = 0;
  uint8_t duty = 0;
  BuzzerSoundKind soundKind = BUZZER_SOUND_NONE;
  bool active = false;
  bool ownsPwm = false;
  bool repeat = false;
  bool tempoScale = false;
  uint32_t deadlineMs = 0;
  uint32_t startMs = 0;
  uint32_t guardMs = 0;
};

static constexpr BuzzerPwmStep kErrorBeepPattern[] = {
    {1760, 70},
    {0, 50},
    {1760, 90},
    {0, 1000},
};
static constexpr size_t kErrorBeepPatternCount =
    sizeof(kErrorBeepPattern) / sizeof(kErrorBeepPattern[0]);
static constexpr uint32_t kErrorBeepGuardMs = 1500;
static constexpr BuzzerPwmStep kLinkOkPattern[] = {
    {523, 70}, {0, 25}, {659, 70}, {0, 25}, {784, 95},
};
static constexpr size_t kLinkOkPatternCount =
    sizeof(kLinkOkPattern) / sizeof(kLinkOkPattern[0]);
static constexpr uint32_t kLinkOkGuardMs = 450;
static constexpr BuzzerPwmStep kKeyPressPattern[] = {
    {988, 35},
};
static constexpr size_t kKeyPressPatternCount =
    sizeof(kKeyPressPattern) / sizeof(kKeyPressPattern[0]);
static constexpr uint32_t kKeyPressGuardMs = 80;
static constexpr BuzzerPwmStep kKey5PressPattern[] = {
    {523, 22},
    {0, 10},
    {784, 30},
};
static constexpr size_t kKey5PressPatternCount =
    sizeof(kKey5PressPattern) / sizeof(kKey5PressPattern[0]);
static constexpr uint32_t kKey5PressGuardMs = 110;
static constexpr BuzzerPwmStep kKey5ReleasePattern[] = {
    {784, 22},
    {0, 10},
    {523, 30},
};
static constexpr size_t kKey5ReleasePatternCount =
    sizeof(kKey5ReleasePattern) / sizeof(kKey5ReleasePattern[0]);
static constexpr uint32_t kKey5ReleaseGuardMs = 110;

static uint32_t scaleDurationForTempo(uint32_t durationMs) {
  if (durationMs == 0U || BOOT_MELODY_BASE_BPM == 0U ||
      BOOT_MELODY_TARGET_BPM == 0U) {
    return durationMs;
  }

  uint64_t scaled = (uint64_t)durationMs * (uint64_t)BOOT_MELODY_BASE_BPM +
                    (uint64_t)(BOOT_MELODY_TARGET_BPM / 2U);
  scaled /= (uint64_t)BOOT_MELODY_TARGET_BPM;
  if (scaled == 0U) {
    scaled = 1U;
  }
  return (uint32_t)scaled;
}

bool gBuzzerPwmReady = false;
uint32_t gLastLinkInitSuccessCount = 0;
bool gLastErrorState = false;
uint8_t gLastKeyFlags = 0;
bool gLastKey5Pressed = false;
volatile bool gBootBuzzerStartRequested = false;
BuzzerPlaybackState gBuzzerPlayback;

static void startBootBuzzer();
static void startErrorBuzzer();
static void startLinkOkBuzzer();
static void startKeyPressBuzzer();
static void startKey5PressBuzzer();
static void startKey5ReleaseBuzzer();
static void stopBuzzerPlayback();

static void buzzerMute() {
#if defined(ARDUINO_ARCH_ESP32)
  if (gBuzzerPwmReady) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteTone(PIN_BUZZER, 0);
    ledcWrite(PIN_BUZZER, 0);
#else
    ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
#endif
  }
#endif
  digitalWrite(PIN_BUZZER, LOW);
}

static void buzzerShutdown() {
#if defined(ARDUINO_ARCH_ESP32)
  if (gBuzzerPwmReady) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteTone(PIN_BUZZER, 0);
    ledcWrite(PIN_BUZZER, 0);
    ledcDetach(PIN_BUZZER);
#else
    ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
    ledcDetachPin(PIN_BUZZER);
#endif
  }
#endif
  gBuzzerPwmReady = false;
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
}

static bool buzzerInitPwm() {
  if (gBuzzerPwmReady) {
    return true;
  }

#if defined(ARDUINO_ARCH_ESP32)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  if (ledcAttach(PIN_BUZZER, 1000, BUZZER_PWM_RES_BITS)) {
    ledcWriteTone(PIN_BUZZER, 0);
    ledcWrite(PIN_BUZZER, 0);
    return true;
  }
#else
  ledcSetup(BUZZER_PWM_CHANNEL, 1000, BUZZER_PWM_RES_BITS);
  ledcAttachPin(PIN_BUZZER, BUZZER_PWM_CHANNEL);
  ledcWriteTone(BUZZER_PWM_CHANNEL, 0);
  ledcWrite(BUZZER_PWM_CHANNEL, 0);
  return true;
#endif
#endif
  return false;
}

static void buzzerSetTone(uint16_t freqHz, uint8_t duty) {
  if (!gBuzzerPwmReady || freqHz == 0U) {
    buzzerMute();
    return;
  }

#if defined(ARDUINO_ARCH_ESP32)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteTone(PIN_BUZZER, freqHz);
  ledcWrite(PIN_BUZZER, duty);
#else
  ledcWriteTone(BUZZER_PWM_CHANNEL, freqHz);
  ledcWrite(BUZZER_PWM_CHANNEL, duty);
#endif
#endif
}

static void stopBuzzerPlayback() {
  gBuzzerPlayback.active = false;
  gBuzzerPlayback.steps = nullptr;
  gBuzzerPlayback.count = 0;
  gBuzzerPlayback.index = 0;
  gBuzzerPlayback.duty = 0;
  gBuzzerPlayback.soundKind = BUZZER_SOUND_NONE;
  gBuzzerPlayback.deadlineMs = 0;
  gBuzzerPlayback.startMs = 0;
  gBuzzerPlayback.guardMs = 0;
  gBuzzerPlayback.repeat = false;
  gBuzzerPlayback.tempoScale = false;
  if (gBuzzerPlayback.ownsPwm || gBuzzerPwmReady) {
    buzzerShutdown();
  }
  gBuzzerPlayback.ownsPwm = false;
}

static void startBuzzerPlayback(const BuzzerPwmStep *steps, size_t count,
                                uint8_t duty, uint32_t guardMs, bool tempoScale,
                                bool repeat, BuzzerSoundKind soundKind) {
  stopBuzzerPlayback();
  if (!steps || count == 0U) {
    return;
  }

  gBuzzerPwmReady = buzzerInitPwm();
  if (!gBuzzerPwmReady) {
    return;
  }

  gBuzzerPlayback.steps = steps;
  gBuzzerPlayback.count = count;
  gBuzzerPlayback.index = 0;
  gBuzzerPlayback.duty = duty;
  gBuzzerPlayback.soundKind = soundKind;
  gBuzzerPlayback.active = true;
  gBuzzerPlayback.ownsPwm = true;
  gBuzzerPlayback.startMs = millis();
  gBuzzerPlayback.guardMs = guardMs;
  gBuzzerPlayback.repeat = repeat;
  gBuzzerPlayback.tempoScale = tempoScale;
}

static bool isBootBuzzerActive() {
  return gBuzzerPlayback.active &&
         gBuzzerPlayback.soundKind == BUZZER_SOUND_BOOT;
}

static void requestPromptPlayback(const BuzzerPwmStep *steps, size_t count,
                                  uint8_t duty, uint32_t guardMs, bool repeat,
                                  BuzzerSoundKind soundKind) {
  // Boot melody owns the buzzer until it finishes so power-on audio cannot be
  // cut off by transient prompt events during startup.
  if (isBootBuzzerActive()) {
    return;
  }
  startBuzzerPlayback(steps, count, duty, guardMs, false, repeat, soundKind);
  if (gBuzzerPlayback.active) {
    buzzerSetTone(0, 0);
    gBuzzerPlayback.deadlineMs = gBuzzerPlayback.startMs;
  }
}

static void advanceBuzzerPlayback(uint32_t nowMs, bool errorState) {
  while (gBuzzerPlayback.active) {
    if (gBuzzerPlayback.index >= gBuzzerPlayback.count) {
      if (gBuzzerPlayback.repeat &&
          gBuzzerPlayback.soundKind == BUZZER_SOUND_ERROR && errorState) {
        gBuzzerPlayback.index = 0;
        gBuzzerPlayback.startMs = nowMs;
      } else {
        stopBuzzerPlayback();
        return;
      }
    }

    if (!gBuzzerPlayback.active) {
      return;
    }

    const BuzzerPwmStep &step = gBuzzerPlayback.steps[gBuzzerPlayback.index++];
    buzzerSetTone(step.freq_hz, gBuzzerPlayback.duty);

    if (step.duration_ms > 0U) {
      const uint32_t durationMs = gBuzzerPlayback.tempoScale
                                      ? scaleDurationForTempo(step.duration_ms)
                                      : step.duration_ms;
      gBuzzerPlayback.deadlineMs = nowMs + durationMs;
      return;
    }
  }
}

void updateBootBuzzer() {
  if (gBootBuzzerStartRequested) {
    gBootBuzzerStartRequested = false;
    if (MasterBusiness::getBootSoundEnabled()) {
      startBootBuzzer();
    }
  }

  UiMonitorSnapshot snap{};
  const bool hasSnap = MasterBusiness::getMonitorSnapshot(snap);
  const uint32_t linkInitSuccessCount =
      MasterBusiness::getLinkInitSuccessCount();
  const bool errorState =
      hasSnap && (snap.errFlags != 0U || snap.disconnectMode != 0U);
  const uint8_t keyFlags = hasSnap ? snap.keyFlags : 0;
  const uint8_t keyPressMask = (uint8_t)(keyFlags & (uint8_t)~gLastKeyFlags);
  const bool key5Pressed = hasSnap && (snap.deltaKey != 0U);
  const bool anyKeyPressed =
      ((keyPressMask & (uint8_t)~KEY5_FLAG_MASK) != 0U) ||
      (hasSnap && key5Pressed && !gLastKey5Pressed);

  if (isBootBuzzerActive() && anyKeyPressed) {
    stopBuzzerPlayback();
  }

  if (linkInitSuccessCount != gLastLinkInitSuccessCount) {
    startLinkOkBuzzer();
    gLastLinkInitSuccessCount = linkInitSuccessCount;
  }

  if ((keyPressMask & (uint8_t)~KEY5_FLAG_MASK) != 0U) {
    startKeyPressBuzzer();
  }

  if (hasSnap && key5Pressed != gLastKey5Pressed) {
    if (key5Pressed) {
      startKey5PressBuzzer();
    } else {
      startKey5ReleaseBuzzer();
    }
    gLastKey5Pressed = key5Pressed;
  }
  if (!hasSnap) {
    gLastKeyFlags = 0;
    gLastKey5Pressed = false;
  } else {
    gLastKeyFlags = keyFlags;
  }

  if (!MasterBusiness::getBootSoundEnabled() && isBootBuzzerActive()) {
    stopBuzzerPlayback();
  }

  if (!errorState && gBuzzerPlayback.active &&
      gBuzzerPlayback.soundKind == BUZZER_SOUND_ERROR) {
    stopBuzzerPlayback();
  }

  if (errorState && !gLastErrorState) {
    startErrorBuzzer();
  }
  gLastErrorState = errorState;

  const uint32_t nowMs = millis();
  if (gBuzzerPlayback.active) {
    if ((int32_t)(nowMs - gBuzzerPlayback.startMs) >=
        (int32_t)gBuzzerPlayback.guardMs) {
      stopBuzzerPlayback();
    } else if ((int32_t)(nowMs - gBuzzerPlayback.deadlineMs) >= 0) {
      advanceBuzzerPlayback(nowMs, errorState);
    }
  }

  if (!gBuzzerPlayback.active && errorState && !isBootBuzzerActive()) {
    startErrorBuzzer();
  }
}

void startBootBuzzer() {
  stopBuzzerPlayback();

  if (!MasterBusiness::getBootSoundEnabled()) {
    return;
  }
  startBuzzerPlayback(kIceBootMelody, kIceBootMelodyCount, BUZZER_PWM_DUTY_BOOT,
                      scaleDurationForTempo(kIceBootMelodyTotalMs) +
                          BOOT_MELODY_STOP_GUARD_MS,
                      true, false, BUZZER_SOUND_BOOT);
  if (gBuzzerPlayback.active) {
    advanceBuzzerPlayback(gBuzzerPlayback.startMs, false);
  }
}

void startErrorBuzzer() {
  requestPromptPlayback(kErrorBeepPattern, kErrorBeepPatternCount,
                        BUZZER_PWM_DUTY_ERROR, kErrorBeepGuardMs, true,
                        BUZZER_SOUND_ERROR);
}

void startLinkOkBuzzer() {
  requestPromptPlayback(kLinkOkPattern, kLinkOkPatternCount,
                        BUZZER_PWM_DUTY_LINK_OK, kLinkOkGuardMs, false,
                        BUZZER_SOUND_LINK_OK);
}

void startKeyPressBuzzer() {
  requestPromptPlayback(kKeyPressPattern, kKeyPressPatternCount,
                        BUZZER_PWM_DUTY_KEY_PRESS, kKeyPressGuardMs, false,
                        BUZZER_SOUND_KEY_PRESS);
}

void startKey5PressBuzzer() {
  requestPromptPlayback(kKey5PressPattern, kKey5PressPatternCount,
                        BUZZER_PWM_DUTY_KEY5, kKey5PressGuardMs, false,
                        BUZZER_SOUND_KEY5_PRESS);
}

void startKey5ReleaseBuzzer() {
  requestPromptPlayback(kKey5ReleasePattern, kKey5ReleasePatternCount,
                        BUZZER_PWM_DUTY_KEY5, kKey5ReleaseGuardMs, false,
                        BUZZER_SOUND_KEY5_RELEASE);
}

bool gStatLedOn = false;
unsigned long gLastBlinkMs = 0;
TaskHandle_t gUiTaskHandle = nullptr;
TaskHandle_t gBusinessTaskHandle = nullptr;

static void renderXrStatusUi() {
  XrWebSnapshot xr{};
  MasterBusiness::getXrWebSnapshot(xr);

  char line1[24] = {0};
  char line2[24] = {0};
  char line3[24] = {0};

  if (!xr.hostLinked) {
    snprintf(line1, sizeof(line1), "USB Wait Pose");
    snprintf(line2, sizeof(line2), "Seq %lu", (unsigned long)xr.lastPacketSeq);
    snprintf(line3, sizeof(line3), "OK Exit");
  } else if (!xr.hasPose) {
    snprintf(line1, sizeof(line1), "Host Linked");
    snprintf(line2, sizeof(line2), "Pose warming");
    snprintf(line3, sizeof(line3), "Seq %lu", (unsigned long)xr.lastPacketSeq);
  } else {
    snprintf(line1, sizeof(line1), "Host Ready");
    snprintf(line2, sizeof(line2), "Seq %lu", (unsigned long)xr.lastPacketSeq);
    snprintf(line3, sizeof(line3), "Age %lu ms",
             (unsigned long)xr.lastPacketAgeMs);
  }

  if (xr.restorePending) {
    snprintf(line3, sizeof(line3), "Restoring bus");
  }

  if (WouoUiLiteGeneral::xrStatusTick("XR-UART Mode", line1, line2, line3)) {
    MasterBusiness::setXrWebMode(false);
  }
}

void uiTask(void *) {
  WouoUiLiteGeneral::begin();

  for (;;) {
    updateBootBuzzer();

    UiMonitorSnapshot snap{};
    bool hasSnap = MasterBusiness::getMonitorSnapshot(snap);
    digitalWrite(PIN_LED_ERR, (hasSnap && snap.disconnectMode) ? HIGH : LOW);

    if (MasterBusiness::getXrWebMode())
      renderXrStatusUi();
    else
      WouoUiLiteGeneral::tick();

    const unsigned long nowMs = millis();
    if ((nowMs - gLastBlinkMs) >= 400UL) {
      gLastBlinkMs = nowMs;
      gStatLedOn = !gStatLedOn;
      digitalWrite(PIN_LED_STAT, gStatLedOn ? HIGH : LOW);
    }

    vTaskDelay(MasterBusiness::getXrWebMode() ? 10 : 1);
  }
}

void businessTask(void *) {
  MasterBusiness::setup();
  if (MasterBusiness::getBootSoundEnabled()) {
    gBootBuzzerStartRequested = true;
  }

  for (;;) {
    MasterBusiness::loop();
    taskYIELD();
  }
}
} // namespace

void setup() {
  pinMode(PIN_LED_ERR, OUTPUT);
  pinMode(PIN_LED_STAT, OUTPUT);

  digitalWrite(PIN_LED_ERR, HIGH);
  digitalWrite(PIN_LED_STAT, HIGH);
  delay(400);
  digitalWrite(PIN_LED_ERR, LOW);
  digitalWrite(PIN_LED_STAT, LOW);

  xTaskCreatePinnedToCore(uiTask, "ui_core0", 8192, nullptr, 2, &gUiTaskHandle,
                          0);
  xTaskCreatePinnedToCore(businessTask, "biz_core1", 12288, nullptr, 3,
                          &gBusinessTaskHandle, 1);
}

void loop() { delay(1000); }
