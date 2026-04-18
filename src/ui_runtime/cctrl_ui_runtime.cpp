#include <Arduino.h>

#include <WouoUiLiteGeneralBridge.h>
#include <master_business.h>

namespace {
constexpr uint8_t PIN_LED_ERR = 42;
constexpr uint8_t PIN_LED_STAT = 41;

bool gStatLedOn = false;
unsigned long gLastBlinkMs = 0;
TaskHandle_t gUiTaskHandle = nullptr;
TaskHandle_t gBusinessTaskHandle = nullptr;

void uiTask(void *) {
  WouoUiLiteGeneral::begin();

  for (;;) {
    WouoUiLiteGeneral::tick();

    UiMonitorSnapshot snap{};
    bool hasSnap = MasterBusiness::getMonitorSnapshot(snap);
    digitalWrite(PIN_LED_ERR, (hasSnap && snap.disconnectMode) ? HIGH : LOW);

    const unsigned long nowMs = millis();
    if ((nowMs - gLastBlinkMs) >= 400UL) {
      gLastBlinkMs = nowMs;
      gStatLedOn = !gStatLedOn;
      digitalWrite(PIN_LED_STAT, gStatLedOn ? HIGH : LOW);
    }

    vTaskDelay(1);
  }
}

void businessTask(void *) {
  MasterBusiness::setup();

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
