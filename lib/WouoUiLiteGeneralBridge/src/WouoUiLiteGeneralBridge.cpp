#include <Arduino.h>

#include <WouoUiLiteGeneralBridge.h>
#include <WouoUiLiteGeneralOfficial.h>

namespace {
bool gStarted = false;
}

namespace WouoUiLiteGeneral {

void begin() {
  if (gStarted) {
    return;
  }
  WouoUiLiteGeneralOfficial::setup();
  gStarted = true;
}

void tick() {
  if (!gStarted) {
    begin();
  }
  WouoUiLiteGeneralOfficial::loop();
}

} // namespace WouoUiLiteGeneral
