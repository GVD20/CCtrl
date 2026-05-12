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

bool xrStatusTick(const char *title, const char *line1, const char *line2,
                  const char *line3) {
  if (!gStarted) {
    begin();
  }
  return WouoUiLiteGeneralOfficial::xrStatusTick(title, line1, line2, line3);
}

} // namespace WouoUiLiteGeneral
