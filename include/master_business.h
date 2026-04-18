#pragma once

#include <stddef.h>
#include <stdint.h>

enum : uint8_t {
  MB_OUTPUT_IF_RS232 = 0,
  MB_OUTPUT_IF_USB = 1,
};

enum : uint8_t {
  MB_POSE_MODE_RELATIVE = 0,
  MB_POSE_MODE_ABSOLUTE = 1,
};

enum : uint8_t {
  MB_ROT_OUT_EULER = 0,
  MB_ROT_OUT_QUATERNION = 1,
};

enum : uint8_t {
  MB_CTRL_STATUS_IDLE = 0,
  MB_CTRL_STATUS_ACTIVE = 1,
  MB_CTRL_STATUS_NODE_DISCONNECT = 2,
};

struct UiMonitorSnapshot {
  uint8_t totalNodes = 0;
  uint8_t expectedEnc = 0;
  uint8_t expectedHnd = 0;
  uint8_t observedEnc = 0;
  uint8_t observedHnd = 0;
  uint8_t errFlags = 0;
  uint8_t controllerStatus = MB_CTRL_STATUS_IDLE;
  uint8_t encStatus[3] = {0xFF, 0xFF, 0xFF};
  uint16_t enc[3] = {0, 0, 0};
  float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float absEuler[3] = {0.0f, 0.0f, 0.0f};
  float deltaEuler[3] = {0.0f, 0.0f, 0.0f};
  float absPos[3] = {0.0f, 0.0f, 0.0f};
  float deltaPos[3] = {0.0f, 0.0f, 0.0f};
  uint8_t keyFlags = 0;
  uint8_t deltaKey = 0;
  int16_t wheelPos = 0;
  uint8_t joyX = 50;
  uint8_t joyY = 50;
  uint8_t outputInterface = MB_OUTPUT_IF_RS232;
  uint8_t poseMode = MB_POSE_MODE_RELATIVE;
  uint8_t disconnectMode = 0;
  uint8_t disconnectNodeKnown = 0;
  uint8_t disconnectNodeId = 0xFF;
  float lossRate10s = 0.0f;
  uint8_t valid = 0;
};

struct JoystickCalibrationData {
  uint8_t valid = 0;
  uint8_t minX = 0;
  uint8_t maxX = 100;
  uint8_t minY = 0;
  uint8_t maxY = 100;
  uint8_t deadzoneX = 2;
  uint8_t deadzoneY = 2;
  uint8_t normMinX = 0;
  uint8_t normMaxX = 100;
  uint8_t normMinY = 0;
  uint8_t normMaxY = 100;
};

struct EncoderCalibrationState {
  uint16_t raw[3] = {0, 0, 0};
  uint16_t offset[3] = {0, 0, 0};
  int8_t dir[3] = {1, 1, 1};
};

struct UiPopupState {
  uint8_t active = 0;
  char text[32] = {0};
};

namespace MasterBusiness {
void setup();
void loop();
bool getMonitorSnapshot(UiMonitorSnapshot &out);
bool getUiPopupState(UiPopupState &out);

void startJoystickCalibration();
void stopJoystickCalibration();
bool commitJoystickCalibration();
bool getJoystickCalibration(JoystickCalibrationData &out);
bool getJoystickCalibrationLive(JoystickCalibrationData &out, uint8_t &curX,
                                uint8_t &curY);

bool setOutputInterface(uint8_t mode);
uint8_t getOutputInterface();

bool setPoseMode(uint8_t mode);
uint8_t getPoseMode();

bool setRotationOutputMode(uint8_t mode);
uint8_t getRotationOutputMode();

bool getEncoderCalibrationState(EncoderCalibrationState &out);
bool calibrateEncoderAxisToDegree(uint8_t axis, uint16_t degree);
bool toggleEncoderDirection(uint8_t axis);
bool saveEncoderCalibration();
bool reloadEncoderCalibration();
} // namespace MasterBusiness
