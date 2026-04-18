#include "../shared/crc.h"
#include "../shared/protocol.h"
#include "mahony_filter.h"
#include "master_business.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>

#include <math.h>
#include <string.h>

// --- UART pins ---
#define PIN_TX 17
#define PIN_RX 18
#define RS232_TX_PIN 15
#define RS232_RX_PIN 16

// --- Local custom keys (KEY1..KEY4) ---
#define KEY1_PIN 35
#define KEY2_PIN 36
#define KEY3_PIN 37
#define KEY4_PIN 38

// --- Units ---
#define ACC_LSB_PER_G 16384.0f
#define GYR_LSB_PER_DPS 1.0f
#define MAG_UT_PER_LSB 0.15f

// Arm geometry (mm)
#define ARM_L0_MM 42.5f
#define ARM_L1_MM 100.0f
#define ARM_L2_MM 100.0f

// --- Error flags ---
#define ERR_CRC 0x01
#define ERR_TIMEOUT 0x02
#define ERR_PARSE 0x04
#define ERR_TOPO 0x08

// --- RoboMaster custom frame ---
#define RM_SOF 0xA5
#define RM_CMD_ID_CUSTOM_CONTROLLER 0x0302
#define RM_DATA_LEN 30

enum ControllerStatusCode : uint8_t {
  CTRL_STATUS_IDLE = MB_CTRL_STATUS_IDLE,
  CTRL_STATUS_ACTIVE = MB_CTRL_STATUS_ACTIVE,
  CTRL_STATUS_NODE_DISCONNECT = MB_CTRL_STATUS_NODE_DISCONNECT,
};

// KEY5..KEY8 are mapped from node-side btnState bit0..bit3
constexpr uint8_t KEY5_MASK = 0x10; // node btn bit0

constexpr uint32_t LOOP_POLL_PERIOD_US = 22222; // 45Hz
constexpr uint32_t OUTPUT_PERIOD_US = 33333;    // 30Hz
constexpr uint32_t LINK_TIMEOUT_MS = 500;

#pragma pack(push, 1)
struct RmFrameHeader {
  uint8_t sof = RM_SOF;
  uint16_t dataLength = RM_DATA_LEN;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
};

// Unified 30-byte custom payload, exactly matching the new requirement.
struct UnifiedPayload30 {
  // [0] high nibble: controller status, low nibble: errFlags
  uint8_t statusErr = 0;
  // [1..2] KEY1..KEY8 flags in low 8 bits, high 8 bits reserved
  uint16_t keyFlags = 0;
  // [3] dedicated delta-valid key state (duplicate of KEY5 trigger)
  uint8_t deltaKey = 0;
  // [4..5] wheel position accumulator
  int16_t wheelPos = 0;
  // [6..7] joystick
  uint8_t joyX = 50;
  uint8_t joyY = 50;
  // [8..13] position vector in float16 (delta or absolute)
  uint16_t posH[3] = {0, 0, 0};
  // [14..21] attitude vector in float16:
  // Euler mode -> [roll, pitch, yaw, 0]
  // Quaternion mode -> [w, x, y, z]
  uint16_t eulerH[4] = {0, 0, 0, 0};
  // [22] mode flags: bit0=USB/RS232, bit1=ABS/REL, bit2=EUL/QUAT
  uint8_t modeFlags = 0;
  // [23..28] raw encoder angles
  uint16_t encRaw[3] = {0, 0, 0};
  // [29] reserved
  uint8_t reserved = 0;
};
#pragma pack(pop)

static_assert(sizeof(UnifiedPayload30) == RM_DATA_LEN,
              "Unified payload must be exactly 30 bytes");

MahonyFilter mahony;
Preferences prefs;

// --- Runtime/state ---
static bool sSystemReady = false;
static bool sTopoError = false;
static uint8_t sLastErrFlags = 0;
static int sTotalNodes = 0;
static int sExpectedEnc = 0;
static int sExpectedHnd = 0;
static uint8_t sObservedEnc = 0;
static uint8_t sObservedHnd = 0;

static uint16_t sEncRaw[3] = {0, 0, 0};
static uint16_t sEncCalRaw[3] = {0, 0, 0};
static uint8_t sEncStatus[3] = {0xFF, 0xFF, 0xFF};

static float sQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float sDeltaQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float sAbsEuler[3] = {0.0f, 0.0f, 0.0f};
static float sDeltaEuler[3] = {0.0f, 0.0f, 0.0f};
static float sAbsPos[3] = {0.0f, 0.0f, 0.0f};
static float sDeltaPos[3] = {0.0f, 0.0f, 0.0f};

static uint8_t sNodeButtons = 0;
static uint8_t sLocalKeys = 0;
static uint8_t sKeyFlags = 0;
static uint8_t sDeltaKey = 0;
static uint8_t sJoyX = 50;
static uint8_t sJoyY = 50;

static JoystickCalibrationData sJoyCalData;
static bool sJoyCalActive = false;
static uint8_t sJoyCalCurX = 50;
static uint8_t sJoyCalCurY = 50;
static uint8_t sJoyCalPeakCenterX = 0;
static uint8_t sJoyCalPeakCenterY = 0;

static uint8_t sWheelSeq = 0;
static bool sHasWheelSeq = false;
static int16_t sWheelPos = 0;

static bool sPrevDeltaPressed = false;
static float sRefQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float sRefPos[3] = {0.0f, 0.0f, 0.0f};

static uint16_t sEncOffsets[3] = {0, 0, 0};
static int8_t sEncDirs[3] = {1, 1, 1};

static uint8_t sOutputInterface = MB_OUTPUT_IF_RS232;
static uint8_t sPoseMode = MB_POSE_MODE_RELATIVE;
static uint8_t sRotOutputMode = MB_ROT_OUT_EULER;

static uint32_t sPacketSeq = 0;
static uint8_t sTxSeq = 0;
static unsigned long sLastAhrsUpdate = 0;
static uint32_t sLastPollUs = 0;
static uint32_t sLastOutputUs = 0;
static bool sHasSample = false;

static bool sDisconnectMode = false;
static bool sDisconnectNodeKnown = false;
static uint8_t sDisconnectNodeId = 0xFF;
static uint32_t sLastValidFrameMs = 0;
static uint32_t sLastInitRetryMs = 0;

static bool sHasLastRxSeq = false;
static uint8_t sLastRxSeq = 0;
static uint32_t sLossWindowPackets = 0;
static uint32_t sLossWindowMissing = 0;
static float sLossRate10s = 0.0f;

static portMUX_TYPE sMonitorSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
static UiMonitorSnapshot sMonitorSnapshot;
static bool sMonitorSnapshotReady = false;

static portMUX_TYPE sUiPopupMux = portMUX_INITIALIZER_UNLOCKED;
static UiPopupState sUiPopupState;
static bool sUiPopupSticky = false;
static uint32_t sUiPopupExpireMs = 0;

static portMUX_TYPE sConfigMux = portMUX_INITIALIZER_UNLOCKED;

static float clampf(float x, float lo, float hi) {
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

static uint8_t absDiffU8(uint8_t a, uint8_t b) {
  return (a > b) ? (uint8_t)(a - b) : (uint8_t)(b - a);
}

static uint16_t encoderRawToCalibratedRaw(int idx, uint16_t raw) {
  int32_t diff = (int32_t)raw - (int32_t)sEncOffsets[idx];
  diff %= 4096;
  if (diff < 0)
    diff += 4096;

  if (sEncDirs[idx] < 0)
    diff = (4096 - diff) & 0x0FFF;

  return (uint16_t)(diff & 0x0FFF);
}

static float calibratedRawToRad(uint16_t calRaw) {
  int32_t signedTick = (calRaw > 2048u) ? ((int32_t)calRaw - 4096) : calRaw;
  return ((float)signedTick / 4096.0f) * (2.0f * PI);
}

static void computeEndEffectorPos(const uint16_t enc[3], float outPos[3]) {
  const float th0 = calibratedRawToRad(enc[0]);
  const float th1 = calibratedRawToRad(enc[1]);
  const float th2 = calibratedRawToRad(enc[2]);
  const float th12 = th1 + th2;

  const float reach =
      ARM_L0_MM + ARM_L1_MM * cosf(th1) + ARM_L2_MM * cosf(th12);
  outPos[0] = cosf(th0) * reach;
  outPos[1] = ARM_L1_MM * sinf(th1) + ARM_L2_MM * sinf(th12);
  outPos[2] = -sinf(th0) * reach;
}

static void quatConjugate(const float q[4], float out[4]) {
  out[0] = q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = -q[3];
}

static void quatMultiply(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void normalizeQuat(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n <= 1e-9f) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }
  q[0] /= n;
  q[1] /= n;
  q[2] /= n;
  q[3] /= n;
}

static void quatToEulerDeg(const float q[4], float &rollDeg, float &pitchDeg,
                           float &yawDeg) {
  const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  float roll = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (qw * qy - qz * qx);
  sinp = clampf(sinp, -1.0f, 1.0f);
  float pitch = asinf(sinp);

  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  float yaw = atan2f(siny_cosp, cosy_cosp);

  constexpr float RAD2DEG = 57.295779513082320876f;
  rollDeg = roll * RAD2DEG;
  pitchDeg = pitch * RAD2DEG;
  yawDeg = yaw * RAD2DEG;
}

// IEEE754 float32 -> float16
static uint16_t floatToHalf(float value) {
  uint32_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));

  const uint32_t sign = (bits >> 16) & 0x8000u;
  uint32_t exp = (bits >> 23) & 0xFFu;
  uint32_t mant = bits & 0x7FFFFFu;

  if (exp == 0xFFu) {
    // Inf/NaN
    if (mant == 0)
      return (uint16_t)(sign | 0x7C00u);
    return (uint16_t)(sign | 0x7E00u);
  }

  int32_t halfExp = (int32_t)exp - 127 + 15;
  if (halfExp >= 31)
    return (uint16_t)(sign | 0x7C00u); // overflow => Inf

  if (halfExp <= 0) {
    if (halfExp < -10)
      return (uint16_t)sign; // underflow => 0

    mant |= 0x800000u;
    uint32_t shift = (uint32_t)(14 - halfExp);
    uint32_t halfMant = mant >> shift;
    // Round to nearest
    if ((mant >> (shift - 1)) & 1u)
      halfMant++;
    return (uint16_t)(sign | (halfMant & 0x03FFu));
  }

  uint16_t half =
      (uint16_t)(sign | ((uint16_t)halfExp << 10) | (uint16_t)(mant >> 13));
  if (mant & 0x00001000u)
    half++;
  return half;
}

static void setUiPopup(const char *text, bool sticky, uint32_t durationMs) {
  portENTER_CRITICAL(&sUiPopupMux);
  sUiPopupState.active = 1;
  snprintf(sUiPopupState.text, sizeof(sUiPopupState.text), "%s", text);
  sUiPopupSticky = sticky;
  sUiPopupExpireMs = sticky ? 0 : (millis() + durationMs);
  portEXIT_CRITICAL(&sUiPopupMux);
}

static void updateUiPopupState() {
  portENTER_CRITICAL(&sUiPopupMux);
  if (sUiPopupState.active && !sUiPopupSticky &&
      (long)(millis() - sUiPopupExpireMs) >= 0) {
    sUiPopupState.active = 0;
    sUiPopupState.text[0] = '\0';
  }
  portEXIT_CRITICAL(&sUiPopupMux);
}

static uint8_t getOutputInterfaceUnsafe() {
  return (sOutputInterface == MB_OUTPUT_IF_USB) ? MB_OUTPUT_IF_USB
                                                : MB_OUTPUT_IF_RS232;
}

static uint8_t getPoseModeUnsafe() {
  return (sPoseMode == MB_POSE_MODE_ABSOLUTE) ? MB_POSE_MODE_ABSOLUTE
                                              : MB_POSE_MODE_RELATIVE;
}

static uint8_t getRotationOutputModeUnsafe() {
  return (sRotOutputMode == MB_ROT_OUT_QUATERNION) ? MB_ROT_OUT_QUATERNION
                                                   : MB_ROT_OUT_EULER;
}

static uint8_t composeStatusErr() {
  uint8_t status = CTRL_STATUS_IDLE;
  if (sDisconnectMode)
    status = CTRL_STATUS_NODE_DISCONNECT;
  else if (sSystemReady && sHasSample)
    status = CTRL_STATUS_ACTIVE;

  uint8_t err = (uint8_t)(sLastErrFlags & 0x0F);
  if (sTopoError)
    err |= ERR_TOPO;

  return (uint8_t)(((status & 0x0F) << 4) | (err & 0x0F));
}

static uint8_t scanLocalKeys() {
  uint8_t keys = 0;
  if (digitalRead(KEY1_PIN) == LOW)
    keys |= 0x01;
  if (digitalRead(KEY2_PIN) == LOW)
    keys |= 0x02;
  if (digitalRead(KEY3_PIN) == LOW)
    keys |= 0x04;
  if (digitalRead(KEY4_PIN) == LOW)
    keys |= 0x08;
  return keys;
}

static void updateCombinedKeyFlags() {
  sLocalKeys = scanLocalKeys();
  sKeyFlags = (uint8_t)((sLocalKeys & 0x0F) | ((sNodeButtons & 0x0F) << 4));
  sDeltaKey = (sKeyFlags & KEY5_MASK) ? 1 : 0;
}

static void loadJoystickCalibration() {
  prefs.begin("joy_cal", true);
  sJoyCalData.valid = prefs.getUChar("valid", 0);
  sJoyCalData.minX = prefs.getUChar("minx", 0);
  sJoyCalData.maxX = prefs.getUChar("maxx", 100);
  sJoyCalData.minY = prefs.getUChar("miny", 0);
  sJoyCalData.maxY = prefs.getUChar("maxy", 100);
  sJoyCalData.deadzoneX = prefs.getUChar("dzx", 2);
  sJoyCalData.deadzoneY = prefs.getUChar("dzy", 2);
  sJoyCalData.normMinX = prefs.getUChar("nmnx", 0);
  sJoyCalData.normMaxX = prefs.getUChar("nmxx", 100);
  sJoyCalData.normMinY = prefs.getUChar("nmny", 0);
  sJoyCalData.normMaxY = prefs.getUChar("nmxy", 100);
  prefs.end();

  if (!sJoyCalData.valid) {
    sJoyCalData.minX = 0;
    sJoyCalData.maxX = 100;
    sJoyCalData.minY = 0;
    sJoyCalData.maxY = 100;
    sJoyCalData.deadzoneX = 2;
    sJoyCalData.deadzoneY = 2;
    sJoyCalData.normMinX = 0;
    sJoyCalData.normMaxX = 100;
    sJoyCalData.normMinY = 0;
    sJoyCalData.normMaxY = 100;
  }
}

static void saveJoystickCalibration() {
  prefs.begin("joy_cal", false);
  prefs.putUChar("valid", sJoyCalData.valid);
  prefs.putUChar("minx", sJoyCalData.minX);
  prefs.putUChar("maxx", sJoyCalData.maxX);
  prefs.putUChar("miny", sJoyCalData.minY);
  prefs.putUChar("maxy", sJoyCalData.maxY);
  prefs.putUChar("dzx", sJoyCalData.deadzoneX);
  prefs.putUChar("dzy", sJoyCalData.deadzoneY);
  prefs.putUChar("nmnx", sJoyCalData.normMinX);
  prefs.putUChar("nmxx", sJoyCalData.normMaxX);
  prefs.putUChar("nmny", sJoyCalData.normMinY);
  prefs.putUChar("nmxy", sJoyCalData.normMaxY);
  prefs.end();
}

static void updateJoystickCalibrationSample() {
  if (!sJoyCalActive)
    return;

  const uint8_t x = sJoyCalCurX;
  const uint8_t y = sJoyCalCurY;

  if (x < sJoyCalData.minX)
    sJoyCalData.minX = x;
  if (x > sJoyCalData.maxX)
    sJoyCalData.maxX = x;
  if (y < sJoyCalData.minY)
    sJoyCalData.minY = y;
  if (y > sJoyCalData.maxY)
    sJoyCalData.maxY = y;

  if (x >= 40 && x <= 60) {
    uint8_t d = absDiffU8(x, 50);
    if (d > sJoyCalPeakCenterX)
      sJoyCalPeakCenterX = d;
  }
  if (y >= 40 && y <= 60) {
    uint8_t d = absDiffU8(y, 50);
    if (d > sJoyCalPeakCenterY)
      sJoyCalPeakCenterY = d;
  }
}

static void loadEncoderCalibration() {
  prefs.begin("enc_cal", true);
  sEncOffsets[0] = (uint16_t)prefs.getUShort("off0", 0);
  sEncOffsets[1] = (uint16_t)prefs.getUShort("off1", 0);
  sEncOffsets[2] = (uint16_t)prefs.getUShort("off2", 0);
  sEncDirs[0] = (int8_t)prefs.getChar("dir0", 1);
  sEncDirs[1] = (int8_t)prefs.getChar("dir1", 1);
  sEncDirs[2] = (int8_t)prefs.getChar("dir2", 1);
  prefs.end();

  for (int i = 0; i < 3; i++) {
    sEncOffsets[i] &= 0x0FFF;
    sEncDirs[i] = (sEncDirs[i] >= 0) ? 1 : -1;
  }
}

static void saveEncoderCalibrationInternal() {
  prefs.begin("enc_cal", false);
  prefs.putUShort("off0", sEncOffsets[0]);
  prefs.putUShort("off1", sEncOffsets[1]);
  prefs.putUShort("off2", sEncOffsets[2]);
  prefs.putChar("dir0", sEncDirs[0]);
  prefs.putChar("dir1", sEncDirs[1]);
  prefs.putChar("dir2", sEncDirs[2]);
  prefs.end();
}

static void loadOutputConfig() {
  prefs.begin("sys_cfg", true);
  uint8_t outIf = prefs.getUChar("out_if", MB_OUTPUT_IF_RS232);
  uint8_t pose = prefs.getUChar("pose_mode", MB_POSE_MODE_RELATIVE);
  uint8_t rot = prefs.getUChar("rot_fmt", MB_ROT_OUT_EULER);
  prefs.end();

  portENTER_CRITICAL(&sConfigMux);
  sOutputInterface =
      (outIf == MB_OUTPUT_IF_USB) ? MB_OUTPUT_IF_USB : MB_OUTPUT_IF_RS232;
  sPoseMode = (pose == MB_POSE_MODE_ABSOLUTE) ? MB_POSE_MODE_ABSOLUTE
                                              : MB_POSE_MODE_RELATIVE;
  sRotOutputMode =
      (rot == MB_ROT_OUT_QUATERNION) ? MB_ROT_OUT_QUATERNION : MB_ROT_OUT_EULER;
  portEXIT_CRITICAL(&sConfigMux);
}

static void saveOutputConfig() {
  uint8_t outIf = MB_OUTPUT_IF_RS232;
  uint8_t pose = MB_POSE_MODE_RELATIVE;
  uint8_t rot = MB_ROT_OUT_EULER;
  portENTER_CRITICAL(&sConfigMux);
  outIf = getOutputInterfaceUnsafe();
  pose = getPoseModeUnsafe();
  rot = getRotationOutputModeUnsafe();
  portEXIT_CRITICAL(&sConfigMux);

  prefs.begin("sys_cfg", false);
  prefs.putUChar("out_if", outIf);
  prefs.putUChar("pose_mode", pose);
  prefs.putUChar("rot_fmt", rot);
  prefs.end();
}

static void fillUnifiedPayload(UnifiedPayload30 &payload) {
  memset(&payload, 0, sizeof(payload));

  uint8_t outIf = MB_OUTPUT_IF_RS232;
  uint8_t pose = MB_POSE_MODE_RELATIVE;
  uint8_t rot = MB_ROT_OUT_EULER;
  portENTER_CRITICAL(&sConfigMux);
  outIf = getOutputInterfaceUnsafe();
  pose = getPoseModeUnsafe();
  rot = getRotationOutputModeUnsafe();
  portEXIT_CRITICAL(&sConfigMux);

  payload.statusErr = composeStatusErr();
  payload.keyFlags = (uint16_t)sKeyFlags;
  payload.deltaKey = sDeltaKey;
  payload.wheelPos = sWheelPos;
  payload.joyX = sJoyX;
  payload.joyY = sJoyY;

  const float *posSrc = (pose == MB_POSE_MODE_ABSOLUTE) ? sAbsPos : sDeltaPos;
  const float *eulerSrc =
      (pose == MB_POSE_MODE_ABSOLUTE) ? sAbsEuler : sDeltaEuler;

  payload.posH[0] = floatToHalf(posSrc[0]);
  payload.posH[1] = floatToHalf(posSrc[1]);
  payload.posH[2] = floatToHalf(posSrc[2]);

  if (rot == MB_ROT_OUT_QUATERNION) {
    const float *quatSrc = (pose == MB_POSE_MODE_ABSOLUTE) ? sQuat : sDeltaQuat;
    payload.eulerH[0] = floatToHalf(quatSrc[0]);
    payload.eulerH[1] = floatToHalf(quatSrc[1]);
    payload.eulerH[2] = floatToHalf(quatSrc[2]);
    payload.eulerH[3] = floatToHalf(quatSrc[3]);
  } else {
    payload.eulerH[0] = floatToHalf(eulerSrc[0]);
    payload.eulerH[1] = floatToHalf(eulerSrc[1]);
    payload.eulerH[2] = floatToHalf(eulerSrc[2]);
    payload.eulerH[3] = 0;
  }

  payload.modeFlags = 0;
  if (outIf == MB_OUTPUT_IF_USB)
    payload.modeFlags |= 0x01;
  if (pose == MB_POSE_MODE_ABSOLUTE)
    payload.modeFlags |= 0x02;
  if (rot == MB_ROT_OUT_QUATERNION)
    payload.modeFlags |= 0x04;

  payload.encRaw[0] = sEncCalRaw[0];
  payload.encRaw[1] = sEncCalRaw[1];
  payload.encRaw[2] = sEncCalRaw[2];
  payload.reserved = 0;
}

static void sendUnifiedFrame() {
  UnifiedPayload30 payload;
  fillUnifiedPayload(payload);

  RmFrameHeader header;
  header.seq = sTxSeq++;

  uint8_t headerRaw[4] = {header.sof, (uint8_t)(header.dataLength & 0xFF),
                          (uint8_t)((header.dataLength >> 8) & 0xFF),
                          header.seq};
  header.crc8 = rm_crc8(headerRaw, sizeof(headerRaw));

  uint8_t frame[sizeof(RmFrameHeader) + 2 + sizeof(UnifiedPayload30) + 2] = {0};
  size_t off = 0;

  frame[off++] = header.sof;
  frame[off++] = (uint8_t)(header.dataLength & 0xFF);
  frame[off++] = (uint8_t)((header.dataLength >> 8) & 0xFF);
  frame[off++] = header.seq;
  frame[off++] = header.crc8;

  frame[off++] = (uint8_t)(RM_CMD_ID_CUSTOM_CONTROLLER & 0xFF);
  frame[off++] = (uint8_t)((RM_CMD_ID_CUSTOM_CONTROLLER >> 8) & 0xFF);

  memcpy(frame + off, &payload, sizeof(payload));
  off += sizeof(payload);

  uint16_t crc16 = rm_crc16(frame, off);
  frame[off++] = (uint8_t)(crc16 & 0xFF);
  frame[off++] = (uint8_t)((crc16 >> 8) & 0xFF);

  uint8_t outIf = MB_OUTPUT_IF_RS232;
  portENTER_CRITICAL(&sConfigMux);
  outIf = getOutputInterfaceUnsafe();
  portEXIT_CRITICAL(&sConfigMux);

  if (outIf == MB_OUTPUT_IF_USB)
    Serial.write(frame, off);
  else
    Serial2.write(frame, off);
}

static void publishMonitorSnapshot() {
  UiMonitorSnapshot snap;
  snap.totalNodes = (uint8_t)sTotalNodes;
  snap.expectedEnc = (uint8_t)sExpectedEnc;
  snap.expectedHnd = (uint8_t)sExpectedHnd;
  snap.observedEnc = sObservedEnc;
  snap.observedHnd = sObservedHnd;
  snap.errFlags = (uint8_t)(sLastErrFlags | (sTopoError ? ERR_TOPO : 0));
  snap.controllerStatus = (uint8_t)(composeStatusErr() >> 4);

  snap.encStatus[0] = sEncStatus[0];
  snap.encStatus[1] = sEncStatus[1];
  snap.encStatus[2] = sEncStatus[2];
  snap.enc[0] = sEncCalRaw[0];
  snap.enc[1] = sEncCalRaw[1];
  snap.enc[2] = sEncCalRaw[2];

  snap.quat[0] = sQuat[0];
  snap.quat[1] = sQuat[1];
  snap.quat[2] = sQuat[2];
  snap.quat[3] = sQuat[3];

  snap.absEuler[0] = sAbsEuler[0];
  snap.absEuler[1] = sAbsEuler[1];
  snap.absEuler[2] = sAbsEuler[2];
  snap.deltaEuler[0] = sDeltaEuler[0];
  snap.deltaEuler[1] = sDeltaEuler[1];
  snap.deltaEuler[2] = sDeltaEuler[2];

  snap.absPos[0] = sAbsPos[0];
  snap.absPos[1] = sAbsPos[1];
  snap.absPos[2] = sAbsPos[2];
  snap.deltaPos[0] = sDeltaPos[0];
  snap.deltaPos[1] = sDeltaPos[1];
  snap.deltaPos[2] = sDeltaPos[2];

  snap.keyFlags = sKeyFlags;
  snap.deltaKey = sDeltaKey;
  snap.wheelPos = sWheelPos;
  snap.joyX = sJoyX;
  snap.joyY = sJoyY;

  portENTER_CRITICAL(&sConfigMux);
  snap.outputInterface = getOutputInterfaceUnsafe();
  snap.poseMode = getPoseModeUnsafe();
  portEXIT_CRITICAL(&sConfigMux);

  snap.disconnectMode = sDisconnectMode ? 1 : 0;
  snap.disconnectNodeKnown = sDisconnectNodeKnown ? 1 : 0;
  snap.disconnectNodeId = sDisconnectNodeId;
  snap.lossRate10s = sLossRate10s;
  snap.valid = sSystemReady ? 1 : 0;

  portENTER_CRITICAL(&sMonitorSnapshotMux);
  sMonitorSnapshot = snap;
  sMonitorSnapshotReady = true;
  portEXIT_CRITICAL(&sMonitorSnapshotMux);
}

static void enterDisconnectMode(bool nodeKnown, uint8_t nodeId) {
  sDisconnectMode = true;
  sDisconnectNodeKnown = nodeKnown;
  sDisconnectNodeId = nodeId;
  sSystemReady = false;

  if (nodeKnown) {
    char buf[32];
    snprintf(buf, sizeof(buf), "ERR: Check Conn %u", (unsigned)nodeId);
    setUiPopup(buf, true, 0);
  } else {
    setUiPopup("ERR: Check Conn", true, 0);
  }
}

static void exitDisconnectMode() {
  const bool wasDisconnect = sDisconnectMode;
  sDisconnectMode = false;
  sDisconnectNodeKnown = false;
  sDisconnectNodeId = 0xFF;
  sLastInitRetryMs = millis();
  sLastValidFrameMs = millis();
  if (wasDisconnect)
    setUiPopup("Conn restored", false, 1500);
}

static bool tryInitHandshake(uint32_t timeoutMs) {
  uint8_t addrPayload[] = {CMD_ADDR_RESET, 0, 0, 0};
  uint8_t crc = calcCRC8(addrPayload, 4);

  while (Serial1.available())
    Serial1.read();

  Serial1.write(addrPayload, 4);
  Serial1.write(crc);

  uint8_t buf[5] = {0};
  int idx = 0;
  uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < timeoutMs) {
    if (!Serial1.available())
      continue;
    uint8_t b = (uint8_t)Serial1.read();
    if (idx == 0 && b != CMD_ADDR_RESET)
      continue;
    if (idx < (int)sizeof(buf))
      buf[idx++] = b;
    if (idx >= (int)sizeof(buf))
      break;
  }

  if (idx < (int)sizeof(buf))
    return false;
  if (calcCRC8(buf, 4) != buf[4])
    return false;

  sTotalNodes = buf[1];
  sExpectedEnc = buf[2];
  sExpectedHnd = buf[3];
  sTopoError = (sTotalNodes == 0 || sExpectedHnd == 0);
  sSystemReady = !sTopoError;
  sLastErrFlags = sTopoError ? ERR_TOPO : 0;

  if (sSystemReady) {
    sLastValidFrameMs = millis();
    sHasLastRxSeq = false;
  }
  return sSystemReady;
}

static void resetSampleCache() {
  sObservedEnc = 0;
  sObservedHnd = 0;
  sNodeButtons = 0;
}

static void master_business_setup() {
  setCpuFrequencyMhz(240);
  WiFi.mode(WIFI_OFF);
  btStop();

  pinMode(KEY1_PIN, INPUT_PULLUP);
  pinMode(KEY2_PIN, INPUT_PULLUP);
  pinMode(KEY3_PIN, INPUT_PULLUP);
  pinMode(KEY4_PIN, INPUT_PULLUP);

  Serial.begin(2000000);
  Serial1.begin(250000, SERIAL_8N1, PIN_RX, PIN_TX);
  Serial2.begin(115200, SERIAL_8N1, RS232_RX_PIN, RS232_TX_PIN);

  mahony.reset();
  mahony.setGains(2.0f, 0.0f);

  loadJoystickCalibration();
  loadEncoderCalibration();
  loadOutputConfig();

  resetSampleCache();
  updateCombinedKeyFlags();

  sLastPollUs = micros();
  sLastOutputUs = sLastPollUs;
  sLastValidFrameMs = millis();
  sLastInitRetryMs = millis();

  delay(500);
  if (tryInitHandshake(1000))
    exitDisconnectMode();
  else
    enterDisconnectMode(false, 0xFF);

  publishMonitorSnapshot();
}

static void processValidBusPacket(uint8_t *rxBuf, int rxIdx) {
  const uint8_t rxSeq = rxBuf[1];
  if (sHasLastRxSeq) {
    uint8_t seqDiff = (uint8_t)(rxSeq - sLastRxSeq);
    if (seqDiff > 1)
      sLossWindowMissing += (uint32_t)(seqDiff - 1);
  }
  sLastRxSeq = rxSeq;
  sHasLastRxSeq = true;
  sLossWindowPackets++;
  if (sLossWindowPackets >= 450U) {
    uint32_t denom = sLossWindowPackets + sLossWindowMissing;
    sLossRate10s = (denom > 0U)
                       ? ((float)sLossWindowMissing * 100.0f / (float)denom)
                       : 0.0f;
    sLossWindowPackets = 0;
    sLossWindowMissing = 0;
  }

  uint8_t *ptr = &rxBuf[3];
  uint8_t remain = rxBuf[2];
  uint8_t encIdx = 0;
  uint8_t hndCount = 0;

  int16_t accRaw[3] = {0, 0, 0};
  int16_t gyrRaw[3] = {0, 0, 0};
  int16_t magRaw[3] = {0, 0, 0};
  uint8_t nodeButtons = 0;
  uint8_t wheelSeq = sWheelSeq;
  uint8_t joyX = sJoyX;
  uint8_t joyY = sJoyY;

  for (int i = 0; i < 3; ++i) {
    sEncStatus[i] = 0xFF;
    sEncRaw[i] = 0;
    sEncCalRaw[i] = 0;
  }

  while (remain > 0) {
    uint8_t type = ptr[0];
    if (type == NODE_TYPE_ENCODER) {
      if (remain < sizeof(EncoderPayload)) {
        sLastErrFlags = ERR_PARSE;
        break;
      }
      EncoderPayload *p = (EncoderPayload *)ptr;
      if (encIdx < 3) {
        sEncRaw[encIdx] = p->rawAngle;
        sEncStatus[encIdx] = p->status;
        encIdx++;
      }
      ptr += sizeof(EncoderPayload);
      remain -= sizeof(EncoderPayload);
    } else if (type == NODE_TYPE_HANDLE) {
      if (remain < sizeof(HandlePayload)) {
        sLastErrFlags = ERR_PARSE;
        break;
      }
      HandlePayload *p = (HandlePayload *)ptr;
      accRaw[0] = p->accX;
      accRaw[1] = p->accY;
      accRaw[2] = p->accZ;
      gyrRaw[0] = p->gyrX;
      gyrRaw[1] = p->gyrY;
      gyrRaw[2] = p->gyrZ;
      magRaw[0] = p->magX;
      magRaw[1] = p->magY;
      magRaw[2] = p->magZ;
      nodeButtons = p->btnState;
      wheelSeq = p->wheelSeq;
      joyX = p->joyX;
      joyY = p->joyY;
      hndCount++;
      ptr += sizeof(HandlePayload);
      remain -= sizeof(HandlePayload);
    } else {
      sLastErrFlags = ERR_PARSE;
      break;
    }
  }

  sObservedEnc = encIdx;
  sObservedHnd = hndCount;

  for (int i = 0; i < 3; ++i) {
    sEncCalRaw[i] = encoderRawToCalibratedRaw(i, sEncRaw[i]);
  }

  sNodeButtons = nodeButtons;
  sJoyX = joyX;
  sJoyY = joyY;
  sJoyCalCurX = joyX;
  sJoyCalCurY = joyY;
  updateJoystickCalibrationSample();
  updateCombinedKeyFlags();

  if (hndCount > 0) {
    if (sHasWheelSeq) {
      int8_t diff = (int8_t)(wheelSeq - sWheelSeq);
      int32_t next = (int32_t)sWheelPos + (int32_t)diff;
      if (next > 32767)
        next = 32767;
      if (next < -32768)
        next = -32768;
      sWheelPos = (int16_t)next;
    }
    sWheelSeq = wheelSeq;
    sHasWheelSeq = true;

    float raw_ax = accRaw[0] / ACC_LSB_PER_G;
    float raw_ay = accRaw[1] / ACC_LSB_PER_G;
    float raw_az = accRaw[2] / ACC_LSB_PER_G;

    float raw_gx = gyrRaw[0] / GYR_LSB_PER_DPS;
    float raw_gy = gyrRaw[1] / GYR_LSB_PER_DPS;
    float raw_gz = gyrRaw[2] / GYR_LSB_PER_DPS;

    float raw_mx = magRaw[0] * MAG_UT_PER_LSB;
    float raw_my = magRaw[1] * MAG_UT_PER_LSB;
    float raw_mz = magRaw[2] * MAG_UT_PER_LSB;

    unsigned long nowMicros = micros();
    float dt = (sLastAhrsUpdate == 0)
                   ? 0.01f
                   : (nowMicros - sLastAhrsUpdate) / 1000000.0f;
    sLastAhrsUpdate = nowMicros;
    if (dt > 0.1f)
      dt = 0.01f;
    if (dt < 0.001f)
      dt = 0.001f;

    mahony.update(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz, raw_mx,
                  raw_my, raw_mz, dt, true, false);
    mahony.getQuaternion(&sQuat[0], &sQuat[1], &sQuat[2], &sQuat[3]);
    normalizeQuat(sQuat);

    quatToEulerDeg(sQuat, sAbsEuler[0], sAbsEuler[1], sAbsEuler[2]);
    computeEndEffectorPos(sEncCalRaw, sAbsPos);

    bool deltaPressed = (sDeltaKey != 0);
    if (deltaPressed && !sPrevDeltaPressed) {
      sRefQuat[0] = sQuat[0];
      sRefQuat[1] = sQuat[1];
      sRefQuat[2] = sQuat[2];
      sRefQuat[3] = sQuat[3];
      normalizeQuat(sRefQuat);

      sRefPos[0] = sAbsPos[0];
      sRefPos[1] = sAbsPos[1];
      sRefPos[2] = sAbsPos[2];
    }

    if (deltaPressed) {
      float qCur[4] = {sQuat[0], sQuat[1], sQuat[2], sQuat[3]};
      float qRefConj[4];
      float qDelta[4];
      quatConjugate(sRefQuat, qRefConj);
      quatMultiply(qRefConj, qCur, qDelta);
      normalizeQuat(qDelta);
      sDeltaQuat[0] = qDelta[0];
      sDeltaQuat[1] = qDelta[1];
      sDeltaQuat[2] = qDelta[2];
      sDeltaQuat[3] = qDelta[3];
      quatToEulerDeg(qDelta, sDeltaEuler[0], sDeltaEuler[1], sDeltaEuler[2]);

      sDeltaPos[0] = sAbsPos[0] - sRefPos[0];
      sDeltaPos[1] = sAbsPos[1] - sRefPos[1];
      sDeltaPos[2] = sAbsPos[2] - sRefPos[2];
    } else {
      sDeltaQuat[0] = 1.0f;
      sDeltaQuat[1] = 0.0f;
      sDeltaQuat[2] = 0.0f;
      sDeltaQuat[3] = 0.0f;
      sDeltaEuler[0] = 0.0f;
      sDeltaEuler[1] = 0.0f;
      sDeltaEuler[2] = 0.0f;
      sDeltaPos[0] = 0.0f;
      sDeltaPos[1] = 0.0f;
      sDeltaPos[2] = 0.0f;
    }
    sPrevDeltaPressed = deltaPressed;
  }

  sHasSample = true;
}

static void master_business_loop() {
  updateCombinedKeyFlags();
  updateUiPopupState();

  if (sDisconnectMode || !sSystemReady) {
    if ((uint32_t)(millis() - sLastInitRetryMs) >= 500U) {
      sLastInitRetryMs = millis();
      if (tryInitHandshake(80))
        exitDisconnectMode();
    }
    publishMonitorSnapshot();
    delay(2);
    return;
  }

  uint32_t nowUs = micros();
  bool doPoll = false;
  if ((uint32_t)(nowUs - sLastPollUs) >= LOOP_POLL_PERIOD_US) {
    sLastPollUs += LOOP_POLL_PERIOD_US;
    if ((uint32_t)(nowUs - sLastPollUs) >= LOOP_POLL_PERIOD_US)
      sLastPollUs = nowUs;
    doPoll = true;
  }

  if (doPoll) {
    sPacketSeq++;
    uint8_t txBuf[4];
    txBuf[0] = CMD_DATA_LOOP;
    txBuf[1] = (uint8_t)(sPacketSeq & 0xFF);
    txBuf[2] = 0;
    txBuf[3] = calcCRC8(txBuf, 3);

    Serial1.write(txBuf, 4);

    uint8_t rxBuf[128] = {0};
    int rxIdx = 0;
    unsigned long rxStart = micros();
    bool pktComplete = false;
    size_t targetLen = 0;
    uint8_t firstCmd = 0;

    while (micros() - rxStart < 8000) {
      if (!Serial1.available())
        continue;

      uint8_t b = (uint8_t)Serial1.read();
      if (rxIdx == 0) {
        if (b != CMD_DATA_LOOP && b != CMD_DISCONNECT_REPORT)
          continue;
        firstCmd = b;
        rxBuf[rxIdx++] = b;
        if (firstCmd == CMD_DISCONNECT_REPORT)
          targetLen = sizeof(DisconnectReportFrame);
        continue;
      }

      if (rxIdx < (int)sizeof(rxBuf))
        rxBuf[rxIdx++] = b;

      if (firstCmd == CMD_DATA_LOOP && rxIdx == 3) {
        uint8_t payloadLen = rxBuf[2];
        targetLen = (size_t)payloadLen + 4U;
        if (targetLen > sizeof(rxBuf)) {
          rxIdx = 0;
          break;
        }
      }

      if (targetLen > 0 && (size_t)rxIdx >= targetLen) {
        pktComplete = true;
        break;
      }
    }

    bool crcOk = false;
    bool gotDisconnectReport = false;

    if (pktComplete) {
      if (firstCmd == CMD_DATA_LOOP) {
        uint8_t recvCRC = rxBuf[rxIdx - 1];
        if (calcCRC8(rxBuf, rxIdx - 1) == recvCRC) {
          crcOk = true;
          sLastErrFlags = 0;
          sLastValidFrameMs = millis();
        } else {
          sLastErrFlags = ERR_CRC;
        }
      } else if (firstCmd == CMD_DISCONNECT_REPORT &&
                 rxIdx >= (int)sizeof(DisconnectReportFrame)) {
        DisconnectReportFrame rpt;
        memcpy(&rpt, rxBuf, sizeof(rpt));
        if (calcCRC8((const uint8_t *)&rpt, 4) == rpt.crc) {
          gotDisconnectReport = true;
          sLastValidFrameMs = millis();
          enterDisconnectMode(true, rpt.nodeId);
        }
      } else {
        sLastErrFlags = ERR_PARSE;
      }
    } else {
      sLastErrFlags = ERR_TIMEOUT;
    }

    if (gotDisconnectReport) {
      publishMonitorSnapshot();
      return;
    }

    if (crcOk) {
      processValidBusPacket(rxBuf, rxIdx);
    }
  }

  if (!sDisconnectMode &&
      (uint32_t)(millis() - sLastValidFrameMs) > LINK_TIMEOUT_MS) {
    enterDisconnectMode(false, 0xFF);
    publishMonitorSnapshot();
    return;
  }

  if (sDisconnectMode) {
    publishMonitorSnapshot();
    return;
  }

  nowUs = micros();
  if ((uint32_t)(nowUs - sLastOutputUs) >= OUTPUT_PERIOD_US) {
    sLastOutputUs += OUTPUT_PERIOD_US;
    if ((uint32_t)(nowUs - sLastOutputUs) >= OUTPUT_PERIOD_US)
      sLastOutputUs = nowUs;

    if (sHasSample)
      sendUnifiedFrame();
  }

  publishMonitorSnapshot();
}

namespace MasterBusiness {

void setup() { master_business_setup(); }

void loop() { master_business_loop(); }

bool getMonitorSnapshot(UiMonitorSnapshot &out) {
  bool ready = false;
  portENTER_CRITICAL(&sMonitorSnapshotMux);
  out = sMonitorSnapshot;
  ready = sMonitorSnapshotReady;
  portEXIT_CRITICAL(&sMonitorSnapshotMux);
  return ready;
}

bool getUiPopupState(UiPopupState &out) {
  portENTER_CRITICAL(&sUiPopupMux);
  out = sUiPopupState;
  portEXIT_CRITICAL(&sUiPopupMux);
  return out.active != 0;
}

void startJoystickCalibration() {
  sJoyCalActive = true;
  sJoyCalData.minX = 100;
  sJoyCalData.maxX = 0;
  sJoyCalData.minY = 100;
  sJoyCalData.maxY = 0;
  sJoyCalPeakCenterX = 0;
  sJoyCalPeakCenterY = 0;
}

void stopJoystickCalibration() { sJoyCalActive = false; }

bool commitJoystickCalibration() {
  if (!sJoyCalActive)
    return false;

  uint8_t dzx = (uint8_t)(sJoyCalPeakCenterX + 1);
  uint8_t dzy = (uint8_t)(sJoyCalPeakCenterY + 1);
  if (dzx < 2)
    dzx = 2;
  if (dzy < 2)
    dzy = 2;

  sJoyCalData.deadzoneX = dzx;
  sJoyCalData.deadzoneY = dzy;

  const uint8_t leftX = (50 > dzx) ? (uint8_t)(50 - dzx) : 0;
  const uint8_t rightX = (uint8_t)((50 + dzx <= 100) ? (50 + dzx) : 100);
  const uint8_t leftY = (50 > dzy) ? (uint8_t)(50 - dzy) : 0;
  const uint8_t rightY = (uint8_t)((50 + dzy <= 100) ? (50 + dzy) : 100);

  sJoyCalData.normMinX = (sJoyCalData.minX < leftX) ? sJoyCalData.minX : leftX;
  sJoyCalData.normMaxX =
      (sJoyCalData.maxX > rightX) ? sJoyCalData.maxX : rightX;
  sJoyCalData.normMinY = (sJoyCalData.minY < leftY) ? sJoyCalData.minY : leftY;
  sJoyCalData.normMaxY =
      (sJoyCalData.maxY > rightY) ? sJoyCalData.maxY : rightY;
  sJoyCalData.valid = 1;

  saveJoystickCalibration();
  sJoyCalActive = false;
  setUiPopup("Joystick saved", false, 1200);
  return true;
}

bool getJoystickCalibration(JoystickCalibrationData &out) {
  out = sJoyCalData;
  return sJoyCalData.valid != 0;
}

bool getJoystickCalibrationLive(JoystickCalibrationData &out, uint8_t &curX,
                                uint8_t &curY) {
  out = sJoyCalData;
  curX = sJoyCalCurX;
  curY = sJoyCalCurY;
  if (sJoyCalActive) {
    uint8_t dzx = (uint8_t)(sJoyCalPeakCenterX + 1);
    uint8_t dzy = (uint8_t)(sJoyCalPeakCenterY + 1);
    out.deadzoneX = (dzx < 2) ? 2 : dzx;
    out.deadzoneY = (dzy < 2) ? 2 : dzy;
  }
  return sJoyCalActive || (sJoyCalData.valid != 0);
}

bool setOutputInterface(uint8_t mode) {
  if (mode != MB_OUTPUT_IF_RS232 && mode != MB_OUTPUT_IF_USB)
    return false;
  portENTER_CRITICAL(&sConfigMux);
  sOutputInterface = mode;
  portEXIT_CRITICAL(&sConfigMux);
  saveOutputConfig();
  setUiPopup((mode == MB_OUTPUT_IF_USB) ? "Output: USB" : "Output: RS232",
             false, 1200);
  return true;
}

uint8_t getOutputInterface() {
  uint8_t mode = MB_OUTPUT_IF_RS232;
  portENTER_CRITICAL(&sConfigMux);
  mode = getOutputInterfaceUnsafe();
  portEXIT_CRITICAL(&sConfigMux);
  return mode;
}

bool setPoseMode(uint8_t mode) {
  if (mode != MB_POSE_MODE_RELATIVE && mode != MB_POSE_MODE_ABSOLUTE)
    return false;
  portENTER_CRITICAL(&sConfigMux);
  sPoseMode = mode;
  portEXIT_CRITICAL(&sConfigMux);
  saveOutputConfig();
  setUiPopup((mode == MB_POSE_MODE_ABSOLUTE) ? "Pose: ABS" : "Pose: REL", false,
             1200);
  return true;
}

uint8_t getPoseMode() {
  uint8_t mode = MB_POSE_MODE_RELATIVE;
  portENTER_CRITICAL(&sConfigMux);
  mode = getPoseModeUnsafe();
  portEXIT_CRITICAL(&sConfigMux);
  return mode;
}

bool setRotationOutputMode(uint8_t mode) {
  if (mode != MB_ROT_OUT_EULER && mode != MB_ROT_OUT_QUATERNION)
    return false;
  portENTER_CRITICAL(&sConfigMux);
  sRotOutputMode = mode;
  portEXIT_CRITICAL(&sConfigMux);
  saveOutputConfig();
  setUiPopup((mode == MB_ROT_OUT_QUATERNION) ? "Rot: QUAT" : "Rot: EUL", false,
             1200);
  return true;
}

uint8_t getRotationOutputMode() {
  uint8_t mode = MB_ROT_OUT_EULER;
  portENTER_CRITICAL(&sConfigMux);
  mode = getRotationOutputModeUnsafe();
  portEXIT_CRITICAL(&sConfigMux);
  return mode;
}

bool getEncoderCalibrationState(EncoderCalibrationState &out) {
  out.raw[0] = sEncRaw[0];
  out.raw[1] = sEncRaw[1];
  out.raw[2] = sEncRaw[2];
  out.offset[0] = sEncOffsets[0];
  out.offset[1] = sEncOffsets[1];
  out.offset[2] = sEncOffsets[2];
  out.dir[0] = sEncDirs[0];
  out.dir[1] = sEncDirs[1];
  out.dir[2] = sEncDirs[2];
  return true;
}

bool calibrateEncoderAxisToDegree(uint8_t axis, uint16_t degree) {
  if (axis >= 3)
    return false;

  uint16_t raw = sEncRaw[axis];
  int dir = (sEncDirs[axis] >= 0) ? 1 : -1;
  uint16_t target = (uint16_t)(degree % 360u);

  int ticks = (int)lroundf((target / 360.0f) * 4096.0f / (float)dir);
  int off = (int)raw - ticks;
  off %= 4096;
  if (off < 0)
    off += 4096;

  sEncOffsets[axis] = (uint16_t)off;
  char msg[32];
  snprintf(msg, sizeof(msg), "A%u -> %u deg", (unsigned)(axis + 1),
           (unsigned)target);
  setUiPopup(msg, false, 900);
  return true;
}

bool toggleEncoderDirection(uint8_t axis) {
  if (axis >= 3)
    return false;
  sEncDirs[axis] = (sEncDirs[axis] >= 0) ? -1 : 1;
  char msg[32];
  snprintf(msg, sizeof(msg), "A%u Dir:%d", (unsigned)(axis + 1),
           (int)sEncDirs[axis]);
  setUiPopup(msg, false, 900);
  return true;
}

bool saveEncoderCalibration() {
  saveEncoderCalibrationInternal();
  setUiPopup("Enc cal saved", false, 1200);
  return true;
}

bool reloadEncoderCalibration() {
  loadEncoderCalibration();
  setUiPopup("Enc cal loaded", false, 1200);
  return true;
}

} // namespace MasterBusiness

void setup() __attribute__((weak));
void loop() __attribute__((weak));

void setup() { MasterBusiness::setup(); }

void loop() { MasterBusiness::loop(); }
