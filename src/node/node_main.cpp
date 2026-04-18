#include "../shared/protocol.h"

#include <AS5600.h>
#include <Arduino.h>
#include <ICM_20948.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <stddef.h>
#include <string.h>
#include <util/atomic.h>

// --- 硬件引脚 ---
#define LED_ERR 8   // PB0
#define LED_STAT 13 // PB5

#define PIN_BTN1 5  // PD5
#define PIN_BTN2 9  // PB1
#define PIN_BTN3 10 // PB2
#define PIN_BTN4 7  // PD7 (摇杆按钮)

#define PIN_WHEEL_MINUS 2 // PD2 / INT0
#define PIN_WHEEL_PLUS 3  // PD3 / INT1

#define PIN_JOY_X A0 // PC0
#define PIN_JOY_Y A1 // PC1

// --- 参数 ---
#define PACKET_QUEUE_DEPTH 8
#define MAX_CHAIN_PAYLOAD 96
#define MAX_CHAIN_FRAME (MAX_CHAIN_PAYLOAD + 4)
#define RX_FIFO_SIZE 128

#define SENSOR_UPDATE_INTERVAL_US 2000UL
#define LINK_TIMEOUT_MS 500UL
#define DISC_TX_PERIOD_US 22222UL // 45Hz
#define STAT_HEARTBEAT_PERIOD_MS 10000UL
#define PARSER_STALL_TIMEOUT_MS 80UL

// --- 按键位 ---
#define BTN_MASK_1 0x01
#define BTN_MASK_2 0x02
#define BTN_MASK_3 0x04
#define BTN_MASK_4 0x08

AS5600 as5600;
ICM_20948_I2C myICM;

uint8_t myType = NODE_TYPE_UNKNOWN;
uint8_t myID = 0;
bool isConfigured = false;

EncoderPayload encCache;
HandlePayload hndCache;

struct PendingPacket {
  uint8_t data[MAX_CHAIN_FRAME];
  uint8_t frameLen;
  uint8_t localButtons;
  uint8_t localWheel;
  uint8_t localPayloadOffset;
  bool prepared;
};

PendingPacket gPacketQueue[PACKET_QUEUE_DEPTH];
uint8_t gQueueCount = 0;

uint8_t gRxFifo[RX_FIFO_SIZE];
uint8_t gRxHead = 0;
uint8_t gRxTail = 0;
uint8_t gRxCount = 0;

enum ParserState {
  PARSER_WAIT_CMD,
  PARSER_READ_FIXED,
  PARSER_READ_DATA_BODY,
};

ParserState gParserState = PARSER_WAIT_CMD;
uint8_t gParseBuf[MAX_CHAIN_FRAME];
uint8_t gParseIndex = 0;
uint8_t gParseNeed = 0;
uint8_t gParseExpectedTotal = 0;

volatile uint8_t gButtonState = 0;
volatile uint8_t gWheelSeq = 0;
volatile uint8_t gWheelPrevAB = 0;
volatile uint8_t gErrFlashReq = 0;

bool gDisconnectMode = false;
uint8_t gDisconnectOriginNode = 0xFF;
uint8_t gDisconnectReason = DISC_REASON_TIMEOUT;
uint8_t gDisconnectSeq = 0;

unsigned long gLastRxPacketMs = 0;
unsigned long gLastSensorReadUs = 0;
unsigned long gLastDisconnectTxUs = 0;
unsigned long gParserLastByteMs = 0;

uint8_t gStatBlinkTarget = 1;
uint8_t gStatBlinkDone = 0;
bool gStatLedOn = false;
bool gStatPause = false;
unsigned long gStatNextMs = 0;

bool gErrFlashPhaseOn = false;
unsigned long gErrNextMs = 0;

static void startupLedPulse();
static void setupInputInterrupts();
static void resetStatPattern();
static void requestErrFlash();
static void updateLeds();

static void updateButtonStateFromPins();
static void updateWheelFromPhases();

static void detectSensors();
static void updateSensorsLatest();
static uint8_t mapJoystickPercent(int rawAdc);

static void clearRxFifo();
static void clearPacketQueue();
static bool rxFifoPush(uint8_t b);
static bool rxFifoPop(uint8_t &b);
static void drainSerialToRxFifo();

static void parserReset();
static void maintainParserHealth();
static void processRxParser();
static void handleCompleteFrame(const uint8_t *frame, uint8_t len);
static void handleInitFrame(const uint8_t *frame, uint8_t len);
static void handleDataFrame(const uint8_t *frame, uint8_t len);
static void handleDisconnectFrame(const uint8_t *frame, uint8_t len);

static void enterDisconnectMode(bool originKnown, uint8_t originNode,
                                uint8_t reasonFlags);
static void sendDisconnectReport();

static void dropSecondOldestPacket();
static bool enqueueDataPacket(const uint8_t *frame, uint8_t len);
static void popFrontPacket();
static bool appendLocalPayload(PendingPacket &slot);
static void processPacketQueue();

ISR(PCINT2_vect) { updateButtonStateFromPins(); }

ISR(PCINT0_vect) { updateButtonStateFromPins(); }

ISR(INT1_vect) { updateWheelFromPhases(); }

ISR(INT0_vect) { updateWheelFromPhases(); }

void setup() {
  wdt_disable();

  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_STAT, OUTPUT);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);
  pinMode(PIN_BTN4, INPUT_PULLUP);
  pinMode(PIN_WHEEL_MINUS, INPUT_PULLUP);
  pinMode(PIN_WHEEL_PLUS, INPUT_PULLUP);

  Serial.begin(250000);

  Wire.begin();
  Wire.setClock(400000);

  startupLedPulse();
  setupInputInterrupts();
  detectSensors();

  clearRxFifo();
  clearPacketQueue();
  parserReset();
  gLastRxPacketMs = millis();

  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();

  drainSerialToRxFifo();
  processRxParser();
  maintainParserHealth();

  unsigned long nowUs = micros();

  if (isConfigured && !gDisconnectMode) {
    if ((unsigned long)(nowUs - gLastSensorReadUs) >=
        SENSOR_UPDATE_INTERVAL_US) {
      gLastSensorReadUs = nowUs;
      updateSensorsLatest();
    }

    processPacketQueue();

    if ((unsigned long)(millis() - gLastRxPacketMs) > LINK_TIMEOUT_MS) {
      enterDisconnectMode(true, myID, DISC_REASON_TIMEOUT);
    }
  } else if (gDisconnectMode) {
    if ((unsigned long)(nowUs - gLastDisconnectTxUs) >= DISC_TX_PERIOD_US) {
      gLastDisconnectTxUs = nowUs;
      sendDisconnectReport();
    }
  }

  updateLeds();
}

static void startupLedPulse() {
  for (uint8_t i = 0; i < 2; i++) {
    digitalWrite(LED_ERR, HIGH);
    digitalWrite(LED_STAT, HIGH);
    delay(90);
    digitalWrite(LED_ERR, LOW);
    digitalWrite(LED_STAT, LOW);
    delay(90);
  }
}

static void setupInputInterrupts() {
  updateButtonStateFromPins();
  uint8_t p = PIND;
  gWheelSeq = 0;

  uint8_t initAB = 0;
  if (p & _BV(PD2))
    initAB |= 0x01;
  if (p & _BV(PD3))
    initAB |= 0x02;
  gWheelPrevAB = initAB;

  PCICR |= _BV(PCIE2) | _BV(PCIE0);
  PCMSK2 |= _BV(PCINT21) | _BV(PCINT23); // PD5, PD7
  PCMSK0 |= _BV(PCINT1) | _BV(PCINT2);   // PB1, PB2

  EICRA &= (uint8_t)~(_BV(ISC01) | _BV(ISC00) | _BV(ISC11) | _BV(ISC10));
  EICRA |= _BV(ISC00) | _BV(ISC10); // INT0/1 任意边沿
  EIMSK |= _BV(INT0) | _BV(INT1);
}

static void updateButtonStateFromPins() {
  uint8_t state = 0;
  if ((PIND & _BV(PD5)) == 0)
    state |= BTN_MASK_1;
  if ((PINB & _BV(PB1)) == 0)
    state |= BTN_MASK_2;
  if ((PINB & _BV(PB2)) == 0)
    state |= BTN_MASK_3;
  if ((PIND & _BV(PD7)) == 0)
    state |= BTN_MASK_4;
  gButtonState = state;
}

static void updateWheelFromPhases() {
  uint8_t ab = 0;
  uint8_t p = PIND;
  if (p & _BV(PD2))
    ab |= 0x01;
  if (p & _BV(PD3))
    ab |= 0x02;

  uint8_t prevAB = gWheelPrevAB;
  uint8_t idx = (uint8_t)((prevAB << 2) | ab);
  static const int8_t kQuadLut[16] = {
      0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
  };

  int8_t delta = kQuadLut[idx & 0x0F];
  gWheelPrevAB = ab;
  if (delta > 0)
    gWheelSeq++;
  else if (delta < 0)
    gWheelSeq--;
}

static void resetStatPattern() {
  gStatBlinkDone = 0;
  gStatLedOn = false;
  gStatPause = false;
  gStatNextMs = millis();
  digitalWrite(LED_STAT, LOW);
}

static void requestErrFlash() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { gErrFlashReq++; }
}

static void updateLeds() {
  unsigned long now = millis();

  if (gDisconnectMode) {
    digitalWrite(LED_ERR, HIGH);
  } else {
    if ((unsigned long)(now - gErrNextMs) >= 50UL) {
      uint8_t req = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { req = gErrFlashReq; }

      if (req > 0) {
        gErrNextMs = now;
        gErrFlashPhaseOn = !gErrFlashPhaseOn;
        digitalWrite(LED_ERR, gErrFlashPhaseOn ? HIGH : LOW);
        if (!gErrFlashPhaseOn) {
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (gErrFlashReq > 0)
              gErrFlashReq--;
          }
        }
      } else {
        gErrFlashPhaseOn = false;
        digitalWrite(LED_ERR, LOW);
      }
    }
  }

  if (!isConfigured || gDisconnectMode) {
    digitalWrite(LED_STAT, LOW);
    return;
  }

  if ((long)(now - gStatNextMs) < 0)
    return;

  if (gStatPause) {
    gStatPause = false;
    gStatBlinkDone = 0;
    gStatLedOn = false;
    gStatNextMs = now;
    return;
  }

  if (!gStatLedOn) {
    gStatLedOn = true;
    digitalWrite(LED_STAT, HIGH);
    gStatNextMs = now + 70UL;
  } else {
    gStatLedOn = false;
    digitalWrite(LED_STAT, LOW);
    gStatBlinkDone++;
    if (gStatBlinkDone >= gStatBlinkTarget) {
      gStatPause = true;
      gStatNextMs = now + STAT_HEARTBEAT_PERIOD_MS;
    } else {
      gStatNextMs = now + 120UL;
    }
  }
}

static uint8_t mapJoystickPercent(int rawAdc) {
  long v = (long)(1023 - rawAdc) * 100L;
  v = (v + 511L) / 1023L;
  if (v < 0)
    v = 0;
  if (v > 100)
    v = 100;
  return (uint8_t)v;
}

static void detectSensors() {
  if (as5600.begin()) {
    myType = NODE_TYPE_ENCODER;
    encCache.nodeType = NODE_TYPE_ENCODER;
    encCache.status = 1;
    encCache.rawAngle = 0;
    encCache.reserved = 0;
    return;
  }

  myICM.begin(Wire, 1);
  if (myICM.status != ICM_20948_Stat_Ok) {
    myICM.begin(Wire, 0);
  }

  if (myICM.status == ICM_20948_Stat_Ok) {
    myType = NODE_TYPE_HANDLE;
    hndCache.nodeType = NODE_TYPE_HANDLE;
    hndCache.status = 1;
    hndCache.accX = hndCache.accY = hndCache.accZ = 0;
    hndCache.gyrX = hndCache.gyrY = hndCache.gyrZ = 0;
    hndCache.magX = hndCache.magY = hndCache.magZ = 0;
    hndCache.btnState = 0;
    hndCache.wheelSeq = 0;
    hndCache.joyX = 50;
    hndCache.joyY = 50;

    ICM_20948_fss_t myFSS;
    myFSS.a = gpm2;
    myFSS.g = dps2000;
    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                       myFSS);
    return;
  }

  myType = NODE_TYPE_UNKNOWN;
  requestErrFlash();
}

static void updateSensorsLatest() {
  if (myType == NODE_TYPE_ENCODER) {
    uint8_t status = as5600.readStatus();
    encCache.status = (status & 0x20) ? 0 : 1; // 0=OK, 1=NoMagnet
    encCache.rawAngle = as5600.readAngle();
    return;
  }

  if (myType != NODE_TYPE_HANDLE)
    return;

  if (myICM.dataReady()) {
    myICM.getAGMT();
    hndCache.status = 0;
    hndCache.accX = (int16_t)myICM.accX();
    hndCache.accY = (int16_t)myICM.accY();
    hndCache.accZ = (int16_t)myICM.accZ();
    hndCache.gyrX = (int16_t)myICM.gyrX();
    hndCache.gyrY = (int16_t)myICM.gyrY();
    hndCache.gyrZ = (int16_t)myICM.gyrZ();
    hndCache.magX = (int16_t)myICM.magX();
    hndCache.magY = (int16_t)myICM.magY();
    hndCache.magZ = (int16_t)myICM.magZ();
  } else {
    hndCache.status = 1;
  }

  hndCache.joyX = mapJoystickPercent(analogRead(PIN_JOY_X));
  hndCache.joyY = mapJoystickPercent(analogRead(PIN_JOY_Y));

  uint8_t wheel = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { wheel = gWheelSeq; }
  hndCache.wheelSeq = wheel;
}

static void clearRxFifo() {
  gRxHead = 0;
  gRxTail = 0;
  gRxCount = 0;
}

static void clearPacketQueue() { gQueueCount = 0; }

static bool rxFifoPush(uint8_t b) {
  if (gRxCount >= RX_FIFO_SIZE)
    return false;
  gRxFifo[gRxTail] = b;
  gRxTail = (uint8_t)((gRxTail + 1) % RX_FIFO_SIZE);
  gRxCount++;
  return true;
}

static bool rxFifoPop(uint8_t &b) {
  if (gRxCount == 0)
    return false;
  b = gRxFifo[gRxHead];
  gRxHead = (uint8_t)((gRxHead + 1) % RX_FIFO_SIZE);
  gRxCount--;
  return true;
}

static void drainSerialToRxFifo() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    if (!rxFifoPush(b)) {
      uint8_t dump = 0;
      rxFifoPop(dump);
      rxFifoPush(b);
      requestErrFlash();
    }
  }
}

static void parserReset() {
  gParserState = PARSER_WAIT_CMD;
  gParseIndex = 0;
  gParseNeed = 0;
  gParseExpectedTotal = 0;
  gParserLastByteMs = millis();
}

static void maintainParserHealth() {
  if (gParserState == PARSER_WAIT_CMD)
    return;

  if ((unsigned long)(millis() - gParserLastByteMs) > PARSER_STALL_TIMEOUT_MS) {
    parserReset();
    requestErrFlash();
  }
}

static void processRxParser() {
  uint8_t b = 0;
  while (rxFifoPop(b)) {
    gParserLastByteMs = millis();

    // In disconnect mode, allow init/disconnect headers to preempt any stale
    // partial frame so the node can recover without power-cycling.
    if (gDisconnectMode &&
        (b == CMD_ADDR_RESET || b == CMD_DISCONNECT_REPORT) &&
        gParserState != PARSER_WAIT_CMD) {
      gParseBuf[0] = b;
      gParseIndex = 1;
      gParseNeed = 4;
      gParserState = PARSER_READ_FIXED;
      continue;
    }

    switch (gParserState) {
    case PARSER_WAIT_CMD:
      if (b == CMD_ADDR_RESET || b == CMD_DISCONNECT_REPORT) {
        gParseBuf[0] = b;
        gParseIndex = 1;
        gParseNeed = 4;
        gParserState = PARSER_READ_FIXED;
      } else if (!gDisconnectMode && b == CMD_DATA_LOOP) {
        gParseBuf[0] = b;
        gParseIndex = 1;
        gParseNeed = 2;
        gParserState = PARSER_READ_FIXED;
      }
      break;

    case PARSER_READ_FIXED:
      if (gParseIndex >= MAX_CHAIN_FRAME) {
        requestErrFlash();
        parserReset();
        break;
      }
      gParseBuf[gParseIndex++] = b;
      if (--gParseNeed == 0) {
        if (gParseBuf[0] == CMD_DATA_LOOP) {
          uint8_t payloadLen = gParseBuf[2];
          if (payloadLen > MAX_CHAIN_PAYLOAD) {
            requestErrFlash();
            parserReset();
            break;
          }
          gParseExpectedTotal = (uint8_t)(payloadLen + 4);
          gParseNeed = (uint8_t)(payloadLen + 1);
          gParserState = PARSER_READ_DATA_BODY;
        } else {
          handleCompleteFrame(gParseBuf, 5);
          parserReset();
        }
      }
      break;

    case PARSER_READ_DATA_BODY:
      if (gParseIndex >= MAX_CHAIN_FRAME) {
        requestErrFlash();
        parserReset();
        break;
      }
      gParseBuf[gParseIndex++] = b;
      if (--gParseNeed == 0) {
        handleCompleteFrame(gParseBuf, gParseExpectedTotal);
        parserReset();
      }
      break;
    }
  }
}

static void handleCompleteFrame(const uint8_t *frame, uint8_t len) {
  if (len == 0)
    return;

  switch (frame[0]) {
  case CMD_ADDR_RESET:
    handleInitFrame(frame, len);
    break;
  case CMD_DATA_LOOP:
    handleDataFrame(frame, len);
    break;
  case CMD_DISCONNECT_REPORT:
    handleDisconnectFrame(frame, len);
    break;
  default:
    break;
  }
}

static void handleInitFrame(const uint8_t *frame, uint8_t len) {
  if (len != 5)
    return;
  if (calcCRC8(frame, 4) != frame[4]) {
    requestErrFlash();
    return;
  }

  clearRxFifo();
  clearPacketQueue();
  parserReset();

  gDisconnectMode = false;
  gDisconnectOriginNode = 0xFF;
  gDisconnectReason = DISC_REASON_TIMEOUT;

  detectSensors();

  uint8_t total = frame[1];
  uint8_t nEnc = frame[2];
  uint8_t nHnd = frame[3];

  myID = total;
  isConfigured = true;

  total++;
  if (myType == NODE_TYPE_ENCODER)
    nEnc++;
  else if (myType == NODE_TYPE_HANDLE)
    nHnd++;

  uint8_t out[5] = {CMD_ADDR_RESET, total, nEnc, nHnd, 0};
  out[4] = calcCRC8(out, 4);
  Serial.write(out, sizeof(out));

  gStatBlinkTarget = (uint8_t)(myID + 1);
  if (gStatBlinkTarget == 0)
    gStatBlinkTarget = 1;
  resetStatPattern();

  startupLedPulse();
  gLastRxPacketMs = millis();
}

static void handleDataFrame(const uint8_t *frame, uint8_t len) {
  if (len < 4)
    return;
  if (calcCRC8(frame, (size_t)(len - 1)) != frame[len - 1]) {
    requestErrFlash();
    return;
  }

  gLastRxPacketMs = millis();

  if (!isConfigured || gDisconnectMode)
    return;

  enqueueDataPacket(frame, len);
}

static void handleDisconnectFrame(const uint8_t *frame, uint8_t len) {
  if (len != sizeof(DisconnectReportFrame))
    return;
  if (calcCRC8(frame, 4) != frame[4]) {
    requestErrFlash();
    return;
  }

  gLastRxPacketMs = millis();

  if (!gDisconnectMode) {
    enterDisconnectMode(true, frame[1],
                        (uint8_t)(frame[2] | DISC_REASON_PROPAGATED));
  } else if (gDisconnectOriginNode == 0xFF) {
    gDisconnectOriginNode = frame[1];
  }

  // 快速透传一次，降低下游响应延迟。
  Serial.write(frame, len);
}

static void enterDisconnectMode(bool originKnown, uint8_t originNode,
                                uint8_t reasonFlags) {
  gDisconnectMode = true;
  gDisconnectOriginNode = originKnown ? originNode : 0xFF;
  gDisconnectReason = reasonFlags;
  gDisconnectSeq = 0;
  gLastDisconnectTxUs = 0;
  clearPacketQueue();
  clearRxFifo();
  parserReset();
}

static void sendDisconnectReport() {
  DisconnectReportFrame rpt;
  rpt.cmd = CMD_DISCONNECT_REPORT;
  rpt.nodeId = (gDisconnectOriginNode == 0xFF) ? myID : gDisconnectOriginNode;
  rpt.reasonFlags = gDisconnectReason;
  rpt.seq = gDisconnectSeq++;
  rpt.crc = calcCRC8((const uint8_t *)&rpt, 4);
  Serial.write((const uint8_t *)&rpt, sizeof(rpt));
}

static void dropSecondOldestPacket() {
  if (gQueueCount < 2)
    return;

  for (uint8_t i = 2; i < gQueueCount; i++) {
    gPacketQueue[i - 1] = gPacketQueue[i];
  }
  gQueueCount--;
  requestErrFlash();
}

static bool enqueueDataPacket(const uint8_t *frame, uint8_t len) {
  if (len > MAX_CHAIN_FRAME) {
    requestErrFlash();
    return false;
  }

  if (gQueueCount >= PACKET_QUEUE_DEPTH)
    dropSecondOldestPacket();

  if (gQueueCount >= PACKET_QUEUE_DEPTH) {
    requestErrFlash();
    return false;
  }

  PendingPacket &slot = gPacketQueue[gQueueCount++];
  memcpy(slot.data, frame, len);
  slot.frameLen = len;
  slot.localButtons = 0;
  slot.localWheel = 0;
  slot.localPayloadOffset = 0;
  slot.prepared = false;
  return true;
}

static void popFrontPacket() {
  if (gQueueCount == 0)
    return;
  for (uint8_t i = 1; i < gQueueCount; i++) {
    gPacketQueue[i - 1] = gPacketQueue[i];
  }
  gQueueCount--;
}

static bool appendLocalPayload(PendingPacket &slot) {
  if (slot.frameLen < 4 || slot.data[0] != CMD_DATA_LOOP)
    return false;

  uint8_t payloadLen = slot.data[2];
  uint8_t appendOffset = (uint8_t)(3 + payloadLen);
  if (appendOffset >= MAX_CHAIN_FRAME)
    return false;

  updateSensorsLatest();

  uint8_t btnState = 0;
  uint8_t wheelSeq = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    btnState = gButtonState;
    wheelSeq = gWheelSeq;
  }

  slot.localButtons = btnState;
  slot.localWheel = wheelSeq;
  slot.localPayloadOffset = appendOffset;

  uint8_t addLen = 0;
  if (myType == NODE_TYPE_ENCODER) {
    EncoderPayload payload = encCache;
    addLen = sizeof(EncoderPayload);
    if ((uint16_t)appendOffset + addLen + 1U > MAX_CHAIN_FRAME)
      return false;
    memcpy(slot.data + appendOffset, &payload, addLen);
  } else if (myType == NODE_TYPE_HANDLE) {
    HandlePayload payload = hndCache;
    payload.btnState = slot.localButtons;
    payload.wheelSeq = slot.localWheel;
    addLen = sizeof(HandlePayload);
    if ((uint16_t)appendOffset + addLen + 1U > MAX_CHAIN_FRAME)
      return false;
    memcpy(slot.data + appendOffset, &payload, addLen);
  } else {
    return false;
  }

  uint16_t newPayloadLen = (uint16_t)payloadLen + addLen;
  if (newPayloadLen > MAX_CHAIN_PAYLOAD)
    return false;

  slot.data[2] = (uint8_t)newPayloadLen;
  slot.frameLen = (uint8_t)(newPayloadLen + 4);
  slot.data[slot.frameLen - 1] =
      calcCRC8(slot.data, (size_t)(slot.frameLen - 1));
  slot.prepared = true;
  return true;
}

static void processPacketQueue() {
  if (gQueueCount == 0)
    return;

  PendingPacket &slot = gPacketQueue[0];
  if (!slot.prepared) {
    if (!appendLocalPayload(slot)) {
      requestErrFlash();
      popFrontPacket();
      return;
    }
  }

  Serial.write(slot.data, slot.frameLen);
  popFrontPacket();
}