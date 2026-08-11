#include "Globals.h"

// ============================================================
// MT6701 SSI 编码器类
// 作用：以 24bit SSI 帧读取 MT6701，取前 14bit 角度
// ============================================================

MT6701SSI::MT6701SSI(uint8_t csPin, SPIClass* spiBus) {
  _csPin = csPin;
  _spiBus = spiBus;
  _angle = 0.0f;
}

void MT6701SSI::init() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  this->Sensor::init();
}

float MT6701SSI::getSensorAngle() {
  uint32_t raw = readRaw24();
  uint16_t angle14 = (raw >> 10) & 0x3FFF;
  _angle = ((float)angle14 / 16384.0f) * _2PI;
  return _angle;
}

uint32_t MT6701SSI::readRaw24() {
  SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

  _spiBus->beginTransaction(settings);
  digitalWrite(_csPin, LOW);

  uint8_t b0 = _spiBus->transfer(0x00);
  uint8_t b1 = _spiBus->transfer(0x00);
  uint8_t b2 = _spiBus->transfer(0x00);

  digitalWrite(_csPin, HIGH);
  _spiBus->endTransaction();

  return ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;
}

// ============================================================
// 编码器对象
// ============================================================

SPIClass spiBus(FSPI);

MT6701SSI sensorA(PIN_ENC_A_CS, &spiBus);
MT6701SSI sensorB(PIN_ENC_B_CS, &spiBus);
MT6701SSI sensorC(PIN_ENC_C_CS, &spiBus);
