#include "Globals.h"

// ============================================================
// Flash / NVS 对象和零位运行变量
// ============================================================

Preferences zeroPrefs;

float homeAOffset = 0.0f;
float homeBOffset = 0.0f;
float homeCOffset = 0.0f;

void loadZeroOffsetsFromFlash() {
  zeroPrefs.begin("zero", false);

  homeAOffset = zeroPrefs.getFloat("A_offset", 0.0f);
  homeBOffset = zeroPrefs.getFloat("B_offset", 0.0f);
  homeCOffset = zeroPrefs.getFloat("C_offset", 0.0f);
}


void saveZeroOffsetsToFlash() {
  zeroPrefs.putFloat("A_offset", homeAOffset);
  zeroPrefs.putFloat("B_offset", homeBOffset);
  zeroPrefs.putFloat("C_offset", homeCOffset);
}


void printZeroOffsetsAtBoot() {
  Serial.println("Zero offsets loaded from Flash:");

  Serial.print("A_offset=");
  Serial.println(homeAOffset, 6);

  Serial.print("B_offset=");
  Serial.println(homeBOffset, 6);

  Serial.print("C_offset=");
  Serial.println(homeCOffset, 6);
}


float normalizeToPi(float angle) {
  while (angle > PI) angle -= _2PI;
  while (angle < -PI) angle += _2PI;
  return angle;
}


float getRelativePosA() {
  sensorA.update();
  return normalizeToPi(sensorA.getAngle() - homeAOffset);
}


float getRelativePosB() {
  sensorB.update();
  return normalizeToPi(sensorB.getAngle() - homeBOffset);
}


float getRelativePosC() {
  sensorC.update();
  return normalizeToPi(sensorC.getAngle() - homeCOffset);
}


float getAbsoluteTargetA(float relativeTarget) {
  return homeAOffset + relativeTarget;
}


float getAbsoluteTargetB(float relativeTarget) {
  return homeBOffset + relativeTarget;
}


float getAbsoluteTargetC(float relativeTarget) {
  return homeCOffset + relativeTarget;
}


String getCalibrationInfo() {
  String msg = "";

  msg += "A_offset=";
  msg += String(homeAOffset, 6);
  msg += ", B_offset=";
  msg += String(homeBOffset, 6);
  msg += ", C_offset=";
  msg += String(homeCOffset, 6);

  return msg;
}


String setCurrentPositionAsZero(char axis) {
  if (axis == 'A') {
    sensorA.update();
    homeAOffset = sensorA.getAngle();
    targetPosA = 0.0f;
  } else if (axis == 'B') {
    sensorB.update();
    homeBOffset = sensorB.getAngle();
    targetPosB = 0.0f;
  } else if (axis == 'C') {
    sensorC.update();
    homeCOffset = sensorC.getAngle();
    targetPosC = 0.0f;
  } else {
    sensorA.update();
    homeAOffset = sensorA.getAngle();
    sensorB.update();
    homeBOffset = sensorB.getAngle();
    sensorC.update();
    homeCOffset = sensorC.getAngle();

    targetPosA = 0.0f;
    targetPosB = 0.0f;
    targetPosC = 0.0f;
  }

  saveZeroOffsetsToFlash();

  String msg = "OK, ";
  msg += getCalibrationInfo();
  return msg;
}
