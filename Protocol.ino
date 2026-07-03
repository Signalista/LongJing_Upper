#include "Globals.h"

void sendReply(uint8_t source, const String& msg) {
  String out = getTimestampHeader();
  out += msg;

  if (source == 0) {
    Serial.println(out);
  }

#if ENABLE_BLE
  if (source == 1) {
    bleSendText(out);
  }
#endif

#if WIFI_MODE != WIFI_MODE_OFF
  if (source == 2) {
    if (wifiClient && wifiClient.connected()) {
      wifiClient.println(out);
    }
  }
#endif
}


void broadcastMsg(const String& msg) {
  String out = getTimestampHeader();
  out += msg;

  Serial.println(out);

#if ENABLE_BLE
  bleSendText(out);
#endif

#if WIFI_MODE != WIFI_MODE_OFF
  if (wifiClient && wifiClient.connected()) {
    wifiClient.println(out);
  }
#endif
}


void broadcastRaw(const String& msg) {
  Serial.print(msg);

#if ENABLE_BLE
  bleSendText(msg);
#endif

#if WIFI_MODE != WIFI_MODE_OFF
  if (wifiClient && wifiClient.connected()) {
    wifiClient.print(msg);
  }
#endif
}


String getWifiBriefText() {
#if WIFI_MODE == WIFI_MODE_OFF
  return "disabled";
#else
  if (wifiClient && wifiClient.connected()) {
    return "connected";
  }
  return "waiting";
#endif
}


String getStatusText() {
  updateFaultLatch();

  String msg = "";

  msg += "A_pos=";
  msg += String(getRelativePosA(), 4);
  msg += ", A_vel=";
  msg += String(motorA.shaft_velocity, 4);
  msg += ", A_sleep=";
  msg += (sleepA ? "YES" : "NO");
  msg += ",\n";

  msg += "B_pos=";
  msg += String(getRelativePosB(), 4);
  msg += ", B_vel=";
  msg += String(motorB.shaft_velocity, 4);
  msg += ", B_sleep=";
  msg += (sleepB ? "YES" : "NO");
  msg += ", \n";

  msg += "C_pos=";
  msg += String(getRelativePosC(), 4);
  msg += ", C_vel=";
  msg += String(motorC.shaft_velocity, 4);
  msg += ", C_sleep=";
  msg += (sleepC ? "YES" : "NO");
  msg += ",\n";

  msg += "Von=";
  msg += String(readVonVoltage(), 2);
  msg += ", Vfoc=";
  msg += String(Vfoc, 4);
  msg += ", Ifoc=";
  msg += String(Ifoc, 4);
  msg += ", Fault=";
  msg += getFaultBriefText();
  msg += ", \n";

  msg += "USB=ready";

#if ENABLE_BLE
  msg += ", BLE=";
  msg += (bleDeviceConnected ? "connected" : "disconnected");
#else
  msg += ", BLE=disabled";
#endif

  msg += ", WIFI=";
  msg += getWifiBriefText();

  return msg;
}


String getSingleAxisStatus(char axis) {
  updateFaultLatch();

  String prefix = "";
  float pos = 0.0f;
  float vel = 0.0f;
  uint8_t mode = AXIS_MODE_POSITION;
  float targetVel = 0.0f;
  float targetPos = 0.0f;
  float targetIq = 0.0f;
  String limit = "NONE";
  String sleep = "NO";
  String fault = "Perfect";
  float maxPos = 0.0f;
  float minPos = 0.0f;
  float maxVel = 0.0f;

  if (axis == 'A') {
    prefix = "A";
    pos = getRelativePosA();
    vel = motorA.shaft_velocity;
    mode = modeA;
    targetVel = targetVelA;
    targetPos = targetPosA;
    targetIq = targetIqA;
    limit = getAxisLimitText(limitAUpper, limitALower);
    sleep = (sleepA ? "YES" : "NO");
    fault = getAxisFaultBriefText('A');
    maxPos = A_MAX_POS;
    minPos = A_MIN_POS;
    maxVel = A_MAX_VEL;
  } else if (axis == 'B') {
    prefix = "B";
    pos = getRelativePosB();
    vel = motorB.shaft_velocity;
    mode = modeB;
    targetVel = targetVelB;
    targetPos = targetPosB;
    targetIq = targetIqB;
    limit = getAxisLimitText(limitBUpper, limitBLower);
    sleep = (sleepB ? "YES" : "NO");
    fault = getAxisFaultBriefText('B');
    maxPos = B_MAX_POS;
    minPos = B_MIN_POS;
    maxVel = B_MAX_VEL;
  } else if (axis == 'C') {
    prefix = "C";
    pos = getRelativePosC();
    vel = motorC.shaft_velocity;
    mode = modeC;
    targetVel = targetVelC;
    targetPos = targetPosC;
    targetIq = targetIqC;
    limit = getAxisLimitText(limitCUpper, limitCLower);
    sleep = (sleepC ? "YES" : "NO");
    fault = getAxisFaultBriefText('C');
    maxPos = C_MAX_POS;
    minPos = C_MIN_POS;
    maxVel = C_MAX_VEL;
  } else {
    return "ERR, UNKNOWN_AXIS";
  }

  String msg = "";

  msg += prefix + "_pos=";
  msg += String(pos, 4);
  msg += ", " + prefix + "_vel=";
  msg += String(vel, 4);
  msg += ", " + prefix + "_mode=";
  msg += axisModeToString(mode);
  msg += ", \n";

  msg += prefix + "_target_pos=";
  msg += String(targetPos, 4);
  msg += ", " + prefix + "_target_vel=";
  msg += String(targetVel, 4);
  msg += ", " + prefix + "_target_iq=";
  msg += String(targetIq, 4);
  msg += ", \n";

  msg += prefix + "_MAX_POS=";
  msg += String(maxPos, 4);
  msg += ", " + prefix + "_MIN_POS=";
  msg += String(minPos, 4);
  msg += ", " + prefix + "_MAX_VEL=";
  msg += String(maxVel, 4);
  msg += ", " + prefix + "_MAX_IQ=";
  msg += String(Ifoc, 4);
  msg += ", \n";

  msg += prefix + "_limit=";
  msg += limit;
  msg += ", " + prefix + "_sleep=";
  msg += sleep;
  msg += ", " + prefix + "_fault=";
  msg += fault;
  msg += ",\n";

  return msg;
}


String getCommInfo() {
  String msg = "";

  msg += "USB=ready, USB_BAUD=115200, \n";

#if ENABLE_BLE
  msg += "BLE=";
  msg += (bleDeviceConnected ? "connected" : "disconnected");
  msg += ", BLE_NAME=";
  msg += DEVICE_NAME;
#else
  msg += "BLE=disabled";
#endif

  msg += ", \n";

#if WIFI_MODE == WIFI_MODE_OFF
  msg += "WIFI=disabled";
#else

#if WIFI_MODE == WIFI_MODE_ROUTER
  msg += "WIFI=router";
#elif WIFI_MODE == WIFI_MODE_PHONE
  msg += "WIFI=phone_hotspot";
#elif WIFI_MODE == WIFI_MODE_AP
  msg += "WIFI=ap";
#endif

  msg += ", WIFI_CLIENT=";
  msg += ((wifiClient && wifiClient.connected()) ? "connected" : "waiting");

  msg += ", WIFI_IP=";
#if WIFI_MODE == WIFI_MODE_AP
  msg += WiFi.softAPIP().toString();
#else
  msg += WiFi.localIP().toString();
#endif

  msg += ", PORT=";
  msg += String(WIFI_TCP_PORT);

#endif

  return msg;
}


String getLimitInfo() {
  String msg = "";

  msg += "A_MAX_VEL=";
  msg += String(A_MAX_VEL, 4);
  msg += ", A_MIN_POS=";
  msg += String(A_MIN_POS, 4);
  msg += ", A_MAX_POS=";
  msg += String(A_MAX_POS, 4);
  msg += ",\n";

  msg += "B_MAX_VEL=";
  msg += String(B_MAX_VEL, 4);
  msg += ", B_MIN_POS=";
  msg += String(B_MIN_POS, 4);
  msg += ", B_MAX_POS=";
  msg += String(B_MAX_POS, 4);
  msg += ",\n";

  msg += "C_MAX_VEL=";
  msg += String(C_MAX_VEL, 4);
  msg += ", C_MIN_POS=";
  msg += String(C_MIN_POS, 4);
  msg += ", C_MAX_POS=";
  msg += String(C_MAX_POS, 4);
  msg += ", \n";

  msg += "VON_MIN=";
  msg += String(VON_MIN, 2);
  msg += ", VON_MAX=";
  msg += String(VON_MAX, 2);
  msg += ", IF_MAX=";
  msg += String(FOC_CURRENT_LIMIT_MAX, 4);
  msg += ", \n";

  msg += "MOTOR_RS=";
  msg += String(MOTOR_PHASE_RESISTANCE, 4);
  msg += ", MOTOR_KV=";
  msg += String(MOTOR_KV_RATING, 2);
  msg += ", MOTOR_LQ=";
  msg += String(MOTOR_LQ, 6);
  msg += ", MOTOR_LD=";
  msg += String(MOTOR_LD, 6);

  return msg;
}


String getCxkText() {
  String msg = "\n";
  msg += "⠀⠀⠀⠀⠰⢷⢿⠄\n";
  msg += "⠀⠀⠀⠀⠀⣼⣷⣄\n";
  msg += "⠀⠀⣤⣿⣇⣿⣿⣧⣿⡄\n";
  msg += "⢴⠾⠋⠀⠀⠻⣿⣷⣿⣿⡀\n";
  msg += "⣟⣯⠀⢀⣿⣿⡿⢿⠈⣿\n";
  msg += "⠀⠀⠀⢠⣿⡿⠁⠀⡊⠀⠙\n";
  msg += "⠀⠀⠀⢿⣿⠀⠀⠹⣿\n";
  msg += "⠀⠀⠀⠀⠹⣷⡀⠀⣿⡄\n";
  msg += "⠀⠀⠀⠀⣀⣼⣿⠀⢈⣧";
  return msg;
}


static bool parseFloatList(String text, float* values, uint8_t count) {
  text.replace("，", ",");
  text.trim();

  for (uint8_t i = 0; i < count; i++) {
    int commaIndex = text.indexOf(',');
    String item;

    if (i == count - 1) {
      if (commaIndex >= 0) {
        return false;
      }
      item = text;
    } else {
      if (commaIndex < 0) {
        return false;
      }
      item = text.substring(0, commaIndex);
      text = text.substring(commaIndex + 1);
    }

    item.trim();
    if (item.length() == 0) {
      return false;
    }

    char* endPtr = nullptr;
    values[i] = strtof(item.c_str(), &endPtr);
    if (endPtr == item.c_str() || *endPtr != '\0') {
      return false;
    }
  }

  return true;
}


void handleCommand(String cmd, uint8_t source) {
  cmd.trim();

  String rawCmd = cmd;
  String upperCmd = cmd;
  upperCmd.toUpperCase();

  if (upperCmd.length() == 0) {
    return;
  }

  if (upperCmd == "HELLO 5090") {
    sendReply(source, "Hello wxy");
    return;
  }

  if (upperCmd == "HELLO 5060") {
    sendReply(source, "No I'm 5090");
    return;
  }

  if (upperCmd == "CXK") {
    sendReply(source, getCxkText());
    return;
  }

  if (upperCmd == "STAT?") {
    sendReply(source, getStatusText());
    return;
  }

  if (upperCmd == "STATA?") {
    sendReply(source, getSingleAxisStatus('A'));
    return;
  }

  if (upperCmd == "STATB?") {
    sendReply(source, getSingleAxisStatus('B'));
    return;
  }

  if (upperCmd == "STATC?") {
    sendReply(source, getSingleAxisStatus('C'));
    return;
  }

  if (upperCmd == "ENC?") {
    String msg = "";
    msg += "sensorA_single=";
    msg += String(sensorA.getSensorAngle(), 6);
    msg += ", sensorA_multi=";
    msg += String(sensorA.getAngle(), 6);
    msg += ", sensorB_single=";
    msg += String(sensorB.getSensorAngle(), 6);
    msg += ", sensorB_multi=";
    msg += String(sensorB.getAngle(), 6);
    msg += ", sensorC_single=";
    msg += String(sensorC.getSensorAngle(), 6);
    msg += ", sensorC_multi=";
    msg += String(sensorC.getAngle(), 6);
    sendReply(source, msg);
    return;
  }

  if (upperCmd == "COM?") {
    sendReply(source, getCommInfo());
    return;
  }

  if (upperCmd == "FAULT?") {
    sendReply(source, getFaultInfo());
    return;
  }

  if (upperCmd == "CLEARFAULT") {
    sendReply(source, clearFaultLatch());
    return;
  }

  if (upperCmd == "LIMIT?") {
    sendReply(source, getLimitInfo());
    return;
  }

  if (upperCmd == "VF?") {
    sendReply(source, getVfocInfo());
    return;
  }

  if (upperCmd == "IF?") {
    sendReply(source, getIfocInfo());
    return;
  }

  if (upperCmd == "TIME?") {
    sendReply(source, getTimeInfo());
    return;
  }

  if (upperCmd.startsWith("TIME") && upperCmd.length() > 4) {
    String valueText = upperCmd.substring(4);

    if (valueText.length() != 6) {
      sendReply(source, "ERR, TIME_FORMAT, USE_TIMEHHMMSS");
      return;
    }

    for (uint8_t i = 0; i < valueText.length(); i++) {
      if (!isDigit(valueText.charAt(i))) {
        sendReply(source, "ERR, TIME_FORMAT, USE_TIMEHHMMSS");
        return;
      }
    }

    uint8_t hh = valueText.substring(0, 2).toInt();
    uint8_t mm = valueText.substring(2, 4).toInt();
    uint8_t ss = valueText.substring(4, 6).toInt();

    if (hh > 23 || mm > 59 || ss > 59) {
      sendReply(source, "ERR, TIME_RANGE, USE_000000_TO_235959");
      return;
    }

    syncHostTimeHms(hh, mm, ss);

    String msg = "";
    msg += "OK, TIME_SYNC";
    msg += ", Time_sync=YES";
    msg += ", Time=";
    msg += valueText;
    msg += ", Time_offset_ms=";
    msg += String((long long)timeOffsetMs);

    sendReply(source, msg);
    return;
  }

  if (upperCmd == "MUTE") {
    heartbeat = 0;
    String msg = "OK, heartbeat MUTE";
    sendReply(source, msg);
    return;
  }

  if (upperCmd == "BEAT") {
    heartbeat = 1;
    String msg = "OK, heartbeat GO ON";
    sendReply(source, msg);
    return;
  }

  if (upperCmd == "ZERO?") {
    sendReply(source, getCalibrationInfo());
    return;
  }

  if (upperCmd == "ZERO!") {
    sendReply(source, setCurrentPositionAsZero('X'));
    return;
  }

  if (upperCmd == "ZA!") {
    sendReply(source, setCurrentPositionAsZero('A'));
    return;
  }

  if (upperCmd == "ZB!") {
    sendReply(source, setCurrentPositionAsZero('B'));
    return;
  }

  if (upperCmd == "ZC!") {
    sendReply(source, setCurrentPositionAsZero('C'));
    return;
  }

  if (upperCmd == "ALL0") {
    allTargetZero(source);
    return;
  }

  if (upperCmd == "SLEEP") {
    sleepAllAxis();
    sendReply(source, "OK, SLEEP");
    return;
  }

  if (upperCmd == "SLEEPA") {
    sleepAxis('A');
    sendReply(source, "OK, SLEEPA");
    return;
  }

  if (upperCmd == "SLEEPB") {
    sleepAxis('B');
    sendReply(source, "OK, SLEEPB");
    return;
  }

  if (upperCmd == "SLEEPC") {
    sleepAxis('C');
    sendReply(source, "OK, SLEEPC");
    return;
  }

  if (upperCmd == "WAKE") {
    if (!canWakeNow()) {
      sendReply(source, "ERR, WAKE_DENIED, USE_FAULT?_AND_CLEARFAULT");
      return;
    }

    if (wakeAllAxis()) {
      sendReply(source, "OK, WAKE");
    } else {
      sendReply(source, "ERR, WAKE_FAILED");
    }
    return;
  }

  if (upperCmd == "WAKEA") {
    if (!canWakeNow()) {
      sendReply(source, "ERR, WAKEA_DENIED, USE_FAULT?_AND_CLEARFAULT");
      return;
    }

    if (wakeAxis('A')) {
      sendReply(source, "OK, WAKEA");
    } else {
      sendReply(source, "ERR, WAKEA_FAILED");
    }
    return;
  }

  if (upperCmd == "WAKEB") {
    if (!canWakeNow()) {
      sendReply(source, "ERR, WAKEB_DENIED, USE_FAULT?_AND_CLEARFAULT");
      return;
    }

    if (wakeAxis('B')) {
      sendReply(source, "OK, WAKEB");
    } else {
      sendReply(source, "ERR, WAKEB_FAILED");
    }
    return;
  }

  if (upperCmd == "WAKEC") {
    if (!canWakeNow()) {
      sendReply(source, "ERR, WAKEC_DENIED, USE_FAULT?_AND_CLEARFAULT");
      return;
    }

    if (wakeAxis('C')) {
      sendReply(source, "OK, WAKEC");
    } else {
      sendReply(source, "ERR, WAKEC_FAILED");
    }
    return;
  }

  if (upperCmd.startsWith("VF") && upperCmd.length() > 2) {
    float value = upperCmd.substring(2).toFloat();
    setVfoc(value, source);
    return;
  }

  if (upperCmd.startsWith("IF") && upperCmd.length() > 2) {
    float value = upperCmd.substring(2).toFloat();
    setIfoc(value, source);
    return;
  }

  if (upperCmd.length() >= 3) {
    char type = upperCmd.charAt(0);
    char axis = upperCmd.charAt(1);

    if ((type == 'V' || type == 'P' || type == 'D' || type == 'T') &&
        (axis == 'A' || axis == 'B' || axis == 'C')) {
      float value = upperCmd.substring(2).toFloat();

      if (type == 'V') {
        setAxisVelocity(axis, value, source);
        return;
      }

      if (type == 'P') {
        setAxisPosition(axis, value, source);
        return;
      }

      if (type == 'D') {
        setAxisDisplacement(axis, value, source);
        return;
      }

      if (type == 'T') {
        setAxisTorqueCurrent(axis, value, source);
        return;
      }
    }
  }

  sendReply(source, "ERR, UNKNOWN_CMD=" + rawCmd);
}


void broadcastRunningStatus() {
  broadcastMsg(getStatusText());
}
