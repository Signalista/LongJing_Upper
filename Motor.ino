#include "Globals.h"

// ============================================================
// 电机对象定义
// ============================================================

BLDCDriver3PWM driverA(PIN_A_U, PIN_A_V, PIN_A_W);
BLDCDriver3PWM driverB(PIN_B_U, PIN_B_V, PIN_B_W);
BLDCDriver3PWM driverC(PIN_C_U, PIN_C_V, PIN_C_W);

BLDCMotor motorA(MOTOR_A_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV_RATING, MOTOR_LQ, MOTOR_LD);
BLDCMotor motorB(MOTOR_B_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV_RATING, MOTOR_LQ, MOTOR_LD);
BLDCMotor motorC(MOTOR_C_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV_RATING, MOTOR_LQ, MOTOR_LD);

InlineCurrentSense currentSenseA(CURRENT_SHUNT_RESISTANCE, CURRENT_SENSE_GAIN,
                                 PIN_A_CURRENT_U, PIN_A_CURRENT_V);
InlineCurrentSense currentSenseB(CURRENT_SHUNT_RESISTANCE, CURRENT_SENSE_GAIN,
                                 PIN_B_CURRENT_U, PIN_B_CURRENT_V);
InlineCurrentSense currentSenseC(CURRENT_SHUNT_RESISTANCE, CURRENT_SENSE_GAIN,
                                 PIN_C_CURRENT_U, PIN_C_CURRENT_V);

// 当前 FOC 最大输出电压，运行时可由 VF 指令修改，断电后恢复默认值
float Vfoc = FOC_VOLTAGE_LIMIT_DEFAULT;
float Ifoc = FOC_CURRENT_LIMIT_DEFAULT;

// PID gains are persisted in ESP32 NVS. The values in setupMotor() are used
// only when no saved value exists yet.
Preferences pidPrefs;
static bool pidPrefsReady = false;

// ============================================================
// 电机目标和模式变量
// ============================================================

uint8_t modeA = AXIS_MODE_VELOCITY;
uint8_t modeB = AXIS_MODE_VELOCITY;
uint8_t modeC = AXIS_MODE_VELOCITY;

float targetVelA = 0.0f;
float targetVelB = 0.0f;
float targetVelC = 0.0f;

float targetPosA = 0.0f;
float targetPosB = 0.0f;
float targetPosC = 0.0f;

// FOC current torque mode q-axis current commands, unit: A
float targetIqA = 0.0f;
float targetIqB = 0.0f;
float targetIqC = 0.0f;

// ============================================================
// sleep 状态
// ============================================================

bool sleepA = true;
bool sleepB = true;
bool sleepC = true;

// ============================================================
// 软限位状态
// ============================================================

bool limitAUpper = false;
bool limitALower = false;
bool limitBUpper = false;
bool limitBLower = false;
bool limitCUpper = false;
bool limitCLower = false;

String axisModeToString(uint8_t mode) {
  if (mode == AXIS_MODE_VELOCITY) {
    return "VEL";
  }
  if (mode == AXIS_MODE_TORQUE) {
    return "TORQUE";
  }
  return "POS";
}


void applyVfoc() {
  driverA.voltage_limit = Vfoc;
  driverB.voltage_limit = Vfoc;
  driverC.voltage_limit = Vfoc;

  motorA.updateVoltageLimit(Vfoc);
  motorB.updateVoltageLimit(Vfoc);
  motorC.updateVoltageLimit(Vfoc);
}


void setVfoc(float value, uint8_t source) {
  Vfoc = constrain(value, 0.2f, FOC_VOLTAGE_LIMIT_MAX);
  applyVfoc();

  String msg = "";
  msg += "OK, VF=";
  msg += String(Vfoc, 4);

  sendReply(source, msg);
}


String getVfocInfo() {
  String msg = "";
  msg += "Vfoc=";
  msg += String(Vfoc, 4);
  return msg;
}


void applyIfoc() {
  targetIqA = constrain(targetIqA, -Ifoc, Ifoc);
  targetIqB = constrain(targetIqB, -Ifoc, Ifoc);
  targetIqC = constrain(targetIqC, -Ifoc, Ifoc);

  if (modeA == AXIS_MODE_TORQUE) motorA.target = targetIqA;
  if (modeB == AXIS_MODE_TORQUE) motorB.target = targetIqB;
  if (modeC == AXIS_MODE_TORQUE) motorC.target = targetIqC;

  motorA.updateCurrentLimit(Ifoc);
  motorB.updateCurrentLimit(Ifoc);
  motorC.updateCurrentLimit(Ifoc);
}


void setIfoc(float value, uint8_t source) {
  Ifoc = constrain(value, 0.0f, FOC_CURRENT_LIMIT_MAX);
  applyIfoc();
  sendReply(source, "OK, IF=" + String(Ifoc, 4));
}


String getIfocInfo() {
  String msg = "Ifoc=";
  msg += String(Ifoc, 4);
  return msg;
}


static BLDCMotor* getAxisMotor(char axis) {
  if (axis == 'A') return &motorA;
  if (axis == 'B') return &motorB;
  if (axis == 'C') return &motorC;
  return nullptr;
}


static PIDController* getAxisPid(BLDCMotor& motor, char loopType) {
  if (loopType == 'V') return &motor.PID_velocity;
  if (loopType == 'P') return &motor.PID_angle;
  return nullptr;
}


static void makePidKey(char* key, char loopType, char axis, char gainType) {
  key[0] = loopType;
  key[1] = axis;
  key[2] = gainType;
  key[3] = '\0';
}


static bool ensurePidPrefsReady() {
  if (pidPrefsReady) return true;
  pidPrefsReady = pidPrefs.begin("pid", false);
  return pidPrefsReady;
}


static void loadPidValuesForAxis(char loopType, char axis) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return;

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return;

  char key[4];
  makePidKey(key, loopType, axis, 'P');
  pid->P = pidPrefs.getFloat(key, pid->P);
  makePidKey(key, loopType, axis, 'I');
  pid->I = pidPrefs.getFloat(key, pid->I);
  makePidKey(key, loopType, axis, 'D');
  pid->D = pidPrefs.getFloat(key, pid->D);
  pid->reset();
}


void loadPidValuesFromFlash() {
  if (!ensurePidPrefsReady()) {
    Serial.println("PID NVS open failed, using compiled defaults.");
    return;
  }

  loadPidValuesForAxis('V', 'A');
  loadPidValuesForAxis('V', 'B');
  loadPidValuesForAxis('V', 'C');
  loadPidValuesForAxis('P', 'A');
  loadPidValuesForAxis('P', 'B');
  loadPidValuesForAxis('P', 'C');
  Serial.println("PID values loaded from NVS.");
}


static bool savePidValuesForAxis(char loopType, char axis) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return false;

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return false;

  char key[4];
  makePidKey(key, loopType, axis, 'P');
  bool savedP = pidPrefs.putFloat(key, pid->P) == sizeof(float);
  makePidKey(key, loopType, axis, 'I');
  bool savedI = pidPrefs.putFloat(key, pid->I) == sizeof(float);
  makePidKey(key, loopType, axis, 'D');
  bool savedD = pidPrefs.putFloat(key, pid->D) == sizeof(float);
  return savedP && savedI && savedD;
}


static String getSinglePidInfo(char loopType, char axis) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return "ERR, UNKNOWN_AXIS";

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return "ERR, UNKNOWN_PID_LOOP";

  String msg = "";
  msg += loopType;
  msg += axis;
  msg += "_PID=";
  msg += String(pid->P, 4);
  msg += ",";
  msg += String(pid->I, 4);
  msg += ",";
  msg += String(pid->D, 4);
  return msg;
}


String getPidInfo(char loopType, char axis) {
  if (loopType != 'V' && loopType != 'P') {
    return "ERR, UNKNOWN_PID_LOOP";
  }

  if (axis == '\0') {
    String msg = getSinglePidInfo(loopType, 'A');
    msg += "\n";
    msg += getSinglePidInfo(loopType, 'B');
    msg += "\n";
    msg += getSinglePidInfo(loopType, 'C');
    return msg;
  }

  return getSinglePidInfo(loopType, axis);
}


static bool applyPidValuesToAxis(char loopType, char axis, float p, float i, float d) {
  BLDCMotor* motor = getAxisMotor(axis);
  if (motor == nullptr) return false;

  PIDController* pid = getAxisPid(*motor, loopType);
  if (pid == nullptr) return false;

  pid->P = p;
  pid->I = i;
  pid->D = d;

  // Clear the old integral/derivative history so the new gain starts cleanly.
  pid->reset();
  return true;
}


bool setPidValues(char loopType, char axis, float p, float i, float d, uint8_t source) {
  if ((loopType != 'V' && loopType != 'P') ||
      (axis != '\0' && axis != 'A' && axis != 'B' && axis != 'C')) {
    sendReply(source, "ERR, PID_FORMAT, USE_VPID<P,I,D>_OR_VAPID<P,I,D>");
    return false;
  }

  if (!isfinite(p) || !isfinite(i) || !isfinite(d) ||
      p < 0.0f || i < 0.0f || d < 0.0f) {
    sendReply(source, "ERR, PID_VALUE, USE_FINITE_NONNEGATIVE_VALUE");
    return false;
  }

  bool ok = true;
  if (axis == '\0') {
    ok = applyPidValuesToAxis(loopType, 'A', p, i, d) &&
         applyPidValuesToAxis(loopType, 'B', p, i, d) &&
         applyPidValuesToAxis(loopType, 'C', p, i, d);
  } else {
    ok = applyPidValuesToAxis(loopType, axis, p, i, d);
  }

  if (!ok) {
    sendReply(source, "ERR, PID_SET_FAILED");
    return false;
  }

  if (!ensurePidPrefsReady()) {
    sendReply(source, "ERR, PID_NVS_OPEN_FAILED, RAM_UPDATED_NOT_SAVED");
    return false;
  }

  bool saved = true;
  if (axis == '\0') {
    bool savedA = savePidValuesForAxis(loopType, 'A');
    bool savedB = savePidValuesForAxis(loopType, 'B');
    bool savedC = savePidValuesForAxis(loopType, 'C');
    saved = savedA && savedB && savedC;
  } else {
    saved = savePidValuesForAxis(loopType, axis);
  }

  if (!saved) {
    sendReply(source, "ERR, PID_NVS_WRITE_FAILED, RAM_UPDATED_NOT_FULLY_SAVED");
    return false;
  }

  String msg = "OK, ";
  msg += loopType;
  if (axis != '\0') {
    msg += axis;
  }
  msg += "_PID=";
  msg += String(p, 4);
  msg += ",";
  msg += String(i, 4);
  msg += ",";
  msg += String(d, 4);
  sendReply(source, msg);
  return true;
}


void sleepAxis(char axis) {
  if (axis == 'A') {
    targetVelA = 0.0f;
    targetIqA = 0.0f;
    motorA.disable();
    digitalWrite(PIN_A_SLEEP, LOW);
    sleepA = true;
  } else if (axis == 'B') {
    targetVelB = 0.0f;
    targetIqB = 0.0f;
    motorB.disable();
    digitalWrite(PIN_B_SLEEP, LOW);
    sleepB = true;
  } else if (axis == 'C') {
    targetVelC = 0.0f;
    targetIqC = 0.0f;
    motorC.disable();
    digitalWrite(PIN_C_SLEEP, LOW);
    sleepC = true;
  }
}


bool wakeAxis(char axis) {
  if (axis == 'A') {
    digitalWrite(PIN_A_SLEEP, HIGH);
    delay(10);
    applyVfoc();
    motorA.enable();
    if (motorA.motor_status != FOCMotorStatus::motor_ready) {
      int ok = motorA.initFOC();
      if (!ok) {
        motorA.disable();
        digitalWrite(PIN_A_SLEEP, LOW);
        sleepA = true;
        sendReply(0, "ERR, A_INITFOC_FAILED");
        return false;
      }
    }
    sleepA = false;
    return true;
  } else if (axis == 'B') {
    digitalWrite(PIN_B_SLEEP, HIGH);
    delay(10);
    applyVfoc();
    motorB.enable();
    if (motorB.motor_status != FOCMotorStatus::motor_ready) {
      int ok = motorB.initFOC();
      if (!ok) {
        motorB.disable();
        digitalWrite(PIN_B_SLEEP, LOW);
        sleepB = true;
        sendReply(0, "ERR, B_INITFOC_FAILED");
        return false;
      }
    }
    sleepB = false;
    return true;
  } else if (axis == 'C') {
    digitalWrite(PIN_C_SLEEP, HIGH);
    delay(10);
    applyVfoc();
    motorC.enable();
    if (motorC.motor_status != FOCMotorStatus::motor_ready) {
      int ok = motorC.initFOC();
      if (!ok) {
        motorC.disable();
        digitalWrite(PIN_C_SLEEP, LOW);
        sleepC = true;
        sendReply(0, "ERR, C_INITFOC_FAILED");
        return false;
      }
    }
    sleepC = false;
    return true;
  }
  return false;
}

void sleepAllAxis() {
  sleepAxis('A');
  sleepAxis('B');
  sleepAxis('C');
}


bool wakeAllAxis() {
  bool okA = wakeAxis('A');
  bool okB = wakeAxis('B');
  bool okC = wakeAxis('C');
  return okA && okB && okC;
}


String getAxisLimitText(bool upper, bool lower) {
  if (upper && lower) {
    return "BOTH";
  }

  if (upper) {
    return "UPPER";
  }

  if (lower) {
    return "LOWER";
  }

  return "NONE";
}


void applyPositionSoftLimit() {
  float posA = getRelativePosA();
  float posB = getRelativePosB();
  float posC = getRelativePosC();

  limitAUpper = posA >= A_MAX_POS;
  limitALower = posA <= A_MIN_POS;

  limitBUpper = posB >= B_MAX_POS;
  limitBLower = posB <= B_MIN_POS;

  limitCUpper = posC >= C_MAX_POS;
  limitCLower = posC <= C_MIN_POS;

  if (modeA == AXIS_MODE_VELOCITY) {
    if (limitAUpper && targetVelA > 0.0f) {
      targetVelA = 0.0f;
      motorA.target = 0.0f;
      broadcastMsg("A upper pos limit, A_target_vel=0");
    }

    if (limitALower && targetVelA < 0.0f) {
      targetVelA = 0.0f;
      motorA.target = 0.0f;
      broadcastMsg("A lower pos limit, A_target_vel=0");
    }
  }

  if (modeA == AXIS_MODE_TORQUE) {
    if ((limitAUpper && targetIqA > 0.0f) ||
        (limitALower && targetIqA < 0.0f)) {
      targetIqA = 0.0f;
      motorA.target = 0.0f;
      broadcastMsg("A pos limit, A_target_iq=0");
    }
  }

  if (modeB == AXIS_MODE_VELOCITY) {
    if (limitBUpper && targetVelB > 0.0f) {
      targetVelB = 0.0f;
      motorB.target = 0.0f;
      broadcastMsg("B upper pos limit, B_target_vel=0");
    }

    if (limitBLower && targetVelB < 0.0f) {
      targetVelB = 0.0f;
      motorB.target = 0.0f;
      broadcastMsg("B lower pos limit, B_target_vel=0");
    }
  }

  if (modeB == AXIS_MODE_TORQUE) {
    if ((limitBUpper && targetIqB > 0.0f) ||
        (limitBLower && targetIqB < 0.0f)) {
      targetIqB = 0.0f;
      motorB.target = 0.0f;
      broadcastMsg("B pos limit, B_target_iq=0");
    }
  }

  if (modeC == AXIS_MODE_VELOCITY) {
    if (limitCUpper && targetVelC > 0.0f) {
      targetVelC = 0.0f;
      motorC.target = 0.0f;
      broadcastMsg("C upper pos limit, C_target_vel=0");
    }

    if (limitCLower && targetVelC < 0.0f) {
      targetVelC = 0.0f;
      motorC.target = 0.0f;
      broadcastMsg("C lower pos limit, C_target_vel=0");
    }
  }

  if (modeC == AXIS_MODE_TORQUE) {
    if ((limitCUpper && targetIqC > 0.0f) ||
        (limitCLower && targetIqC < 0.0f)) {
      targetIqC = 0.0f;
      motorC.target = 0.0f;
      broadcastMsg("C pos limit, C_target_iq=0");
    }
  }
}


void setupDriver(BLDCDriver3PWM& driver) {
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver.voltage_limit = Vfoc;
  driver.pwm_frequency = PWM_FREQUENCY;
  driver.init();
}


bool setupMotor(BLDCMotor& motor,
                BLDCDriver3PWM& driver,
                Sensor& sensor,
                InlineCurrentSense& currentSense,
                float maxVel) {
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  currentSense.linkDriver(&driver);
  if (!currentSense.init()) {
    return false;
  }
  motor.linkCurrentSense(&currentSense);

  motor.voltage_limit = Vfoc;
  motor.velocity_limit = maxVel;

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.current_limit = Ifoc;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.PID_velocity.P = 0.10f;
  motor.PID_velocity.I = 0.20f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.limit = Ifoc;
  motor.PID_velocity.output_ramp = 100.0f;

  motor.PID_angle.P = 6.0f;
  motor.PID_angle.I = 0.0f;
  motor.PID_angle.D = 0.0f;
  motor.PID_angle.output_ramp = 100.0f;
  motor.PID_angle.limit = maxVel;

  motor.LPF_velocity.Tf = 0.02f;

  motor.PID_current_q.P = CURRENT_PID_P;
  motor.PID_current_q.I = CURRENT_PID_I;
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_q.limit = Vfoc;
  motor.PID_current_d.P = CURRENT_PID_P;
  motor.PID_current_d.I = CURRENT_PID_I;
  motor.PID_current_d.D = 0.0f;
  motor.PID_current_d.limit = Vfoc;
  motor.LPF_current_q.Tf = CURRENT_LPF_TF;
  motor.LPF_current_d.Tf = CURRENT_LPF_TF;

  motor.useMonitoring(Serial);

  int initOk = motor.init();
  int focOk = 0;
  if (initOk) {
    focOk = motor.initFOC();
  }
  return initOk && focOk;
}


void runAxisControl(BLDCMotor& motor,
                    bool axisSleep,
                    uint8_t axisMode,
                    float velocityTarget,
                    float absolutePositionTarget,
                    float torqueCurrentTarget) {
  if (axisSleep || systemProtected) {
    return;
  }

  motor.loopFOC();

  if (axisMode == AXIS_MODE_VELOCITY) {
    motor.move(velocityTarget);
  } else if (axisMode == AXIS_MODE_TORQUE) {
    motor.move(torqueCurrentTarget);
  } else {
    motor.move(absolutePositionTarget);
  }
}


void setAxisTorqueCurrent(char axis, float value, uint8_t source) {
  // FOC current mode: move() receives the requested q-axis current in A.
  float iq = constrain(value, -Ifoc, Ifoc);

  if (axis == 'A') {
    modeA = AXIS_MODE_TORQUE;
    motorA.updateCurrentLimit(Ifoc);
    motorA.updateTorqueControlType(TorqueControlType::foc_current);
    motorA.updateMotionControlType(MotionControlType::torque);
    targetIqA = iq;
    motorA.target = targetIqA;
    sendReply(source, "OK, TA=" + String(targetIqA, 4));
  } else if (axis == 'B') {
    modeB = AXIS_MODE_TORQUE;
    motorB.updateCurrentLimit(Ifoc);
    motorB.updateTorqueControlType(TorqueControlType::foc_current);
    motorB.updateMotionControlType(MotionControlType::torque);
    targetIqB = iq;
    motorB.target = targetIqB;
    sendReply(source, "OK, TB=" + String(targetIqB, 4));
  } else if (axis == 'C') {
    modeC = AXIS_MODE_TORQUE;
    motorC.updateCurrentLimit(Ifoc);
    motorC.updateTorqueControlType(TorqueControlType::foc_current);
    motorC.updateMotionControlType(MotionControlType::torque);
    targetIqC = iq;
    motorC.target = targetIqC;
    sendReply(source, "OK, TC=" + String(targetIqC, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


void setAxisVelocity(char axis, float value, uint8_t source) {
  if (axis == 'A') {
    modeA = AXIS_MODE_VELOCITY;
    motorA.updateTorqueControlType(TorqueControlType::foc_current);
    motorA.controller = MotionControlType::velocity;
    targetIqA = 0.0f;
    targetVelA = constrain(value, -A_MAX_VEL, A_MAX_VEL);
    motorA.target = targetVelA;
    sendReply(source, "OK, VA=" + String(targetVelA, 4));
  } else if (axis == 'B') {
    modeB = AXIS_MODE_VELOCITY;
    motorB.updateTorqueControlType(TorqueControlType::foc_current);
    motorB.controller = MotionControlType::velocity;
    targetIqB = 0.0f;
    targetVelB = constrain(value, -B_MAX_VEL, B_MAX_VEL);
    motorB.target = targetVelB;
    sendReply(source, "OK, VB=" + String(targetVelB, 4));
  } else if (axis == 'C') {
    modeC = AXIS_MODE_VELOCITY;
    motorC.updateTorqueControlType(TorqueControlType::foc_current);
    motorC.controller = MotionControlType::velocity;
    targetIqC = 0.0f;
    targetVelC = constrain(value, -C_MAX_VEL, C_MAX_VEL);
    motorC.target = targetVelC;
    sendReply(source, "OK, VC=" + String(targetVelC, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


void setAxisPosition(char axis, float value, uint8_t source) {
  if (axis == 'A') {
    modeA = AXIS_MODE_POSITION;
    motorA.updateTorqueControlType(TorqueControlType::foc_current);
    motorA.controller = MotionControlType::angle;
    targetIqA = 0.0f;
    targetPosA = constrain(value, A_MIN_POS, A_MAX_POS);
    motorA.target = getAbsoluteTargetA(targetPosA);
    sendReply(source, "OK, PA=" + String(targetPosA, 4));
  } else if (axis == 'B') {
    modeB = AXIS_MODE_POSITION;
    motorB.updateTorqueControlType(TorqueControlType::foc_current);
    motorB.controller = MotionControlType::angle;
    targetIqB = 0.0f;
    targetPosB = constrain(value, B_MIN_POS, B_MAX_POS);
    motorB.target = getAbsoluteTargetB(targetPosB);
    sendReply(source, "OK, PB=" + String(targetPosB, 4));
  } else if (axis == 'C') {
    modeC = AXIS_MODE_POSITION;
    motorC.updateTorqueControlType(TorqueControlType::foc_current);
    motorC.controller = MotionControlType::angle;
    targetIqC = 0.0f;
    targetPosC = constrain(value, C_MIN_POS, C_MAX_POS);
    motorC.target = getAbsoluteTargetC(targetPosC);
    sendReply(source, "OK, PC=" + String(targetPosC, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


void setAxisDisplacement(char axis, float value, uint8_t source) {
  if (axis == 'A') {
    modeA = AXIS_MODE_POSITION;
    motorA.updateTorqueControlType(TorqueControlType::foc_current);
    motorA.controller = MotionControlType::angle;
    targetIqA = 0.0f;
    targetPosA = constrain(getRelativePosA() + value, A_MIN_POS, A_MAX_POS);
    motorA.target = getAbsoluteTargetA(targetPosA);
    sendReply(source, "OK, PA=" + String(targetPosA, 4));
  } else if (axis == 'B') {
    modeB = AXIS_MODE_POSITION;
    motorB.updateTorqueControlType(TorqueControlType::foc_current);
    motorB.controller = MotionControlType::angle;
    targetIqB = 0.0f;
    targetPosB = constrain(getRelativePosB() + value, B_MIN_POS, B_MAX_POS);
    motorB.target = getAbsoluteTargetB(targetPosB);
    sendReply(source, "OK, PB=" + String(targetPosB, 4));
  } else if (axis == 'C') {
    modeC = AXIS_MODE_POSITION;
    motorC.updateTorqueControlType(TorqueControlType::foc_current);
    motorC.controller = MotionControlType::angle;
    targetIqC = 0.0f;
    targetPosC = constrain(getRelativePosC() + value, C_MIN_POS, C_MAX_POS);
    motorC.target = getAbsoluteTargetC(targetPosC);
    sendReply(source, "OK, PC=" + String(targetPosC, 4));
  } else {
    sendReply(source, "ERR, UNKNOWN_AXIS");
  }
}


void allTargetZero(uint8_t source) {
  targetPosA = 0.0f;
  targetPosB = 0.0f;
  targetPosC = 0.0f;
  targetIqA = 0.0f;
  targetIqB = 0.0f;
  targetIqC = 0.0f;

  modeA = AXIS_MODE_POSITION;
  modeB = AXIS_MODE_POSITION;
  modeC = AXIS_MODE_POSITION;

  motorA.updateTorqueControlType(TorqueControlType::foc_current);
  motorB.updateTorqueControlType(TorqueControlType::foc_current);
  motorC.updateTorqueControlType(TorqueControlType::foc_current);

  motorA.controller = MotionControlType::angle;
  motorB.controller = MotionControlType::angle;
  motorC.controller = MotionControlType::angle;

  motorA.target = getAbsoluteTargetA(0.0f);
  motorB.target = getAbsoluteTargetB(0.0f);
  motorC.target = getAbsoluteTargetC(0.0f);

  sendReply(source, "OK, ALL0");
}
