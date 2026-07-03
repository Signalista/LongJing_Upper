#include "Globals.h"

// ============================================================
// Timestamp 时间函数
//
// 未同步时：
//   Timestamp = ESP32 上电运行时间 millis()
//
// 同步后：
//   Timestamp = millis() + timeOffsetMs
//
// 上位机发送 TIME<HHMMSS> 后：
//   timeOffsetMs = HH:MM:SS 当天毫秒数 - millis()
//
// 输出格式：
//   【Timestamp=00:00:00.000】
//
// 注意：
//   msToTimestamp() 只显示一天内的 HH:MM:SS.mmm，超过 24h 会循环。
//   如果上位机想显示本地时间，需要发送“已经加过时区偏移的 ms”。
// ============================================================

bool timeSynced = false;
int64_t timeOffsetMs = 0;


String msToTimestamp(uint64_t ms) {
  uint32_t totalMs = ms % 86400000ULL;

  uint32_t hh = totalMs / 3600000UL;
  totalMs %= 3600000UL;

  uint32_t mm = totalMs / 60000UL;
  totalMs %= 60000UL;

  uint32_t ss = totalMs / 1000UL;
  uint32_t mmm = totalMs % 1000UL;

  char buf[20];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu.%03lu",
           (unsigned long)hh,
           (unsigned long)mm,
           (unsigned long)ss,
           (unsigned long)mmm);

  return String(buf);
}


uint64_t getCorrectedTimeMs() {
  uint64_t nowMs = (uint64_t)millis();

  if (!timeSynced) {
    return nowMs;
  }

  int64_t corrected = (int64_t)nowMs + timeOffsetMs;

  if (corrected < 0) {
    return 0;
  }

  return (uint64_t)corrected;
}


String getTimestampText() {
  return msToTimestamp(getCorrectedTimeMs());
}


// 前面自带 \n，防止和 Running 后面的 .......... 粘在一起

String getTimestampHeader() {
  if (IF_TIMESTAMP == 1)
  {
    String msg = "";
    msg += "\n\n【Timestamp=";
    msg += getTimestampText();
    msg += "】\n";
    return msg;
  }
  else
  {
    String msg = "\n\n";
    return msg;
  }
}


void syncHostTimeMs(uint64_t hostTimeMs) {
  uint64_t nowMs = (uint64_t)millis();
  timeOffsetMs = (int64_t)hostTimeMs - (int64_t)nowMs;
  timeSynced = true;
}


void syncHostTimeHms(uint8_t hh, uint8_t mm, uint8_t ss) {
  uint64_t hostTimeMs = ((uint64_t)hh * 3600ULL +
                         (uint64_t)mm * 60ULL +
                         (uint64_t)ss) * 1000ULL;
  syncHostTimeMs(hostTimeMs);
}


String getTimeInfo() {
  String msg = "";

  msg += "Time_sync=";
  msg += (timeSynced ? "YES" : "NO");

  msg += ", Time_offset_ms=";
  msg += String((long long)timeOffsetMs);

  msg += ", Time_millis=";
  msg += String((unsigned long)millis());

  msg += ", Time_corrected_ms=";
  msg += String((unsigned long long)getCorrectedTimeMs());

  return msg;
}
