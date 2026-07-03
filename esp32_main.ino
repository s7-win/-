#include <Arduino.h>

// ===================== 引脚定义 =====================
#define PIR_PIN        27
#define BUZZER_PIN     26
#define CAM_POWER_PIN  25

#define FSR_PIN        34   // ADC1
#define TRIG_PIN       18
#define ECHO_PIN       19

#define CAM_RX_PIN     16   // ESP32 RX2，接 ESP32-CAM GPIO1 / U0T
#define CAM_TX_PIN     17   // ESP32 TX2，接 ESP32-CAM GPIO3 / U0R

// ===================== CAM 电源控制 =====================
// 调试阶段如果 ESP32-CAM 直接常供电，改成 0
#define USE_CAM_POWER_SWITCH  0

// 成品 MOS 电源开关模块通常 HIGH 打开，LOW 关闭
// 如果你用的是 P-MOS 高边开关，可能需要改成 LOW 打开
#define CAM_POWER_ON_LEVEL    HIGH
#define CAM_POWER_OFF_LEVEL   LOW

// ===================== FSR402 参数 =====================
// 需要根据你的实际串口输出调整
// 压力越大，ADC 越大
#define FSR_PRESS_THRESHOLD   1200

// ===================== 超声波判断参数 =====================
const float ARRIVE_DISTANCE_CM = 20.0;
const float REMOVE_DISTANCE_CM = 50.0;

// ===================== 业务流程时间参数 =====================
const unsigned long PREDETECT_WAIT_MS = 5000;
const unsigned long STABLE_WAIT_MS    = 5000;
const unsigned long SAMPLE_INTERVAL_MS = 100;

const int ARRIVE_TOTAL_SAMPLES = 30;
const int ARRIVE_VALID_COUNT   = 20;

const int REMOVE_TOTAL_SAMPLES = 20;
const int REMOVE_VALID_COUNT   = 15;

// 离开检测最长等待时间。
// 0 表示一直等待包裹被取走。
// 课程演示时可以改成 300000，即 5 分钟。
const unsigned long REMOVE_MAX_WAIT_MS = 0;

// PIR 触发冷却时间，避免重复触发
const unsigned long PIR_COOLDOWN_MS = 5000;

HardwareSerial CamSerial(2);

bool camSerialStarted = false;
bool camReadyOnce = false;
unsigned long lastPirTriggerTime = 0;

struct DetectResult {
  bool success;
  int validCount;
  int totalCount;
  int fsrValue;
  float distance;
};

// ===================== 基础函数 =====================
void beepDoorbell() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void camUartBegin() {
  if (!camSerialStarted) {
    CamSerial.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
    camSerialStarted = true;
  }
}

void camUartEnd() {
  CamSerial.end();
  camSerialStarted = false;

  // 防止 ESP32-CAM 断电后被 UART 反向供电
  pinMode(CAM_TX_PIN, INPUT);
  pinMode(CAM_RX_PIN, INPUT);
}

void camPowerOn() {
#if USE_CAM_POWER_SWITCH
  digitalWrite(CAM_POWER_PIN, CAM_POWER_ON_LEVEL);
  delay(3000);
#endif
  camUartBegin();
}

void camPowerOff() {
  camUartEnd();
#if USE_CAM_POWER_SWITCH
  digitalWrite(CAM_POWER_PIN, CAM_POWER_OFF_LEVEL);
#endif
}

String readCamLine(unsigned long timeoutMs) {
  String line = "";
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (CamSerial.available()) {
      char c = CamSerial.read();

      if (c == '\n') {
        line.trim();
        return line;
      } else if (c != '\r') {
        line += c;
      }
    }
    delay(10);
  }

  line.trim();
  return line;
}

bool waitForCamReady(unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    String msg = readCamLine(1000);

    if (msg.length() > 0) {
      Serial.print("[CAM] ");
      Serial.println(msg);

      if (msg == "CAM_READY") {
        return true;
      }
    }
  }

  return false;
}

bool sendCamCommandWaitAck(const String& cmd, const String& expectedAck, unsigned long timeoutMs) {
  Serial.print("[ESP32 -> CAM] ");
  Serial.println(cmd);

  CamSerial.println(cmd);

  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    String msg = readCamLine(1000);

    if (msg.length() > 0) {
      Serial.print("[CAM] ");
      Serial.println(msg);

      if (msg == expectedAck) {
        return true;
      }

      if (msg.startsWith("ERR:")) {
        return false;
      }
    }
  }

  return false;
}

void shutdownCamSession() {
  sendCamCommandWaitAck("CMD:SHUTDOWN", "CAM_SLEEP_OK", 5000);
  delay(300);

#if USE_CAM_POWER_SWITCH
  camPowerOff();
  camReadyOnce = false;
#else
  // 调试阶段 ESP32-CAM 常供电，不关闭串口，不等待下一次 CAM_READY。
  // 后续再次调用 startCamSession() 时使用 CMD:PING 确认 ESP32-CAM 仍在线。
#endif
}

bool startCamSession() {
  camPowerOn();

#if USE_CAM_POWER_SWITCH
  Serial.println("[CAM] 电源控制模式：等待 ESP32-CAM 启动...");
  bool ready = waitForCamReady(25000);
  if (!ready) {
    Serial.println("[ERROR] ESP32-CAM 未返回 CAM_READY");
    camPowerOff();
    return false;
  }
  camReadyOnce = true;
  return true;
#else
  if (!camReadyOnce) {
    Serial.println("[CAM] 常供电调试模式：首次等待 CAM_READY...");
    bool ready = waitForCamReady(30000);
    if (ready) {
      camReadyOnce = true;
      return true;
    }
    Serial.println("[WARN] 未等到 CAM_READY，尝试 CMD:PING...");
  }

  bool pingOK = sendCamCommandWaitAck("CMD:PING", "ACK:PING", 3000);
  if (pingOK) {
    camReadyOnce = true;
    return true;
  }

  Serial.println("[ERROR] ESP32-CAM PING 无响应，请检查 UART 接线或重启 ESP32-CAM");
  return false;
#endif
}

// ===================== 传感器读取 =====================
int readFSR() {
  int value = analogRead(FSR_PIN);
  return value;
}

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return 999.0;
  }

  float distance = duration * 0.0343 / 2.0;
  return distance;
}

bool pirStableTriggered() {
  if (digitalRead(PIR_PIN) == HIGH) {
    delay(120);
    return digitalRead(PIR_PIN) == HIGH;
  }
  return false;
}

bool packagePresentOnce(int fsrValue, float distance) {
  return fsrValue >= FSR_PRESS_THRESHOLD && distance < ARRIVE_DISTANCE_CM;
}

bool packageRemovedOnce(int fsrValue, float distance) {
  return fsrValue < FSR_PRESS_THRESHOLD && distance > REMOVE_DISTANCE_CM;
}

// ===================== 到达检测窗口 =====================
DetectResult detectPackageArrived() {
  DetectResult result;
  result.success = false;
  result.validCount = 0;
  result.totalCount = ARRIVE_TOTAL_SAMPLES;
  result.fsrValue = 0;
  result.distance = 999.0;

  for (int i = 0; i < ARRIVE_TOTAL_SAMPLES; i++) {
    int fsr = readFSR();
    float dist = readDistanceCM();

    result.fsrValue = fsr;
    result.distance = dist;

    if (packagePresentOnce(fsr, dist)) {
      result.validCount++;
    }

    Serial.print("[ARRIVE] ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(ARRIVE_TOTAL_SAMPLES);
    Serial.print(" FSR=");
    Serial.print(fsr);
    Serial.print(" DIST=");
    Serial.print(dist);
    Serial.print("cm VALID=");
    Serial.println(result.validCount);

    delay(SAMPLE_INTERVAL_MS);
  }

  result.success = result.validCount >= ARRIVE_VALID_COUNT;
  return result;
}

// ===================== 离开检测窗口 =====================
DetectResult detectPackageRemoved() {
  DetectResult result;
  result.success = false;
  result.validCount = 0;
  result.totalCount = REMOVE_TOTAL_SAMPLES;
  result.fsrValue = 0;
  result.distance = 999.0;

  for (int i = 0; i < REMOVE_TOTAL_SAMPLES; i++) {
    int fsr = readFSR();
    float dist = readDistanceCM();

    result.fsrValue = fsr;
    result.distance = dist;

    if (packageRemovedOnce(fsr, dist)) {
      result.validCount++;
    }

    Serial.print("[REMOVE] ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(REMOVE_TOTAL_SAMPLES);
    Serial.print(" FSR=");
    Serial.print(fsr);
    Serial.print(" DIST=");
    Serial.print(dist);
    Serial.print("cm VALID=");
    Serial.println(result.validCount);

    delay(SAMPLE_INTERVAL_MS);
  }

  result.success = result.validCount >= REMOVE_VALID_COUNT;
  return result;
}

String makeArrivedEvent(const DetectResult& r) {
  String cmd = "EVENT:ARRIVED";
  cmd += ",FSR=" + String(r.fsrValue);
  cmd += ",DIST=" + String(r.distance, 1);
  cmd += ",VALID=" + String(r.validCount) + "/" + String(r.totalCount);
  return cmd;
}

String makeRemovedEvent(const DetectResult& r) {
  String cmd = "EVENT:REMOVED";
  cmd += ",FSR=" + String(r.fsrValue);
  cmd += ",DIST=" + String(r.distance, 1);
  cmd += ",VALID=" + String(r.validCount) + "/" + String(r.totalCount);
  return cmd;
}

String makeFalseTriggerEvent(const DetectResult& r) {
  String cmd = "EVENT:FALSE_TRIGGER";
  cmd += ",VALID=" + String(r.validCount) + "/" + String(r.totalCount);
  return cmd;
}

// ===================== 主流程 =====================
void runOneBusinessFlow() {
  Serial.println();
  Serial.println("========== PIR 触发，进入业务流程 ==========");

  beepDoorbell();

  if (!startCamSession()) {
    Serial.println("[WARN] CAM 启动失败，主控仍继续执行传感器流程");
  } else {
    sendCamCommandWaitAck("EVENT:DOORBELL", "ACK:DOORBELL", 5000);
  }

  Serial.println("[STATE] 5秒预稳定等待");
  delay(PREDETECT_WAIT_MS);

  Serial.println("[STATE] 3秒包裹到达检测窗口");
  DetectResult arriveResult = detectPackageArrived();

  if (!arriveResult.success) {
    Serial.println("[RESULT] 未检测到包裹，判定为误触发");

    if (CamSerial) {
      String cmd = makeFalseTriggerEvent(arriveResult);
      sendCamCommandWaitAck(cmd, "ACK:FALSE_TRIGGER", 5000);
      shutdownCamSession();
    }

    Serial.println("[STATE] 返回待机");
    return;
  }

  Serial.println("[RESULT] 包裹到达有效，进入5秒稳定确认期");
  delay(STABLE_WAIT_MS);

  Serial.println("[STATE] 通知 ESP32-CAM 拍照并上报 ARRIVED");
  if (CamSerial) {
    String cmd = makeArrivedEvent(arriveResult);
    sendCamCommandWaitAck(cmd, "ACK:ARRIVED", 60000);
    shutdownCamSession();
  }

  Serial.println("[STATE] ESP32 主控进入包裹离开检测");

  unsigned long removeStart = millis();

  while (true) {
    DetectResult removeResult = detectPackageRemoved();

    if (removeResult.success) {
      Serial.println("[RESULT] 包裹已被取走");

      if (startCamSession()) {
        String cmd = makeRemovedEvent(removeResult);
        sendCamCommandWaitAck(cmd, "ACK:REMOVED", 60000);
        shutdownCamSession();
      }

      Serial.println("[STATE] 一次完整流程结束，返回待机");
      return;
    }

    Serial.println("[RESULT] 包裹仍在，继续检测");

    if (REMOVE_MAX_WAIT_MS > 0 && millis() - removeStart > REMOVE_MAX_WAIT_MS) {
      Serial.println("[TIMEOUT] 离开检测超时，返回待机");
      return;
    }

    delay(500);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CAM_POWER_PIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(TRIG_PIN, LOW);

#if USE_CAM_POWER_SWITCH
  digitalWrite(CAM_POWER_PIN, CAM_POWER_OFF_LEVEL);
#endif

  analogReadResolution(12);
  analogSetPinAttenuation(FSR_PIN, ADC_11db);

  Serial.println("ESP32 主控启动完成");
  Serial.println("等待 PIR 触发...");
  Serial.println("提示：先观察无压力/有压力时 FSR ADC 值，再调整 FSR_PRESS_THRESHOLD");
}

void loop() {
  if (millis() - lastPirTriggerTime < PIR_COOLDOWN_MS) {
    delay(50);
    return;
  }

  if (pirStableTriggered()) {
    lastPirTriggerTime = millis();
    runOneBusinessFlow();
  }

  delay(100);
}