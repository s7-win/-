// =============================
// 包裹投递监控系统（修正采样计数）
// 7状态 + 3层防抖 + 双窗口投票
// =============================
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);

#define PIR_PIN     13
#define FSR_PIN     34
#define TRIG_PIN    4
#define ECHO_PIN    2
#define BUZZER_PIN  14

// =============================
// 阈值定义
// =============================
#define FSR_ARRIVE_THRESHOLD   800
#define FSR_REMOVE_THRESHOLD    100
#define DIST_ARRIVE_THRESHOLD   20
#define DIST_REMOVE_THRESHOLD   50

// =============================
// 状态机枚举
// =============================
enum State {
  STATE_SLEEP,
  STATE_DOORBELL,
  STATE_WAIT_PREDETECT,
  STATE_DETECT_WINDOW,
  STATE_PACKAGE_STABLE,
  STATE_PACKAGE_PRESENT,
  STATE_REMOVE_CHECK
};

State state = STATE_SLEEP;

// =============================
// 时间控制变量
// =============================
unsigned long waitStart = 0;
unsigned long stableStart = 0;
unsigned long detectStart = 0;
unsigned long removeStart = 0;

bool detectInit = false;
bool removeInit = false;

int arriveCount = 0;
int removeCount = 0;

// 采样控制
unsigned long sampleTick = 0;
unsigned long removeSampleTick = 0;

unsigned long t1 = 0;
unsigned long t2 = 0;

int fsrValue = 0;
long distanceCM = 999;

// =============================
// 状态名称
// =============================
const char* getStateName(State s) {
  switch (s) {
    case STATE_SLEEP:           return "SLEEP(0)";
    case STATE_DOORBELL:        return "DOORBELL(1)";
    case STATE_WAIT_PREDETECT:  return "WAIT_5S(2)";
    case STATE_DETECT_WINDOW:   return "DETECT(3)";
    case STATE_PACKAGE_STABLE:  return "STABLE(4)";
    case STATE_PACKAGE_PRESENT: return "PRESENT(5)";
    case STATE_REMOVE_CHECK:    return "REMOVE(6)";
    default: return "UNKNOWN";
  }
}

// =============================
// 传感器函数
// =============================
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

void reconnect() {
  while (!client.connected()) {
    client.connect("porch_monitor");
  }
}

void beepOnce() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(120);
  digitalWrite(BUZZER_PIN, LOW);
}

void publishDoorbell() {
  client.publish("porch/doorbell", "TRIGGERED");
}

void publishArrived() {
  client.publish("porch/package", "ARRIVED");
client.publish("porch/image", "image_url");
}

void publishRemoved() {
  client.publish("porch/package", "REMOVED");
}

// =============================
void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  pinMode(FSR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  WiFi.begin(ssid, password);

while (WiFi.status() != WL_CONNECTED) {
  delay(500);
}

client.setServer(mqtt_server, 1883);


  Serial.println("========================================");
  Serial.println("📦 包裹投递监控系统（修正版）");
  Serial.println("7状态 | 3层防抖 | 双窗口投票");
  Serial.println("========================================");
  Serial.println("当前状态: SLEEP(0) - 等待PIR触发");
}

// =============================
void loop() {
  if (!client.connected()) reconnect();
client.loop();
  unsigned long now = millis();
  int pir = digitalRead(PIR_PIN);

  // 低频采样
  if (state != STATE_SLEEP) {
    if (now - t1 > 100) {
      t1 = now;
      fsrValue = analogRead(FSR_PIN);
    }
    if (now - t2 > 300) {
      t2 = now;
      distanceCM = readDistanceCM();
    }
  }

  // =============================
  switch (state) {

    case STATE_SLEEP:
      if (pir == HIGH) {
        Serial.println("\n[PIR] 检测到人体活动");
        state = STATE_DOORBELL;
      }
      break;

    case STATE_DOORBELL:
      Serial.println("[状态1] 门铃触发");
      beepOnce();
      publishDoorbell();
      waitStart = now;
      detectInit = false;
      removeInit = false;
      state = STATE_WAIT_PREDETECT;
      break;

    case STATE_WAIT_PREDETECT:
      if (now - waitStart >= 5000) {
        Serial.println("[状态2] 预稳定完成，进入到达判定");
        detectStart = now;
        arriveCount = 0;
        sampleTick = 0;
        detectInit = true;
        state = STATE_DETECT_WINDOW;
      }
      break;

    case STATE_DETECT_WINDOW:
      if (now - detectStart <= 3000) {
        if (now - sampleTick >= 100) {
          sampleTick = now;
          if (fsrValue > FSR_ARRIVE_THRESHOLD &&
              distanceCM < DIST_ARRIVE_THRESHOLD) {
            arriveCount++;
          }
        }
      } else {
        Serial.print("[状态3] 到达命中: ");
        Serial.print(arriveCount);
        Serial.print("/30");
        if (arriveCount >= 20) {
          Serial.println(" ✅ 判定包裹到达");
          stableStart = 0;
          state = STATE_PACKAGE_STABLE;
        } else {
          Serial.println(" ❌ 未检测到包裹，返回休眠");
          state = STATE_SLEEP;
        }
        detectInit = false;
        sampleTick = 0;
      }
      break;

    case STATE_PACKAGE_STABLE:
      if (stableStart == 0) {
        stableStart = now;
        Serial.println("[状态4] 进入5秒稳定确认期...");
      }
      if (now - stableStart >= 5000) {
        Serial.println("[状态4] 稳定确认完成");
        stableStart = 0;
        state = STATE_PACKAGE_PRESENT;
      }
      break;

    case STATE_PACKAGE_PRESENT:
      Serial.println("[状态5] 包裹确认，执行上报");
      publishArrived();
      state = STATE_REMOVE_CHECK;
      break;

    case STATE_REMOVE_CHECK:
      if (!removeInit) {
        removeStart = now;
        removeCount = 0;
        removeSampleTick = 0;
        removeInit = true;
        Serial.println("[状态6] 进入2秒离开检测窗口...");
      }

      // 窗口内每100ms采样一次
      if (now - removeStart <= 2000) {
        if (now - removeSampleTick >= 100) {
          removeSampleTick = now;
          if (fsrValue < FSR_REMOVE_THRESHOLD &&
              distanceCM > DIST_REMOVE_THRESHOLD) {
            removeCount++;
          }
        }
      } else {
        // 窗口结束，判决
        Serial.print("[状态6] 离开命中: ");
        Serial.print(removeCount);
        Serial.print("/20");
        if (removeCount >= 15) {
          Serial.println(" ✅ 确认包裹被取走");
          publishRemoved();
          state = STATE_SLEEP;
        } else {
          Serial.println(" ❌ 误触发，保持包裹存在状态");
          // 为避免重复上报，直接重新开始离开检测窗口
          state = STATE_REMOVE_CHECK;
          removeInit = false;   // 重置，以便重新初始化窗口
        }
        removeInit = false;
        removeCount = 0;
      }
      break;
  }

  delay(30);
}