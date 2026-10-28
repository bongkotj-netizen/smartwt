#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Same status LED pins as the original Smart Water sketch.
#define LED_RED_PIN 17
#define LED_GREEN_PIN 16
#define BUZZER_PIN 32 // Original active-LOW buzzer

// ===================== 1. SETTINGS ========================
namespace Config {
  const char *const WIFI_NAME = "OPPO";
  const char *const WIFI_PASS = "1234567890";
  const char *const MQTT_HOST = "7e0ad6dbeb414d54ae02b3a3edd1c48c.s1.eu.hivemq.cloud";
  const char *const MQTT_USER = "yamaroi";
  const char *const MQTT_PASS = "123456789";
  constexpr uint16_t MQTT_PORT = 8883;
  const char *const BOT_TOKEN = "8672230296:AAGPfkdcXZaJKJUnQg5dSNi-VHa1b83JLlM";
  const char *const CHAT_ID   = "8656967260";

  constexpr uint8_t LEAK1 = 33, LEAK2 = 25, SOIL = 34, FLOW = 26, RELAY = 13;
  constexpr bool RELAY_ACTIVE_LOW = false; // Old sketch: HIGH = ON
  constexpr uint8_t RELAY_ON  = RELAY_ACTIVE_LOW ? LOW : HIGH;
  constexpr uint8_t RELAY_OFF = RELAY_ACTIVE_LOW ? HIGH : LOW;
  // Telegram: gap between requests + repeated-event suppression.
  constexpr uint32_t TG_SEND_GAP_MS = 6000UL;     // 6s between Telegram requests
  constexpr uint32_t TG_REPEAT_MS = 60000UL;      // same event <= 1/min
  constexpr uint32_t TG_LEAK_REPEAT_MS = 30000UL; // leak <= 1/30s
  constexpr uint32_t TG_FORECAST_REPEAT_MS = 1800000UL; // forecast <= 1/30min

  constexpr float PULSES_PER_LITER = 450.0f;
  constexpr uint32_t MAX_AUTO_MS = 300000UL; // 5 minutes of AUTO/day
  constexpr uint32_t COOLDOWN_MS = 5000UL;  // 5 sec AFTER valve closes before next AUTO open
  constexpr uint32_t BUZZER_BEEP_MS = 150UL;  // short beep from old sketch
  constexpr uint32_t BUZZER_ALARM_MS = 5000UL; // 5-second leak alarm
  constexpr uint32_t SAMPLE_MS = 1000UL; // current readings every 1s
  constexpr uint32_t HISTORY_MS = 60000UL; // CSV history record every 10s
}

struct Crop { const char *name; int dryADC; float maxLiters; };
Crop crops[] = {
  {"LETTUCE", 2200, 0.35f},
  {"TOMATO",  2500, 0.50f},
  {"CHILI",   2800, 0.40f}
};
constexpr uint8_t CROP_COUNT = sizeof(crops) / sizeof(crops[0]);

// ===================== 2. SENSOR CLASS ====================
volatile uint32_t pulseCounter = 0;
void IRAM_ATTR flowISR() { ++pulseCounter; }

class Sensors {
public:
  void begin() {
    pinMode(Config::LEAK1, INPUT_PULLUP);
    pinMode(Config::LEAK2, INPUT_PULLUP);
    pinMode(Config::SOIL, INPUT);
    pinMode(Config::FLOW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Config::FLOW), flowISR, FALLING);
  }
  bool leaking() const { // Both sensors LOW = water detected.
    return digitalRead(Config::LEAK1) == LOW && digitalRead(Config::LEAK2) == LOW;
  }
  int soil() const {
    int sum = 0;
    for (int i = 0; i < 5; ++i) { sum += analogRead(Config::SOIL); delay(2); }
    return sum / 5;
  }
  float readLiters() const {
    noInterrupts();
    uint32_t count = pulseCounter;
    pulseCounter = 0;
    interrupts();
    return count / Config::PULSES_PER_LITER;
  }
  void resetFlow() const {
    noInterrupts(); pulseCounter = 0; interrupts();
  }
};

// ===================== 3. VALVE CLASS =====================
class Valve {
  bool on_ = false;
public:
  void begin() {
    digitalWrite(Config::RELAY, Config::RELAY_OFF); // OFF before OUTPUT.
    pinMode(Config::RELAY, OUTPUT);
    close();
  }
  void open()  { digitalWrite(Config::RELAY, Config::RELAY_ON);  on_ = true; }
  void close() { digitalWrite(Config::RELAY, Config::RELAY_OFF); on_ = false; }
  bool isOpen() const { return on_; }
  const char *gpio() const { return digitalRead(Config::RELAY) ? "HIGH" : "LOW"; }
};

// ===================== STATUS LED CLASS =====================
// Original LED behavior (GPIO17 RED, GPIO16 GREEN):
// Normal: BOTH leak sensors wet -> RED, otherwise GREEN.
// Valve changes: blink GREEN on opening / RED on closing for 5 sec.
// Soil becomes wet: blink GREEN 5 times (250 ms per half-cycle).
// Non-blocking: millis() keeps leak detection and MQTT responsive.
class StatusLED {
  bool flashActive_ = false, flashOn_ = false, flashValveOpen_ = false;
  bool soilBlink_ = false, soilOn_ = false;
  uint8_t soilBlinks_ = 0;
  uint32_t flashStart_ = 0, flashLast_ = 0, soilLast_ = 0;

  void show(bool green, bool red) const {
    digitalWrite(LED_GREEN_PIN, green ? HIGH : LOW);
    digitalWrite(LED_RED_PIN, red ? HIGH : LOW);
  }

public:
  void begin(bool leakDetected) {
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    show(!leakDetected, leakDetected);
  }

  void valveChanged(bool open) {
    flashActive_ = flashOn_ = true;
    flashValveOpen_ = open;
    flashStart_ = flashLast_ = millis();
    show(open, !open);
  }

  void soilBecameWet() {
    soilBlink_ = soilOn_ = true;
    soilBlinks_ = 0;
    soilLast_ = millis();
    show(true, false);
  }

  void update(uint32_t now, bool leakDetected) {
    if (flashActive_) {
      if (now - flashStart_ >= 5000UL) flashActive_ = false;
      else if (now - flashLast_ >= 300UL) {
        flashLast_ = now;
        flashOn_ = !flashOn_;
      }
    }
    if (soilBlink_) {
      if (now - soilLast_ >= 250UL) {
        soilLast_ = now;
        soilOn_ = !soilOn_;
        if (!soilOn_ && ++soilBlinks_ >= 5) soilBlink_ = false;
      }
      if (soilBlink_) { show(soilOn_, false); return; }
    }
    if (flashActive_) {
      show(flashOn_ && flashValveOpen_, flashOn_ && !flashValveOpen_);
      return;
    }
    show(!leakDetected, leakDetected);
  }
};

// ===================== 4. BUZZER CLASS =====================
// Original active-LOW buzzer (GPIO32): LOW = sound, HIGH = silent.
// Valve ON/OFF or soil WET: short 150 ms beep.
// Leak on BOTH sensors: continuous 5 sec alarm, stops early if leak clears.
// Non-blocking: no delay() in the sound controller.
class Buzzer {
  bool beepActive_ = false;
  bool alarmActive_ = false;
  uint32_t beepStart_ = 0;
  uint32_t alarmStart_ = 0;

  void apply() const {
    digitalWrite(BUZZER_PIN, (beepActive_ || alarmActive_) ? LOW : HIGH);
  }

public:
  void begin() {
    digitalWrite(BUZZER_PIN, HIGH); // Prepare quiet level before OUTPUT.
    pinMode(BUZZER_PIN, OUTPUT);
    apply();
  }

  void beep() {
    beepActive_ = true;
    beepStart_ = millis();
    apply();
  }

  void leakStarted() {
    alarmActive_ = true;
    alarmStart_ = millis();
    apply();
  }

  void leakCleared() {
    alarmActive_ = false;
    apply(); // A pending 150 ms beep can finish normally.
  }

  void update(uint32_t now) {
    if (beepActive_ && now - beepStart_ >= Config::BUZZER_BEEP_MS)
      beepActive_ = false;
    if (alarmActive_ && now - alarmStart_ >= Config::BUZZER_ALARM_MS)
      alarmActive_ = false;
    apply();
  }
};

// ================= TELEGRAM NOTIFICATIONS (OOP) =============
// Telegram cannot set arbitrary text colors. Emoji + HTML make
// critical/warning/normal events visually distinct in the app.
// Rate limiting is independent of the AUTO valve cooldown (5 sec).
enum class TgEvent : uint8_t {
  Leak, AutoStart, ManualStart, SoilWet, ManualStop,
  TimeLimit, VolumeLimit, Forecast, ResetOK, ResetError, Test, Count
};

struct TgMessage {
  TgEvent event;
  uint32_t createdAt;
  time_t eventTime; // time of occurrence, not delayed delivery
  char details[144];
};

class TelegramNotifier {
  QueueHandle_t queue_ = nullptr;
  static constexpr size_t EVENT_COUNT = static_cast<size_t>(TgEvent::Count);
  uint32_t lastQueued_[EVENT_COUNT] = {};
  bool queuedBefore_[EVENT_COUNT] = {};
  uint32_t lastAttempt_ = 0; // accessed only by Telegram task
  bool attempted_ = false;

  static uint32_t repeatDelay(TgEvent event) {
    switch (event) {
      case TgEvent::Leak:       return Config::TG_LEAK_REPEAT_MS;
      case TgEvent::Forecast:   return Config::TG_FORECAST_REPEAT_MS;
      case TgEvent::Test:       return 10000UL;
      case TgEvent::ResetError: return 30000UL;
      default:                  return Config::TG_REPEAT_MS;
    }
  }

  static String escapeHTML(const char *s) {
    String out;
    while (*s) {
      switch (*s) {
        case '&': out += "&amp;"; break;
        case '<': out += "&lt;"; break;
        case '>': out += "&gt;"; break;
        case '"': out += "&quot;"; break;
        default: out += *s;
      }
      ++s;
    }
    return out;
  }

  static String escapeJSON(const String &s) {
    String out;
    for (size_t i = 0; i < s.length(); ++i) {
      char c = s[i];
      if (c == '"' || c == '\\') out += '\\';
      if (c == '\n') out += "\\n";
      else if (c != '\r') out += c;
    }
    return out;
  }

  static String format(const TgMessage &msg) {
    String body = "💧 <b>SMART WATER | แจ้งเตือนระบบน้ำ</b>\n"
                  "━━━━━━━━━━━━━━━━\n";
    switch (msg.event) {
      case TgEvent::Leak:
        body += "🚨 <b>ฉุกเฉิน: ตรวจพบน้ำรั่ว!</b>\n"
                "🔴 <b>วาล์ว: ปิดเพื่อความปลอดภัย</b>\n"
                "🔎 กรุณาตรวจสอบเซนเซอร์น้ำรั่วทั้ง 2 จุด\n";
        break;
      case TgEvent::AutoStart:
        body += "🟢 <b>เริ่มรดน้ำอัตโนมัติ</b>\n"
                "🚰 วาล์ว: <b>เปิด</b>\n"
                "🌱 สาเหตุ: ความชื้นดินต่ำกว่าเกณฑ์\n";
        break;
      case TgEvent::ManualStart:
        body += "🟢 <b>เปิดวาล์วด้วยมือ</b>\n"
                "📱 คำสั่งจาก MQTT / Dashboard\n"
                "🚰 วาล์ว: <b>เปิด</b>\n";
        break;
      case TgEvent::SoilWet:
        body += "✅ <b>ดินชื้นเพียงพอแล้ว</b>\n"
                "🟢 หยุดรดน้ำอัตโนมัติ\n"
                "🚰 วาล์ว: <b>ปิด</b>\n";
        break;
      case TgEvent::ManualStop:
        body += "🟠 <b>ปิดวาล์วจาก Dashboard</b>\n"
                "🚰 วาล์ว: <b>ปิด</b>\n"
                "⏸️ โหมด AUTO: พักชั่วคราว\n";
        break;
      case TgEvent::TimeLimit:
        body += "⚠️ <b>ครบเวลารดน้ำอัตโนมัติ</b>\n"
                "🟠 ใช้เวลา AUTO สะสมครบ 5 นาทีต่อวัน\n"
                "🚰 วาล์ว: <b>ปิด</b>\n";
        break;
      case TgEvent::VolumeLimit:
        body += "⚠️ <b>ใช้น้ำถึงขีดจำกัดวันนี้</b>\n"
                "🟠 หยุดรดน้ำอัตโนมัติ\n"
                "🚰 วาล์ว: <b>ปิด</b>\n";
        break;
      case TgEvent::Forecast:
        body += "🟡 <b>พยากรณ์: ดินอาจแห้งใน 30 นาที</b>\n"
                "📈 คาดการณ์จากแนวโน้มค่า ADC ไม่ใช่เวลาที่แน่นอน\n";
        break;
      case TgEvent::ResetOK:
        body += "🔵 <b>RESET ตัวนับสำเร็จ</b>\n"
                "🔄 ล้างตัวนับน้ำและเวลารดน้ำแล้ว\n"
                "🔒 ไม่เปลี่ยนสถานะวาล์ว\n";
        break;
      case TgEvent::ResetError:
        body += "🚨 <b>RESET ไม่สมบูรณ์</b>\n"
                "🔴 บันทึกตัวนับลงหน่วยความจำไม่สำเร็จ\n"
                "🔎 กรุณาตรวจสอบ ESP32 / NVS\n";
        break;
      case TgEvent::Test:
        body += "🧪 <b>ทดสอบ Telegram สำเร็จ</b>\n"
                "🟢 ESP32 ติดต่อบอทได้แล้ว\n";
        break;
      default: break;
    }
    if (msg.details[0]) body += "\n📊 <b>รายละเอียด:</b> " + escapeHTML(msg.details) + "\n";
    time_t t = msg.eventTime;
    tm local = {};
    if (t > 1700000000 && localtime_r(&t, &local)) {
      char timestamp[24];
      strftime(timestamp, sizeof(timestamp), "%d/%m/%Y %H:%M:%S", &local);
      body += String("🕒 <b>เวลา:</b> ") + timestamp + " น.\n";
    } else {
      body += String("🕒 <b>เวลาเครื่อง:</b> ") + String((unsigned int)(msg.createdAt / 1000UL)) + " วินาทีหลังเปิด\n";
    }
    body += "━━━━━━━━━━━━━━━━\n<i>Smart Water • ESP32</i>";
    return body;
  }

  static bool sendHTTP(const TgMessage &msg) {
    WiFiClientSecure tls;
    tls.setInsecure(); // Demo TLS; use a trusted CA certificate for production.
    tls.setTimeout(3000);
    HTTPClient http;
    String url = String("https://api.telegram.org/bot") + Config::BOT_TOKEN + "/sendMessage";
    if (!http.begin(tls, url)) return false;
    http.setTimeout(3000);
    http.addHeader("Content-Type", "application/json");
    bool silent = msg.event == TgEvent::Forecast || msg.event == TgEvent::Test;
    String data = String("{\"chat_id\":\"") + Config::CHAT_ID +
                  "\",\"parse_mode\":\"HTML\",\"disable_notification\":" +
                  (silent ? "true" : "false") +
                  ",\"text\":\"" + escapeJSON(format(msg)) + "\"}";
    int code = http.POST(data);
    http.end();
    if (code != HTTP_CODE_OK)
      Serial.printf("Telegram: HTTP %d (Wi-Fi/token/chat ID/API)\n", code);
    return code == HTTP_CODE_OK;
  }

  // Space every outgoing HTTP request (including retries) to avoid flooding.
  void waitForSendSlot() {
    uint32_t now = millis();
    if (attempted_) {
      uint32_t elapsed = now - lastAttempt_;
      if (elapsed < Config::TG_SEND_GAP_MS)
        vTaskDelay(pdMS_TO_TICKS(Config::TG_SEND_GAP_MS - elapsed));
    }
    lastAttempt_ = millis();
    attempted_ = true;
  }

  static void worker(void *arg) {
    auto *self = static_cast<TelegramNotifier *>(arg);
    TgMessage msg = {};
    for (;;) {
      if (xQueueReceive(self->queue_, &msg, portMAX_DELAY) != pdTRUE) continue;
      // HTTP never blocks the local safety and valve control loop.
      while (WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(3000));
      bool sent = false;
      for (uint8_t tryNo = 0; tryNo < 2 && !sent; ++tryNo) {
        self->waitForSendSlot();
        if (WiFi.status() == WL_CONNECTED) sent = sendHTTP(msg);
      }
      if (!sent) Serial.println("Telegram: alert send failed after 2 attempts");
    }
  }

public:
  void begin() {
    if (strncmp(Config::BOT_TOKEN, "YOUR_", 5) == 0 ||
        strncmp(Config::CHAT_ID, "YOUR_", 5) == 0) {
      Serial.println("Telegram: set BOT_TOKEN + CHAT_ID to enable");
      return;
    }
    queue_ = xQueueCreate(12, sizeof(TgMessage));
    if (!queue_) { Serial.println("Telegram: queue allocation failed"); return; }
    if (xTaskCreatePinnedToCore(worker, "Telegram", 8192, this, 1, nullptr, 0) != pdPASS) {
      vQueueDelete(queue_); queue_ = nullptr;
      Serial.println("Telegram: task creation failed");
    }
  }

  void notify(TgEvent event, const String &details = "") {
    if (!queue_) return;
    uint32_t now = millis();
    size_t index = static_cast<size_t>(event);
    if (index >= EVENT_COUNT) return;
    if (queuedBefore_[index] && now - lastQueued_[index] < repeatDelay(event)) {
      Serial.printf("Telegram: repeated event %u skipped\n", (unsigned)index);
      return;
    }
    TgMessage msg = {};
    msg.event = event;
    msg.createdAt = now;
    msg.eventTime = time(nullptr);
    details.toCharArray(msg.details, sizeof(msg.details));
    // Place leaks ahead of routine queued notifications.
    bool urgent = event == TgEvent::Leak || event == TgEvent::ResetError;
    BaseType_t result = urgent ? xQueueSendToFront(queue_, &msg, 0)
                               : xQueueSend(queue_, &msg, 0);
    if (result != pdTRUE && urgent) {
      // Make room for safety events if ordinary alerts have filled the queue.
      TgMessage dropped = {};
      if (xQueueReceive(queue_, &dropped, 0) == pdTRUE) {
        result = xQueueSendToFront(queue_, &msg, 0);
        Serial.println("Telegram: queue full; urgent alert takes priority");
      }
    }
    if (result == pdTRUE) {
      queuedBefore_[index] = true;
      lastQueued_[index] = now;
    } else Serial.println("Telegram: queue full; event not queued");
  }
};

// ===================== 5. MAIN OOP CONTROLLER ==============
void mqttCallback(char *topic, byte *payload, unsigned int length);

class SmartWater {
  Sensors sensors_;
  Valve valve_;
  StatusLED leds_;
  Buzzer buzzer_;
  TelegramNotifier telegram_;
  Preferences nv_;
  WiFiClientSecure mqttTLS_;
  PubSubClient mqtt_;

  uint8_t crop_ = 1; // Default = TOMATO
  bool manual_ = false, autoPaused_ = false, cooldownActive_ = false;
  bool wasSoilWet_ = false;
  bool wasLeaking_ = false, wasForecastSoon_ = false;
  uint32_t lastTick_ = 0, lastRead_ = 0, lastSave_ = 0;
  uint32_t lastValveClosed_ = 0, manualEnd_ = 0, openMs_ = 0;
  uint32_t lastWiFi_ = 0, lastMQTT_ = 0, lastReport_ = 0;
  uint32_t lastTrend_ = 0, lastForecastAlert_ = 0, savedDay_ = 0;
  uint32_t lastHistory_ = 0, lastHistoryPump_ = 0;
  uint32_t historyId_ = 0, historyAckId_ = 0;
  uint32_t pendingHistoryId_ = 0, pendingAt_ = 0;
  size_t historyReadPos_ = 0, pendingNextPos_ = 0;
  bool historyReady_ = false;
  String pendingHistoryLine_;
  float dailyL_ = 0, totalL_ = 0, flowLpm_ = 0;
  int soil_ = 0;
  float minutesToDry_ = -1, soilSlope_ = 0;
  const char *forecastState_ = "UNKNOWN";
  String reason_ = "BOOT_SAFE_OFF";

  static constexpr uint8_t TREND_N = 12;
  int soilValues_[TREND_N] = {};
  uint32_t soilTimes_[TREND_N] = {};
  uint8_t head_ = 0, count_ = 0;

  const Crop &crop() const { return crops[crop_]; }
  bool dry() const { return soil_ >= crop().dryADC; }
  void send(const char *topic, const String &value, bool retained = false) {
    if (mqtt_.connected()) mqtt_.publish(topic, value.c_str(), retained);
  }
  void send(const char *topic, const char *value, bool retained = false) {
    if (mqtt_.connected()) mqtt_.publish(topic, value, retained);
  }
  void sendInt(const char *topic, uint32_t value) { send(topic, String(value)); }
  void sendFloat(const char *topic, float value) { send(topic, String(value, 2)); }

  void relayState() {
    send("solenoid/status", valve_.isOpen() ? "ON" : "OFF", true);
    send("solenoid/gpio13", valve_.gpio(), true);
  }
  void diagnose(const char *why) {
    if (reason_ == why) return;
    reason_ = why;
    Serial.printf("Relay: %s\n", why);
    send("solenoid/diagnostic", why, true);
  }
  bool saveCounters() {
    bool ok = nv_.putFloat("dailyL", dailyL_) == sizeof(float);
    ok = (nv_.putFloat("totalL", totalL_) == sizeof(float)) && ok;
    ok = (nv_.putULong("openMs", openMs_) == sizeof(uint32_t)) && ok;
    if (savedDay_) ok = (nv_.putUInt("dayKey", savedDay_) == sizeof(uint32_t)) && ok;
    // Checkpoint once per 60s, not once per history row (NVS flash wear).
    ok = (nv_.putUInt("histSeq", historyId_) == sizeof(uint32_t)) && ok;
    ok = (nv_.putUInt("histAck", historyAckId_) == sizeof(uint32_t)) && ok;
    return ok;
  }

  // History row: id,epoch_s,uptime_ms,crop,soil_adc,trend_adc_min,
  // predicted_min,daily_l,leak1,leak2,flow_l_min,valve_on,wifi_connected,event.
  // The Node-RED CSV Function expects a numeric id + exactly 13 fields.
  void beginHistory() {
    historyId_ = nv_.getUInt("histSeq", 0);
    historyAckId_ = nv_.getUInt("histAck", 0);
    historyReady_ = LittleFS.begin(true);
    if (!historyReady_) {
      Serial.println("HISTORY ERROR: LittleFS mount failed");
      return;
    }
    File f = LittleFS.open("/history.csv", "r");
    if (f) {
      while (f.available()) {
        String line = f.readStringUntil('\n');
        uint32_t id = (uint32_t)strtoul(line.c_str(), nullptr, 10);
        if (id > historyId_) historyId_ = id;
      }
      f.close();
    }
    Serial.printf("HISTORY READY: 10s CSV, last id=%lu\n", (unsigned long)historyId_);
  }

  void recordHistory(uint32_t now) {
    lastHistory_ = now;
    if (!historyReady_) return;
    File f = LittleFS.open("/history.csv", "a");
    if (!f) { Serial.println("HISTORY ERROR: open for append"); return; }
    // Keep local history bounded. After ~512KB of records, rotate the file.
    if (f.size() >= 512UL * 1024UL) {
      f.close();
      LittleFS.remove("/history.csv");
      historyReadPos_ = pendingNextPos_ = 0;
      pendingHistoryId_ = 0;
      pendingHistoryLine_ = "";
      nv_.putUInt("histSeq", historyId_); // Preserve unique numeric ids.
      f = LittleFS.open("/history.csv", "a");
      if (!f) return;
      Serial.println("HISTORY: rotated 512KB file (oldest records dropped)");
    }
    ++historyId_;
    time_t epoch = time(nullptr);
    String line = String(historyId_) + "," + String(epoch > 1700000000 ? (uint32_t)epoch : 0U) +
      "," + String(now) + "," + crop().name + "," + String(soil_) +
      "," + String(soil_) +
      "," + String(dailyL_, 3) +
      "," + String(digitalRead(Config::LEAK1) == LOW ? 1 : 0) +
      "," + String(digitalRead(Config::LEAK2) == LOW ? 1 : 0) +
      "," + String(flowLpm_, 2) +
      "," + String(valve_.isOpen() ? 1 : 0) +
      "," + String(WiFi.status() == WL_CONNECTED ? 1 : 0) +
      "," + (sensors_.leaking() ? "LEAK" : "SAMPLE");
    f.println(line);
    f.close();
  }

  // ACK-based delivery: retry the same row until Node-RED acknowledges it.
  // A checkpoint every 60s may replay a few records after power loss; Node-RED
  // filters duplicates by the numeric record id from the first CSV column.
  void publishHistory(uint32_t now) {
    if (!historyReady_ || !mqtt_.connected()) return;
    if (now - lastHistoryPump_ < 250UL) return; // Avoid constantly reopening LittleFS.
    lastHistoryPump_ = now;
    if (pendingHistoryId_) {
      if (now - pendingAt_ >= 4000UL && mqtt_.publish("water/history", pendingHistoryLine_.c_str()))
        pendingAt_ = now;
      return;
    }
    File f = LittleFS.open("/history.csv", "r");
    if (!f) return;
    if (!f.seek(historyReadPos_)) { historyReadPos_ = 0; f.seek(0); }
    while (f.available()) {
      String line = f.readStringUntil('\n');
      const size_t nextPos = f.position();
      line.trim();
      int comma = line.indexOf(',');
      if (comma < 1) { historyReadPos_ = nextPos; continue; }
      uint32_t id = (uint32_t)strtoul(line.c_str(), nullptr, 10);
      if (id <= historyAckId_) { historyReadPos_ = nextPos; continue; }
      if (mqtt_.publish("water/history", line.c_str())) {
        pendingHistoryId_ = id;
        pendingHistoryLine_ = line;
        pendingNextPos_ = nextPos;
        pendingAt_ = now;
      }
      break;
    }
    f.close();
  }

  void ackHistory(uint32_t id) {
    if (pendingHistoryId_ && id == pendingHistoryId_) {
      historyAckId_ = id;
      historyReadPos_ = pendingNextPos_;
      pendingHistoryId_ = 0;
      pendingHistoryLine_ = "";
    }
  }

  void setCrop(uint8_t index) {
    if (index >= CROP_COUNT) return;
    crop_ = index;
    nv_.putUChar("crop", crop_);
    head_ = count_ = 0; // Start a fresh trend for the new crop.
    minutesToDry_ = -1; soilSlope_ = 0; forecastState_ = "UNKNOWN";
    wasForecastSoon_ = false;
    send("crop/name", crop().name, true);
    Serial.printf("CROP=%s threshold=%d autoMax=%.2fL/day\n",
                  crop().name, crop().dryADC, crop().maxLiters);
  }

  bool openValve(bool byUser) {
    if (sensors_.leaking()) { diagnose("BLOCKED_LEAK"); return false; }
    // Preserve original behavior: MANUAL bypasses AUTO limits, never leak safety.
    if (!byUser && openMs_ >= Config::MAX_AUTO_MS) {
      diagnose("OPEN_TIME_LIMIT"); return false;
    }
    if (!byUser && dailyL_ >= crop().maxLiters) {
      diagnose("DAILY_LIMIT_REACHED"); return false;
    }
    bool wasOpen = valve_.isOpen();
    valve_.open(); manual_ = byUser;
    if (!wasOpen) { leds_.valveChanged(true); buzzer_.beep(); }
    diagnose(byUser ? "OPEN_MANUAL" : "OPEN_AUTO");
    relayState();
    if (!wasOpen) telegram_.notify(byUser ? TgEvent::ManualStart : TgEvent::AutoStart,
      String("พืช: ") + crop().name + " | ดิน: " + String(soil_) + " ADC");
    return true;
  }
  void closeValve(const char *why, bool pause = false) {
    bool wasOpen = valve_.isOpen();
    if (wasOpen && manual_) manualEnd_ = millis();
    if (wasOpen) {
      lastValveClosed_ = millis();
      cooldownActive_ = true;
    }
    valve_.close(); manual_ = false;
    if (wasOpen) { leds_.valveChanged(false); buzzer_.beep(); }
    if (pause) autoPaused_ = true;
    diagnose(why);
    if (wasOpen) {
      relayState();
      if (pause) telegram_.notify(TgEvent::ManualStop, String("ดิน: ") + String(soil_) + " ADC");
      else if (strcmp(why, "SOIL_WET") == 0)
        telegram_.notify(TgEvent::SoilWet, String("ดิน: ") + String(soil_) + " ADC");
    }
  }
  void safety(uint32_t now) {
    if (valve_.isOpen() && !manual_) openMs_ += now - lastTick_;
    lastTick_ = now;
    bool leak = sensors_.leaking();
    if (leak && valve_.isOpen()) closeValve("BLOCKED_LEAK");
    if (leak && !wasLeaking_) {
      buzzer_.leakStarted();
      send("alert/leak", "1");
      telegram_.notify(TgEvent::Leak, "เซนเซอร์ 1 และ 2: พบน้ำ | วาล์ว: OFF");
    }
    if (!leak && wasLeaking_) buzzer_.leakCleared();
    wasLeaking_ = leak;
    if (valve_.isOpen() && !manual_ && openMs_ >= Config::MAX_AUTO_MS) {
      closeValve("OPEN_TIME_LIMIT");
      send("solenoid/limit", "TIME_REACHED");
      telegram_.notify(TgEvent::TimeLimit, String("ใช้เวลา AUTO: ") + String((unsigned int)(openMs_ / 1000UL)) + " วินาที");
    }
  }
  void resetCounters() {
    sensors_.resetFlow();
    flowLpm_ = dailyL_ = totalL_ = 0;
    openMs_ = 0;
    lastTick_ = lastRead_ = millis();
    bool saved = saveCounters(); // Preserve relay state and manual/auto mode.
    send("water/flow", "0.00");
    send("water/daily_liters", "0.00");
    send("water/total_liters", "0.00");
    send("water/open_seconds", "0");
    sendInt("water/open_remaining_seconds", (uint32_t)(Config::MAX_AUTO_MS / 1000UL));
    send("water/limit_full", "0");
    send("solenoid/limit", "RESET");
    send("water/reset/status", saved ? "RESET_OK" : "ERROR_NVS_SAVE");
    relayState();
    telegram_.notify(saved ? TgEvent::ResetOK : TgEvent::ResetError,
      String("วาล์ว: ") + (valve_.isOpen() ? "ON" : "OFF"));
    Serial.println(saved ? "RESET OK: valve state unchanged" : "RESET ERROR: NVS save failed");
  }
  void newDay() {
    time_t utc = time(nullptr);
    if (utc < 1700000000) return;
    tm t = {};
    if (!localtime_r(&utc, &t)) return;
    uint32_t day = (uint32_t)(t.tm_year + 1900) * 1000U + t.tm_yday;
    if (!savedDay_) { savedDay_ = day; saveCounters(); }
    else if (day != savedDay_) {
      savedDay_ = day;
      dailyL_ = 0; openMs_ = 0;
      saveCounters();
      send("water/daily_liters", "0.00");
      send("solenoid/limit", "RESET");
      Serial.println("New day: AUTO limits reset");
    }
  }
  void updateForecast(uint32_t now) {
    if (!count_ || now - lastTrend_ >= 10000UL) {
      lastTrend_ = now;
      soilValues_[head_] = soil_;
      soilTimes_[head_] = now;
      head_ = (head_ + 1) % TREND_N;
      if (count_ < TREND_N) ++count_;
    }
    minutesToDry_ = -1; soilSlope_ = 0; forecastState_ = "UNKNOWN";
    if (dry()) { minutesToDry_ = 0; forecastState_ = "DRY_NOW"; }
    else if (count_ >= 4) {
      uint8_t first = (head_ + TREND_N - count_) % TREND_N;
      uint8_t last = (head_ + TREND_N - 1) % TREND_N;
      uint32_t elapsed = soilTimes_[last] - soilTimes_[first];
      if (elapsed >= 25000UL) {
        soilSlope_ = (soilValues_[last] - soilValues_[first]) * 60000.0f / elapsed;
        if (soilSlope_ > 2.0f) {
          minutesToDry_ = (crop().dryADC - soil_) / soilSlope_;
          forecastState_ = minutesToDry_ <= 30 ? "SOON" : "OK";
        }
      }
    }
    bool soon = strcmp(forecastState_, "SOON") == 0;
    if (soon && !wasForecastSoon_ &&
        (!lastForecastAlert_ || now - lastForecastAlert_ >= 1800000UL)) {
      lastForecastAlert_ = now;
      send("alert/forecast", "SOIL_DRY_SOON");
      telegram_.notify(TgEvent::Forecast, String("คาดว่าอีก ") + String(minutesToDry_, 1) + " นาที | ดิน: " + String(soil_) + " ADC");
    }
    wasForecastSoon_ = soon;
  }

  void sample(uint32_t now) {
    float liters = sensors_.readLiters();
    uint32_t elapsed = now - lastRead_;
    lastRead_ = now;
    flowLpm_ = elapsed ? liters * 60000.0f / elapsed : 0;
    totalL_ += liters;
    if (!manual_ && now - manualEnd_ >= 2000UL) dailyL_ += liters;
    soil_ = sensors_.soil();
    bool soilWet = !dry();
    if (soilWet && !wasSoilWet_) { leds_.soilBecameWet(); buzzer_.beep(); }
    wasSoilWet_ = soilWet;
    updateForecast(now);

    if (!dry()) {
      autoPaused_ = false;
      if (valve_.isOpen() && !manual_) closeValve("SOIL_WET");
    } else if (!valve_.isOpen() && !autoPaused_ && !sensors_.leaking() &&
               (!cooldownActive_ || now - lastValveClosed_ >= Config::COOLDOWN_MS)) {
      openValve(false);
    }
    if (valve_.isOpen() && !manual_ && dailyL_ >= crop().maxLiters) {
      closeValve("DAILY_LIMIT_REACHED");
      send("solenoid/limit", "REACHED");
      telegram_.notify(TgEvent::VolumeLimit, String("ใช้น้ำ: ") + String(dailyL_, 2) + "/" + String(crop().maxLiters, 2) + " ลิตร");
    }

    sendFloat("water/flow", flowLpm_);
    sendFloat("water/total_liters", totalL_);
    sendFloat("water/daily_liters", dailyL_);
    sendFloat("water/limit_liters", crop().maxLiters);
    send("water/limit_full", dailyL_ >= crop().maxLiters ? "1" : "0");
    sendInt("water/open_seconds", openMs_ / 1000U);
    sendInt("water/open_limit", (uint32_t)(Config::MAX_AUTO_MS / 1000UL));
    sendInt("water/open_remaining_seconds", openMs_ >= Config::MAX_AUTO_MS
      ? 0U : (uint32_t)((Config::MAX_AUTO_MS - openMs_) / 1000UL));
    sendInt("soil/moisture", (uint32_t)soil_);
    send("soil/status", dry() ? "DRY" : "WET");
    send("crop/name", crop().name);
    sendInt("crop/dry_threshold", (uint32_t)crop().dryADC);
    sendFloat("forecast/minutes_to_dry", minutesToDry_);
    sendFloat("forecast/soil_trend_adc_min", soilSlope_);
    send("forecast/status", forecastState_);
    send("rain/sensor/1", digitalRead(Config::LEAK1) ? "1" : "0");
    send("rain/sensor/2", digitalRead(Config::LEAK2) ? "1" : "0");
    time_t utc = time(nullptr);
    tm local = {};
    if (utc > 1700000000 && localtime_r(&utc, &local)) {
      uint32_t seconds = local.tm_hour * 3600U + local.tm_min * 60U + local.tm_sec;
      sendInt("water/limit_countdown", 86400U - seconds);
    }

    if (valve_.isOpen()) diagnose(manual_ ? "OPEN_MANUAL" : "OPEN_AUTO");
    else if (sensors_.leaking()) diagnose("BLOCKED_LEAK");
    else if (!dry()) diagnose("SOIL_WET");
    else if (autoPaused_) diagnose("AUTO_PAUSED_OFF");
    else if (dailyL_ >= crop().maxLiters) diagnose("DAILY_LIMIT_REACHED");
    else if (openMs_ >= Config::MAX_AUTO_MS) diagnose("OPEN_TIME_LIMIT");
    else if (cooldownActive_ && now - lastValveClosed_ < Config::COOLDOWN_MS) diagnose("AUTO_COOLDOWN");
    else diagnose("READY_FOR_AUTO");
    Serial.printf("Soil=%d(%s) Flow=%.2f L/min Daily=%.2f/%.2fL "
                  "Auto=%lus/300s Valve=%s Leak=%d Forecast=%s\n",
      soil_, dry() ? "DRY" : "WET", flowLpm_, dailyL_, crop().maxLiters,
      (unsigned long)(openMs_ / 1000U), valve_.isOpen() ? "ON" : "OFF",
      sensors_.leaking(), forecastState_);
  }

  void network(uint32_t now) {
    if (WiFi.status() != WL_CONNECTED) {
      if (now - lastWiFi_ >= 15000UL) { lastWiFi_ = now; WiFi.reconnect(); }
      return;
    }
    if (!mqtt_.connected() && now - lastMQTT_ >= 15000UL) {
      lastMQTT_ = now;
      String clientId = String("ESP32_SMART_WATER_") + String((uint32_t)ESP.getEfuseMac(), HEX);
      if (mqtt_.connect(clientId.c_str(), Config::MQTT_USER, Config::MQTT_PASS,
                        "device/status", 1, true, "offline")) {
        mqtt_.subscribe("solenoid/control");
        mqtt_.subscribe("water/reset");
        mqtt_.subscribe("crop/select");
        mqtt_.subscribe("water/history/ack");
        send("device/status", "online", true);
        send("crop/name", crop().name, true);
        send("solenoid/diagnostic", reason_.c_str(), true);
        relayState();
        Serial.println("MQTT ONLINE");
      } else Serial.printf("MQTT ERROR %d\n", mqtt_.state());
    }
    if (mqtt_.connected()) mqtt_.loop();
  }

public:
  SmartWater() : mqtt_(mqttTLS_) {}

  void begin() {
    Serial.begin(115200);
    Serial.setTimeout(50);
    valve_.begin();               // Fail-safe OFF at boot.
    sensors_.begin();
    leds_.begin(sensors_.leaking());
    buzzer_.begin();
    nv_.begin("smartwater", false); // Reuse counters from original sketch.
    for (uint8_t i = 0; i < CROP_COUNT; ++i) {
      char th[8], lim[8];
      snprintf(th, sizeof(th), "th%d", i);
      snprintf(lim, sizeof(lim), "lim%d", i);
      crops[i].dryADC = nv_.getInt(th, crops[i].dryADC);
      crops[i].maxLiters = nv_.getFloat(lim, crops[i].maxLiters);
    }
    crop_ = nv_.getUChar("crop", 1);
    if (crop_ >= CROP_COUNT) crop_ = 1;
    dailyL_ = nv_.getFloat("dailyL", 0);
    totalL_ = nv_.getFloat("totalL", 0);
    openMs_ = nv_.getULong("openMs", 0);
    savedDay_ = nv_.getUInt("dayKey", 0);
    beginHistory();
    setCrop(crop_);
    soil_ = sensors_.soil();
    wasSoilWet_ = !dry(); // Old sketch: no soil-wet chime just because it booted.

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(Config::WIFI_NAME, Config::WIFI_PASS);
    configTime(7 * 3600, 0, "pool.ntp.org");
    mqttTLS_.setInsecure(); // Demo only: use trusted certificate in production.
    mqttTLS_.setTimeout(2000);
    mqtt_.setServer(Config::MQTT_HOST, Config::MQTT_PORT);
    mqtt_.setCallback(mqttCallback);
    mqtt_.setKeepAlive(15);
    mqtt_.setSocketTimeout(2);
    mqtt_.setBufferSize(512);
    telegram_.begin();
    lastTick_ = lastRead_ = lastSave_ = lastWiFi_ = lastHistory_ = millis();
    lastMQTT_ = millis() - 15000UL; // Connect to MQTT immediately once Wi-Fi is ready.
    Serial.printf("READY: GPIO13=%s (OFF), soil=%d crop=%s, auto limit=300s\n",
                  valve_.gpio(), soil_, crop().name);
    Serial.println("Serial Monitor commands: STATUS, TESTTG");
  }

  void handleMqtt(char *topic, byte *bytes, unsigned int length) {
    String cmd;
    if (length > 64) return;
    for (unsigned i = 0; i < length; ++i) cmd += (char)bytes[i];
    cmd.trim(); cmd.toUpperCase();
    Serial.printf("MQTT %s = %s\n", topic, cmd.c_str());
    if (strcmp(topic, "water/history/ack") == 0) {
      ackHistory((uint32_t)strtoul(cmd.c_str(), nullptr, 10));
    } else if (strcmp(topic, "water/reset") == 0) {
      if (cmd == "RESET") resetCounters();
    } else if (strcmp(topic, "solenoid/control") == 0) {
      if (cmd == "ON" || cmd == "OPEN") openValve(true);
      else if (cmd == "OFF" || cmd == "CLOSE") closeValve("AUTO_PAUSED_OFF", true);
      else if (cmd == "AUTO") {
        if (valve_.isOpen() && manual_) closeValve("AUTO_RESTORED");
        manual_ = false; autoPaused_ = false;
        diagnose("AUTO_RESTORED");
      }
    } else if (strcmp(topic, "crop/select") == 0) {
      for (uint8_t i = 0; i < CROP_COUNT; ++i)
        if (cmd == crops[i].name || cmd == String(i)) { setCrop(i); break; }
    }
  }

  void loop() {
    uint32_t now = millis();
    safety(now);  // Local safety has priority over network and Telegram.
    newDay();
    network(now);
    now = millis(); // MQTT callbacks (e.g. RESET) may update timers.
    if (now - lastRead_ >= Config::SAMPLE_MS) sample(now);
    if (now - lastHistory_ >= Config::HISTORY_MS) recordHistory(now);
    publishHistory(now);
    if (now - lastSave_ >= 60000UL) { lastSave_ = now; saveCounters(); }
    if (now - lastReport_ >= 5000UL) { lastReport_ = now; relayState(); }
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n'); input.trim(); input.toUpperCase();
      if (input == "TESTTG") telegram_.notify(TgEvent::Test, "คำสั่งทดสอบจาก Serial Monitor");
      else if (input == "STATUS")
        Serial.printf("Relay=%s GPIO=%s Leak=%d Soil=%d Daily=%.2f Auto=%lus Reason=%s\n",
          valve_.isOpen() ? "ON" : "OFF", valve_.gpio(), sensors_.leaking(),
          soil_, dailyL_, (unsigned long)(openMs_ / 1000U), reason_.c_str());
    }
    leds_.update(millis(), sensors_.leaking());
    buzzer_.update(millis());
    delay(5);
  }
};

SmartWater water;
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  water.handleMqtt(topic, payload, length);
}
void setup() { water.begin(); }
void loop()  { water.loop(); }
