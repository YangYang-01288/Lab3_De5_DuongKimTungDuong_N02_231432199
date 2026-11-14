/*
  Lab 3 - Đề 5: Fall Detection + FreeRTOS (ESP32 + MPU6050 + MQTT)

  TaskReadData (Core 1, prio 2): đọc MPU6050 mỗi 20 ms (50 Hz)
                                 -> dataAlert mỗi 20 ms, -> dataQueue mỗi 500 ms
  TaskSendData (Core 0, prio 1): WiFi + MQTT; đọc dataQueue -> publish; nhận lệnh từ web
  TaskAlert    (Core 1, prio 3): đọc dataAlert -> thuật toán té ngã -> LED / LOA

  Thứ tự in khối Serial (nối đuôi bằng binary semaphore):
     TRUYEN DATA  ->  DOC DATA  ->  CANH BAO

  [BAN SUA] Các chỗ đã sửa được đánh dấu bằng "[FIX 1]", "[FIX 2]", "[FIX 3]", "[OPT]"
*/
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <stdarg.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ====================== 1. CẤU HÌNH ======================
#define PREFIX "lab3de5_DuongKimTungDuong_231432199"

// 1: gọi WiFi.begin() trong setup() (giống Lab 2), Task Core 0 chờ kết nối và báo trạng thái
// 0: Task truyền data tự gọi WiFi.begin() trên Core 0
#define WIFI_BEGIN_IN_SETUP 1

const char* ssid        = "Wokwi-GUEST";
const char* password    = "";
const char* mqtt_server = "broker.emqx.io";
const int   mqtt_port   = 1883;

// Topic gửi lên (publish)
const char* T_ACCEL  = PREFIX "/mpu6050/accel";
const char* T_ANGLE  = PREFIX "/mpu6050/angle";
const char* T_STATUS = PREFIX "/status";
// Topic nhận về (subscribe)
const char* T_LED    = PREFIX "/device/led";
const char* T_BUZZER = PREFIX "/device/buzzer";
const char* T_RESET  = PREFIX "/device/reset";
const char* T_MANUAL = PREFIX "/device/manual";

// Chân kết nối
#define LED_PIN    15
#define BUZZER_PIN 18
// MPU6050: SDA = GPIO21, SCL = GPIO22

// Ngưỡng thuật toán
const float IMPACT_G   = 1.8;
const float ANGLE_FALL = 60.0;
const float STILL_LOW  = 0.8;
const float STILL_HIGH = 1.2;
const unsigned long CONFIRM_MS = 3000;

// Chu kỳ
const int READ_PERIOD_MS  = 20;     // đọc cảm biến: 50 lần/giây
const int SEND_EVERY_MS   = 500;    // đưa dữ liệu vào dataQueue (publish MQTT)
const int PRINT_EVERY_MS  = 3000;   // chu kỳ in một "vòng" 3 khối Serial
const int PRINT_TIMEOUT_MS = 1500;  // quá thời gian này mà chưa tới lượt thì Task đọc tự in

// ====================== 2. CẤU TRÚC DỮ LIỆU ======================
typedef struct {
  float ax, ay, az;   // gia tốc từng trục (g)
  float svm;          // gia tốc tổng (g)
  float pitch;        // độ
  float roll;         // độ
  float tilt;         // góc so với phương thẳng đứng (độ)
} SensorData;

typedef struct {
  bool fall;        // đang cảnh báo té ngã
  bool manualMode;  // chế độ manual
  bool manualLed;
  bool manualBuzz;
  bool resetReq;
} SharedState;

SharedState shared = {false, false, false, false, false};
SemaphoreHandle_t stateMutex;    // bảo vệ biến dùng chung
SemaphoreHandle_t serialMutex;   // bảo vệ Serial

// Cờ hiệu (binary semaphore) để các Task nối đuôi nhau in theo thứ tự
SemaphoreHandle_t printSendSem;   // tới lượt Task truyền data in
SemaphoreHandle_t printReadSem;   // tới lượt Task đọc data in
SemaphoreHandle_t printAlertSem;  // tới lượt Task cảnh báo in

QueueHandle_t dataQueue;   // Task đọc -> Task truyền (MQTT)
QueueHandle_t dataAlert;   // Task đọc -> Task cảnh báo

enum FallState { IDLE, IMPACT_WAIT };

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;

// Thống kê của Task truyền data
SensorData sendLast;
bool sendHaveData = false;
int  sendCntRecv = 0, sendCntPub = 0;
unsigned long sendLastPrint = 0;

// In Serial an toàn: mỗi lần gọi là một khối liền nhau
void logPrint(const char* fmt, ...) {
  char buf[640];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.print(buf);
  xSemaphoreGive(serialMutex);
}

// Gửi vào queue; nếu đầy thì bỏ mẫu cũ nhất. Trả về false nếu phải bỏ mẫu cũ.
bool sendLatest(QueueHandle_t q, const SensorData* d) {
  if (xQueueSend(q, d, 0) == pdTRUE) return true;
  SensorData tmp;
  xQueueReceive(q, &tmp, 0);
  xQueueSend(q, d, 0);
  return false;
}

// Số mẫu/giây theo thời gian thực tế
int ratePerSec(int cnt, unsigned long elapsedMs) {
  if (elapsedMs == 0) elapsedMs = 1;
  return (int)((cnt * 1000UL + elapsedMs / 2) / elapsedMs);
}

// ====================== LOA (BUZZER) ======================
#define BUZZ_FREQ 2000
#define BUZZ_CH   0

void buzzerInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BUZZER_PIN, BUZZ_FREQ, 8);
#else
  ledcSetup(BUZZ_CH, BUZZ_FREQ, 8);
  ledcAttachPin(BUZZER_PIN, BUZZ_CH);
#endif
}
void buzzerOn() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(BUZZER_PIN, BUZZ_FREQ);
#else
  ledcWriteTone(BUZZ_CH, BUZZ_FREQ);
#endif
}
void buzzerOff() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(BUZZER_PIN, 0);
#else
  ledcWriteTone(BUZZ_CH, 0);
#endif
}

// ====================== KHỐI IN CỦA TASK TRUYỀN DATA ======================
void printSendBlock() {
  unsigned long now = millis();
  unsigned long el = now - sendLastPrint;
  sendLastPrint = now;

  logPrint("\n[TASK TRUYEN DATA  | Core %d] ------------------------------\n"
           "   B1 Ket noi WiFi         : %s\n"
           "   B2 Ket noi MQTT broker  : %s\n"
           "   B3 Doc tu data Queue    : %d mau/giay\n",
           (int)xPortGetCoreID(),
           WiFi.status() == WL_CONNECTED ? "Da ket noi" : "Chua ket noi",
           client.connected() ? "Da ket noi" : "Chua ket noi",
           ratePerSec(sendCntRecv, el));
  if (sendHaveData) {
    logPrint("   B4 Gui len MQTT         : accel=%.2f g | angle=%.1f deg (%d ban tin / %lu giay)\n",
             sendLast.svm, sendLast.tilt, sendCntPub, (el + 500) / 1000);
  } else {
    logPrint("   B4 Gui len MQTT         : chua co du lieu\n");
  }
  sendCntRecv = 0; sendCntPub = 0;
}

// Nếu tới lượt thì in khối của Task truyền data, rồi chuyển lượt cho Task đọc data
void servicePrintSend() {
  if (xSemaphoreTake(printSendSem, 0) == pdTRUE) {
    printSendBlock();
    xSemaphoreGive(printReadSem);
  }
}

// ====================== 3. TASK ĐỌC DATA (Core 1) ======================
void TaskReadData(void* pv) {
  SensorData d;
  sensors_event_t a, g, temp;
  TickType_t lastWake = xTaskGetTickCount();
  unsigned long lastSend = 0, lastPrint = millis(), reqTime = 0;
  bool printPending = false;
  int cntQueue = 0, cntAlert = 0, dropQueue = 0, dropAlert = 0;

  for (;;) {
    // B1: đọc cảm biến
    mpu.getEvent(&a, &g, &temp);
    float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;  // m/s^2

    // B2: tính gia tốc tổng và góc nghiêng
    d.ax = ax / 9.80665;
    d.ay = ay / 9.80665;
    d.az = az / 9.80665;
    d.svm   = sqrt(ax * ax + ay * ay + az * az) / 9.80665;
    d.pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    d.roll  = atan2(ay, az) * 180.0 / PI;
    d.tilt  = atan2(sqrt(ax * ax + ay * ay), az) * 180.0 / PI;

    // B5: gửi vào data Alert (mỗi 20 ms)
    cntAlert++;
    if (!sendLatest(dataAlert, &d)) dropAlert++;

    unsigned long now = millis();

    // B4: gửi vào data Queue (mỗi 500 ms)
    if (now - lastSend >= SEND_EVERY_MS) {
      lastSend = now;
      cntQueue++;
      if (!sendLatest(dataQueue, &d)) dropQueue++;
    }

    // Giữ nhịp: tới giờ thì phát lệnh cho Task truyền data in trước
    if (!printPending && now - lastPrint >= PRINT_EVERY_MS) {
      printPending = true;
      reqTime = now;
      xSemaphoreGive(printSendSem);
    }

    // B3: tới lượt (Task truyền data đã in xong) hoặc quá thời gian chờ -> in khối của Task đọc
    if (printPending) {
      bool myTurn = (xSemaphoreTake(printReadSem, 0) == pdTRUE);
      bool timeout = (now - reqTime > PRINT_TIMEOUT_MS);
      if (myTurn || timeout) {
        if (timeout && !myTurn) xSemaphoreTake(printSendSem, 0);   // huỷ lệnh cũ chưa ai nhận

        unsigned long el = now - lastPrint;
        lastPrint = now;
        logPrint("\n[TASK DOC DATA     | Core %d] ------------------------------\n"
                 "   B1 Doc cam bien MPU6050 : X=%.2f | Y=%.2f | Z=%.2f (g)\n"
                 "   B2 Tinh gia toc, goc    : SVM=%.2f g | Pitch=%.1f | Roll=%.1f | Tilt=%.1f (deg)\n"
                 "   B3 Hien thi Serial      : chinh la khung nay\n"
                 "   B4 Gui vao data Queue   : %d mau/giay  -> %s\n"
                 "   B5 Gui vao data Alert   : %d mau/giay -> %s\n",
                 (int)xPortGetCoreID(),
                 d.ax, d.ay, d.az,
                 d.svm, d.pitch, d.roll, d.tilt,
                 ratePerSec(cntQueue, el), dropQueue ? "DAY, bo mau cu" : "OK",
                 ratePerSec(cntAlert, el), dropAlert ? "DAY, bo mau cu" : "OK");
        cntQueue = cntAlert = dropQueue = dropAlert = 0;
        printPending = false;
        xSemaphoreGive(printAlertSem);   // chuyển lượt cho Task cảnh báo
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(READ_PERIOD_MS));
  }
}

// ====================== 4. TASK CẢNH BÁO (Core 1) ======================
// B1: va chạm mạnh (SVM > IMPACT_G)
// B2: chờ CONFIRM_MS
// B3: góc nghiêng > ANGLE_FALL và nằm yên (SVM ~ 1g) -> TÉ NGÃ
void TaskAlert(void* pv) {
  SensorData d;
  d.svm = 1.0; d.tilt = 0.0;
  FallState st = IDLE;
  unsigned long impactTime = 0, lastBlink = 0, lastPrint = millis();
  bool blinkState = false;
  bool fall = false, mLed = false, mBuzz = false, mMode = false;
  bool ledOn = false, buzzOn = false;
  int cntRecv = 0;

  for (;;) {
    if (xQueueReceive(dataAlert, &d, pdMS_TO_TICKS(200)) == pdTRUE) {
      cntRecv++;

      // Đọc trạng thái dùng chung
      bool rst;
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      fall = shared.fall; mLed = shared.manualLed; mBuzz = shared.manualBuzz;
      mMode = shared.manualMode;
      rst = shared.resetReq; shared.resetReq = false;
      xSemaphoreGive(stateMutex);

      if (rst) st = IDLE;

      // Thuật toán phát hiện té ngã
      if (!fall) {
        if (st == IDLE) {
          if (d.svm > IMPACT_G) {
            st = IMPACT_WAIT;
            impactTime = millis();
            logPrint("\n[TASK CANH BAO     | Core %d] >> Phat hien va cham: %.2f g -> cho %lu ms de xac nhan...\n",
                     (int)xPortGetCoreID(), d.svm, CONFIRM_MS);
          }
        } else {  // IMPACT_WAIT
          if (millis() - impactTime >= CONFIRM_MS) {
            if (d.tilt > ANGLE_FALL && d.svm > STILL_LOW && d.svm < STILL_HIGH) {
              fall = true;
              xSemaphoreTake(stateMutex, portMAX_DELAY);
              shared.fall = true;
              xSemaphoreGive(stateMutex);
              logPrint("\n[TASK CANH BAO     | Core %d] >> !!! TE NGA !!! (Tilt = %.1f deg, SVM = %.2f g)\n",
                       (int)xPortGetCoreID(), d.tilt, d.svm);
            } else {
              logPrint("\n[TASK CANH BAO     | Core %d] >> Bao dong gia, bo qua (Tilt = %.1f deg, SVM = %.2f g)\n",
                       (int)xPortGetCoreID(), d.tilt, d.svm);
            }
            st = IDLE;
          }
        }
      }

      // Điều khiển LED / LOA
      unsigned long now = millis();
      if (now - lastBlink >= 300) { lastBlink = now; blinkState = !blinkState; }

      // Khi bật MANUAL: người dùng toàn quyền điều khiển LED/LOA bằng nút Bật/Tắt
      // (callback đã chuyển trạng thái báo động đang kêu thành lệnh "Bật" lúc vào Manual)
      if (mMode) {
        ledOn  = mLed;
        buzzOn = mBuzz;
      } else {                            // AUTO: cảnh báo tự động khi té ngã
        ledOn  = fall && blinkState;
        buzzOn = fall;
      }

      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
      if (buzzOn) buzzerOn(); else buzzerOff();
    }

    // Tới lượt (Task đọc data đã in xong) -> in khối của Task cảnh báo
    if (xSemaphoreTake(printAlertSem, 0) == pdTRUE) {
      unsigned long now2 = millis();
      unsigned long el = now2 - lastPrint;
      lastPrint = now2;
      const char* stStr = fall ? "TE NGA !!!"
                        : (st == IMPACT_WAIT ? "Da co va cham, dang cho xac nhan" : "Binh thuong");
      logPrint("\n[TASK CANH BAO     | Core %d] ------------------------------\n"
               "   B1 Doc tu data Alert    : %d mau/giay\n"
               "   B2 Phan tich            : SVM=%.2f g | Tilt=%.1f deg | Thuat toan: %s\n"
               "   B3 Dieu khien thiet bi  : LED=%s | LOA=%s | Che do: %s\n",
               (int)xPortGetCoreID(),
               ratePerSec(cntRecv, el),
               d.svm, d.tilt, stStr,
               ledOn ? "BAT" : "TAT", buzzOn ? "BAT" : "TAT",
               mMode ? "MANUAL" : "AUTO");
      cntRecv = 0;
    }
  }
}

// ====================== 5. TASK TRUYỀN DATA (Core 0) ======================
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  String t = String(topic);
  bool on = (msg == "ON");
  bool ignored = false;
  bool wasFall = false;
  bool justEnteredManual = false;

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (t == T_MANUAL) {
    // [FIX 3] Chỉ gán LED/LOA theo trạng thái té ngã khi VỪA chuyển từ Auto sang Manual.
    //         Nếu đã ở Manual rồi (web gửi lại MANUAL ON để đồng bộ sau khi mất mạng)
    //         thì giữ nguyên LED/LOA, tránh bật lại một nhịp.
    bool wasManual = shared.manualMode;
    shared.manualMode = on;
    wasFall = shared.fall;
    if (on) {
      if (!wasManual) {
        shared.manualLed  = shared.fall;
        shared.manualBuzz = shared.fall;
        justEnteredManual = true;
      }
    } else {
      shared.manualLed  = false;
      shared.manualBuzz = false;
    }
  } else if (t == T_LED) {
    if (shared.manualMode) shared.manualLed = on; else ignored = true;
  } else if (t == T_BUZZER) {
    if (shared.manualMode) shared.manualBuzz = on; else ignored = true;
  } else if (t == T_RESET) {
    shared.fall = false; shared.manualMode = false;
    shared.manualLed = false; shared.manualBuzz = false;
    shared.resetReq = true;
  }
  xSemaphoreGive(stateMutex);

  int core = (int)xPortGetCoreID();
  if (t == T_RESET) {
    client.publish(T_STATUS, "NORMAL", true);
    logPrint("\n[TASK TRUYEN DATA  | Core %d] << Nhan lenh tu web: RESET canh bao\n", core);
  } else if (t == T_MANUAL) {
    logPrint("\n[TASK TRUYEN DATA  | Core %d] << Nhan lenh tu web: che do MANUAL %s%s\n",
             core, on ? "BAT" : "TAT (tro ve tu dong)",
             (justEnteredManual && wasFall) ? " (dang te nga: LED/LOA van keu, bam Tat de tat)" : "");
  } else if (ignored) {
    logPrint("\n[TASK TRUYEN DATA  | Core %d] << Bo qua lenh %s (chua bat che do MANUAL)\n",
             core, t == T_LED ? "LED" : "LOA");
  } else {
    logPrint("\n[TASK TRUYEN DATA  | Core %d] << Nhan lenh tu web: %s = %s\n",
             core, t == T_LED ? "LED" : "LOA", msg.c_str());
  }
}

// doBegin = true: gọi WiFi.begin() rồi chờ; false: chỉ chờ (đã gọi begin ở setup)
void connectWifi(bool doBegin) {
  int core = (int)xPortGetCoreID();
  logPrint("\n[TASK TRUYEN DATA  | Core %d] B1 Ket noi WiFi '%s' ...\n", core, ssid);
  if (doBegin) WiFi.begin(ssid, password);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    servicePrintSend();                       // vẫn in khối khi tới lượt, dù đang chờ WiFi
    if (millis() - t0 >= 5000) {
      t0 = millis();
      logPrint("[TASK TRUYEN DATA  | Core %d] B1 Van dang cho WiFi... (status=%d)\n",
               core, WiFi.status());
    }
  }
  logPrint("[TASK TRUYEN DATA  | Core %d] B1 WiFi da ket noi! IP: %s\n",
           core, WiFi.localIP().toString().c_str());
}

void connectMqtt() {
  int core = (int)xPortGetCoreID();
  while (!client.connected()) {
    logPrint("[TASK TRUYEN DATA  | Core %d] B2 Ket noi MQTT broker %s ...\n", core, mqtt_server);
    String clientId = "ESP32Fall-" + String(random(0, 0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      logPrint("[TASK TRUYEN DATA  | Core %d] B2 MQTT da ket noi broker!\n", core);
      client.subscribe(T_LED);
      client.subscribe(T_BUZZER);
      client.subscribe(T_RESET);
      client.subscribe(T_MANUAL);

      // [FIX 1] Gửi đúng trạng thái hiện tại thay vì luôn "NORMAL",
      //         tránh ghi đè FALL khi kết nối lại lúc đang báo động.
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      bool f = shared.fall;
      xSemaphoreGive(stateMutex);
      client.publish(T_STATUS, f ? "FALL" : "NORMAL", true);
    } else {
      logPrint("[TASK TRUYEN DATA  | Core %d] B2 MQTT that bai (rc=%d), thu lai sau 2 giay\n",
               core, client.state());
      vTaskDelay(pdMS_TO_TICKS(2000));
      servicePrintSend();
    }
  }
}

void TaskSendData(void* pv) {
  SensorData d;
  char buf[16];
  bool lastFall = false;
  bool firstWifi = true;
  sendLastPrint = millis();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(5);          // [OPT] phát hiện mất kết nối broker nhanh hơn
  WiFi.setAutoReconnect(true);     // [OPT] tự nối lại WiFi khi rớt

  for (;;) {
    // B1: WiFi
    if (WiFi.status() != WL_CONNECTED) {
      bool doBegin = firstWifi ? (WIFI_BEGIN_IN_SETUP == 0) : true;
      firstWifi = false;
      connectWifi(doBegin);
    }
    // B2: MQTT
    if (!client.connected()) connectMqtt();
    client.loop();

    // Gửi trạng thái lên web khi có thay đổi
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    bool fall = shared.fall;
    xSemaphoreGive(stateMutex);
    // [FIX 2] Chỉ cập nhật lastFall khi publish thành công; nếu thất bại (mất mạng)
    //         vòng sau sẽ thử gửi lại.
    if (fall != lastFall) {
      if (client.publish(T_STATUS, fall ? "FALL" : "NORMAL", true)) lastFall = fall;
    }

    // B3: đọc data Queue, B4: publish MQTT
    if (xQueueReceive(dataQueue, &d, pdMS_TO_TICKS(50)) == pdTRUE) {
      sendCntRecv++;
      sendHaveData = true;
      sendLast = d;
      dtostrf(d.svm, 1, 2, buf);  client.publish(T_ACCEL, buf);
      dtostrf(d.tilt, 1, 1, buf); client.publish(T_ANGLE, buf);
      sendCntPub += 2;
    }

    // Tới lượt thì in khối của Task truyền data
    servicePrintSend();
  }
}

// ====================== SETUP / LOOP ======================
void setup() {
  Serial.begin(115200);
  serialMutex = xSemaphoreCreateMutex();
  stateMutex  = xSemaphoreCreateMutex();
  printSendSem  = xSemaphoreCreateBinary();
  printReadSem  = xSemaphoreCreateBinary();
  printAlertSem = xSemaphoreCreateBinary();

  pinMode(LED_PIN, OUTPUT);
  buzzerInit();

  logPrint("\n========== KHOI DONG HE THONG RTOS ==========\n");

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    logPrint("[SETUP] Khong tim thay MPU6050!\n");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  logPrint("[SETUP] MPU6050 san sang\n");

#if WIFI_BEGIN_IN_SETUP
  WiFi.begin(ssid, password);
  logPrint("[SETUP] Bat dau ket noi WiFi (Task truyen data tren Core 0 se cho ket noi)\n");
#endif

  dataQueue = xQueueCreate(10, sizeof(SensorData));
  dataAlert = xQueueCreate(10, sizeof(SensorData));
  logPrint("[SETUP] Tao hang doi 'data Queue' (10 phan tu) : %s\n", dataQueue ? "OK" : "LOI");
  logPrint("[SETUP] Tao hang doi 'data Alert' (10 phan tu) : %s\n", dataAlert ? "OK" : "LOI");
  logPrint("[SETUP] Thu tu in Serial: TRUYEN DATA -> DOC DATA -> CANH BAO (dong bo bang semaphore)\n");

  //                        hàm           tên     stack prio handle core
  xTaskCreatePinnedToCore(TaskSendData, "Send",  8192, NULL, 1, NULL, 0);
  logPrint("[SETUP] Tao Task truyen data  -> Core 0, uu tien 1\n");
  xTaskCreatePinnedToCore(TaskReadData, "Read",  6144, NULL, 2, NULL, 1);
  logPrint("[SETUP] Tao Task doc data     -> Core 1, uu tien 2\n");
  xTaskCreatePinnedToCore(TaskAlert,    "Alert", 6144, NULL, 3, NULL, 1);
  logPrint("[SETUP] Tao Task canh bao     -> Core 1, uu tien 3\n");
  logPrint("=============================================\n");
}

void loop() {
  vTaskDelete(NULL);   // mọi việc đã do 3 Task đảm nhiệm
}
