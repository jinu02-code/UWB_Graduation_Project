// Anchor 1 코드 : TAG1/TAG2의 A1 대상 ranging 요청 처리
#include <SPI.h>
#include <dw3000.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ===== ESP32 <-> DW3000 핀 연결 =====
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_CS    4
#define PIN_RST   27
#define PIN_IRQ   34

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define DWT_TIME_UNITS (1.0 / 499.2e6 / 128.0)
#define SPEED_OF_LIGHT 299702547.0

#define MSG_TYPE_POLL  0xE0
#define MSG_TYPE_RESP  0xE1
#define MSG_TYPE_FINAL 0xE2

#define TAG1_ID 0x01  // Tag1_ID
#define TAG2_ID 0x02  // Tag2_ID

// ===== 다중 Anchor 구분 =====
// A1=0x01, A2=0x02, A3=0x03, A4=0x04
#define MY_ANCHOR_ID 0x01

// 응답/최종 프레임 대기시간
#define POLL_WAIT_MS   3000
#define FINAL_WAIT_MS  500   // 기존 1200ms: FINAL 실패 시 busy 유지시간 단축
#define TX_DONE_TIMEOUT_MS  80
#define BUSY_WATCHDOG_MS  900   // FINAL_WAIT보다 약간 길게 설정

// ===== Wi-Fi / Raspberry Pi HTTP 설정 =====
// 라즈베리파이와 앵커 ESP32는 같은 Wi-Fi에 연결
const char* WIFI_SSID = "WHKNU";
const char* WIFI_PASS = "";        
const char* RPI_SERVER_URL = "http://172.17.153.110:5000/uwb";  //http://라즈베리파이의 Wi-Fi의 ip 주소:5000/uwb

#define WIFI_CONNECT_TIMEOUT_MS 8000
#define WIFI_RETRY_INTERVAL_MS 5000
#define WIFI_RECONNECT_TRY_MS 500     // UWB loop 중 Wi-Fi 재연결 대기시간 단축
#define HTTP_POST_TIMEOUT_MS 300     // HTTP 실패 시 blocking 시간 단축

unsigned long last_wifi_retry_ms = 0;

static dwt_config_t config = {
  5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
  DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
  (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

static uint8_t tx_resp_msg[] = {MSG_TYPE_RESP, 0x00, 0x00, MY_ANCHOR_ID, 0, 0};

static uint8_t rx_buffer[128];
static uint16_t rx_len = 0;
static uint64_t last_rx_ts = 0;

float dist_hist[2][5] = {{0}};
int dist_idx[2] = {0, 0};
bool dist_full[2] = {false, false};

// ===== busy 상태 변수 =====
bool anchor_busy = false;
uint8_t active_tag_id = 0;
uint8_t active_seq = 0;
unsigned long busy_since_ms = 0;

void lockAnchor(uint8_t tag_id, uint8_t seq) {
  anchor_busy = true;
  active_tag_id = tag_id;
  active_seq = seq;
  busy_since_ms = millis();
}

void unlockAnchor() {
  anchor_busy = false;
  active_tag_id = 0;
  active_seq = 0;
  busy_since_ms = 0;
}

// ========================= Wi-Fi / HTTP =========================
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. Anchor IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed. UWB ranging will still run.");
  }
}

bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  unsigned long now = millis();
  if (now - last_wifi_retry_ms < WIFI_RETRY_INTERVAL_MS) return false;
  last_wifi_retry_ms = now;

  Serial.println("WiFi reconnect try...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_RECONNECT_TRY_MS) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi reconnected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("WiFi reconnect failed. HTTP skipped.");
  return false;
}

void sendToRaspberryPi(uint8_t tag_id,
                       uint8_t anchor_id,
                       uint8_t seq,
                       double distance,
                       float avg_distance,
                       float battery_voltage) {
  if (!ensureWiFiConnected()) {
    Serial.println("HTTP SKIP: WiFi not connected");
    return;
  }

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(HTTP_POST_TIMEOUT_MS);

  if (!http.begin(client, RPI_SERVER_URL)) {
    Serial.println("HTTP begin failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");

  String payload = "{";
  payload += "\"tag_id\":" + String(tag_id) + ",";
  payload += "\"anchor_id\":" + String(anchor_id) + ",";
  payload += "\"seq\":" + String(seq) + ",";
  payload += "\"distance\":" + String(distance, 3) + ",";
  payload += "\"avg_distance\":" + String(avg_distance, 3);

  if (battery_voltage >= 3.20f && battery_voltage <= 4.30f) {
    uint16_t battery_mV = (uint16_t)round(battery_voltage * 1000.0f);
    payload += ",\"battery_voltage\":" + String(battery_voltage, 3);
    payload += ",\"battery_mV\":" + String(battery_mV);
  }

  payload += "}";

  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    Serial.print("HTTP POST code: ");
    Serial.println(httpCode);
  } else {
    Serial.print("HTTP POST failed: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

// ========================= 기본 =========================
bool reinitDW3000Runtime() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_IRQ, INPUT);

  resetDW3000();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_CS);
  delay(2);

  if (!dwt_checkidlerc()) return false;

  dwt_softreset();
  delay(50);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) return false;

  dwt_configure(&config);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxtimeout(0);
  clearStatusFlags();
  return true;
}

void recoverDW3000Runtime(const char *reason) {
  Serial.print("ANCHOR RADIO RECOVER: ");
  Serial.println(reason);
  if (!reinitDW3000Runtime()) {
    Serial.println("ANCHOR RADIO RECOVER FAIL");
  }
}


void resetDW3000() {
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);
  pinMode(PIN_RST, INPUT_PULLUP);
  delay(50);
}

void clearStatusFlags() {
  dwt_write32bitreg(SYS_STATUS_ID,
    SYS_STATUS_RXFCG_BIT_MASK |
    SYS_STATUS_ALL_RX_ERR |
    SYS_STATUS_ALL_RX_TO |
    SYS_STATUS_TXFRS_BIT_MASK);
}

void initDW3000() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_IRQ, INPUT);

  resetDW3000();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_CS);

  delay(100);

  if (!dwt_checkidlerc()) {
    Serial.println("ANCHOR ERROR: idle check failed");
    while (1);
  }

  dwt_softreset();
  delay(100);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("ANCHOR ERROR: init failed");
    while (1);
  }

  dwt_configure(&config);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxtimeout(0);
}

// ========================= Timestamp =========================
uint64_t getTxTimestampU64() {
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  dwt_readtxtimestamp(ts_tab);
  for (int i = 4; i >= 0; i--) ts = (ts << 8) | ts_tab[i];
  return ts;
}

uint64_t getRxTimestampU64() {
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  dwt_readrxtimestamp(ts_tab);
  for (int i = 4; i >= 0; i--) ts = (ts << 8) | ts_tab[i];
  return ts;
}

uint32_t read_u32_le(const uint8_t *buf) {
  return ((uint32_t)buf[0]) |
         ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[3] << 24);
}

uint16_t read_u16_le(const uint8_t *buf) {
  return ((uint16_t)buf[0]) |
         ((uint16_t)buf[1] << 8);
}

// ========================= TX =========================
bool sendFrame(uint8_t *msg, uint16_t len) {
  clearStatusFlags();

  dwt_writetxdata(len, msg, 0);
  dwt_writetxfctrl(len, 0, 0);

  if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
    return false;
  }

  unsigned long t0 = millis();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
    if ((millis() - t0) > TX_DONE_TIMEOUT_MS) {
      clearStatusFlags();
      recoverDW3000Runtime("TX wait timeout");
      return false;
    }
    delayMicroseconds(50);
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  return true;
}

// ========================= RX =========================
bool receiveAnyFrame(uint16_t timeout_ms) {
  uint32_t status_reg;
  clearStatusFlags();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  unsigned long t0 = millis();

  while ((millis() - t0) <= timeout_ms) {
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
      rx_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
      dwt_readrxdata(rx_buffer, rx_len, 0);
      last_rx_ts = getRxTimestampU64();
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
      return true;
    }

    if (status_reg & SYS_STATUS_ALL_RX_ERR) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    if (status_reg & SYS_STATUS_ALL_RX_TO) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
  }
  return false;
}

bool waitForPoll(uint16_t timeout_ms, uint8_t &seq, uint8_t &tag_id, uint64_t &poll_rx_ts) {
  if (!receiveAnyFrame(timeout_ms)) return false;

  if (rx_buffer[0] == MSG_TYPE_POLL) {
    seq = rx_buffer[1];
    tag_id = rx_buffer[2];
    uint8_t target_anchor_id = rx_buffer[3];

    if (tag_id != TAG1_ID && tag_id != TAG2_ID) {
      Serial.print("IGNORE POLL: unsupported TAG ");
      Serial.println(tag_id);
      return false;
    }

    if (target_anchor_id != MY_ANCHOR_ID) {
      Serial.print("IGNORE POLL: target A");
      Serial.print(target_anchor_id);
      Serial.print(" / my A");
      Serial.println(MY_ANCHOR_ID);
      return false;
    }

    poll_rx_ts = last_rx_ts;
    return true;
  }
  return false;
}

bool waitForFinal(uint16_t timeout_ms, uint8_t seq, uint8_t tag_id, uint64_t &final_rx_ts) {
  unsigned long t0 = millis();

  while ((millis() - t0) <= timeout_ms) {
    uint16_t remain = (uint16_t)(timeout_ms - (millis() - t0));
    if (!receiveAnyFrame(remain)) return false;

    if (rx_buffer[0] == MSG_TYPE_FINAL &&
        rx_buffer[1] == seq &&
        rx_buffer[2] == tag_id &&
        rx_buffer[11] == MY_ANCHOR_ID) {
      final_rx_ts = last_rx_ts;
      return true;
    }

    if (rx_buffer[0] == MSG_TYPE_POLL) {
      Serial.print("BUSY: ignore POLL from TAG ");
      Serial.print(rx_buffer[2]);
      Serial.print(" while waiting FINAL from TAG ");
      Serial.println(tag_id);
    }
  }

  return false;
}

// ========================= 평균 =========================
int tagIdToIndex(uint8_t tag_id) {
  if (tag_id == TAG1_ID) return 0;
  if (tag_id == TAG2_ID) return 1;
  return -1;
}

float averageDistanceByTag(uint8_t tag_id, float new_val) {
  int idx_tag = tagIdToIndex(tag_id);
  if (idx_tag < 0) return new_val;

  dist_hist[idx_tag][dist_idx[idx_tag]] = new_val;
  dist_idx[idx_tag]++;

  if (dist_idx[idx_tag] >= 5) {
    dist_idx[idx_tag] = 0;
    dist_full[idx_tag] = true;
  }

  int n = dist_full[idx_tag] ? 5 : dist_idx[idx_tag];
  float sum = 0.0f;
  for (int i = 0; i < n; i++) sum += dist_hist[idx_tag][i];

  return sum / n;
}

// ========================= MAIN =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ANCHOR 1 BATTERY VOLTAGE ONLY v1 (TAG1/TAG2)");
  initWiFi();
  initDW3000();
  unlockAnchor();
}

void loop() {
  if (anchor_busy) {
    if (millis() - busy_since_ms > BUSY_WATCHDOG_MS) {
      Serial.println("BUSY watchdog -> unlock + recover");
      unlockAnchor();
      recoverDW3000Runtime("busy watchdog");
    }
    delay(2);
    return;
  }

  uint8_t seq = 0;
  uint8_t tag_id = 0;
  uint64_t poll_rx_ts = 0;

  if (!waitForPoll(POLL_WAIT_MS, seq, tag_id, poll_rx_ts)) {
    delay(10);
    return;
  }

  lockAnchor(tag_id, seq);

  Serial.print("LOCK TAG ");
  Serial.print(active_tag_id);
  Serial.print(" seq=");
  Serial.println(active_seq);

  tx_resp_msg[1] = seq;
  tx_resp_msg[2] = tag_id;
  tx_resp_msg[3] = MY_ANCHOR_ID;

  if (!sendFrame(tx_resp_msg, sizeof(tx_resp_msg))) {
    Serial.println("RESP FAIL");
    unlockAnchor();
    return;
  }

  uint64_t resp_tx_ts = getTxTimestampU64();

  clearStatusFlags();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  uint64_t final_rx_ts = 0;

  if (!waitForFinal(FINAL_WAIT_MS, seq, tag_id, final_rx_ts)) {
    Serial.print("FINAL timeout TAG ");
    Serial.println(tag_id);
    unlockAnchor();
    return;
  }

  uint32_t poll_tx_ts_tag = read_u32_le(&rx_buffer[3]);
  uint32_t resp_rx_ts_tag = read_u32_le(&rx_buffer[7]);

  float battery_voltage = -1.0f;
  uint16_t battery_mv = 0;

  Serial.print("FINAL rx_len=");
  Serial.print(rx_len);
  Serial.print(" raw_bat_mv_bytes=");
  if (rx_len > 15) Serial.print(rx_buffer[15]); else Serial.print(-1);
  Serial.print(",");
  if (rx_len > 16) Serial.println(rx_buffer[16]); else Serial.println(-1);

  if (rx_len >= 17) {
    battery_mv = read_u16_le(&rx_buffer[15]);

    // 말도 안 되는 바이트 오염값이 Raspberry Pi latest_battery를 덮어쓰는 것을 방지.
    if (battery_mv >= 3200 && battery_mv <= 4300) {
      battery_voltage = battery_mv / 1000.0f;
    }
  }

  uint64_t round1 = resp_rx_ts_tag - poll_tx_ts_tag;
  uint64_t reply1 = resp_tx_ts - poll_rx_ts;

  double tof_dtu = ((double)(round1 - reply1)) / 2.0;
  double distance = tof_dtu * DWT_TIME_UNITS * SPEED_OF_LIGHT;

  // 실제 가까운 거리(0.1~0.3m)는 살리고, 말도 안 되게 큰 값만 버림
  if (isnan(distance) || isinf(distance) || distance < -1.0 || distance > 20.0) {
    Serial.print("DROP INVALID DIST A");
    Serial.print(MY_ANCHOR_ID);
    Serial.print(" TAG");
    Serial.print(tag_id);
    Serial.print(" seq=");
    Serial.print(seq);
    Serial.print(" distance=");
    Serial.println(distance, 6);
    unlockAnchor();
    return;
  }

  float avg = averageDistanceByTag(tag_id, (float)distance);

  if (isnan(avg) || isinf(avg) || avg < -1.0 || avg > 20.0) {
    Serial.print("DROP INVALID AVG A");
    Serial.print(MY_ANCHOR_ID);
    Serial.print(" TAG");
    Serial.print(tag_id);
    Serial.print(" seq=");
    Serial.print(seq);
    Serial.print(" avg=");
    Serial.println(avg, 6);
    unlockAnchor();
    return;
  }

  Serial.print("A");
  Serial.print(MY_ANCHOR_ID);
  Serial.print("-TAG");
  Serial.print(tag_id);
  Serial.print(" distance: ");
  Serial.print(distance, 3);
  Serial.print(" m | avg: ");
  Serial.print(avg, 3);
  if (battery_voltage > 0.0f) {
    Serial.print(" | BAT_V=");
    Serial.print(battery_voltage, 3);
    Serial.println("V");
  } else {
    Serial.println(" | BAT_V=none");
  }

  // Raspberry Pi/AWS 연동용 파싱 라인
  // 형식: DATA,TAG_ID,ANCHOR_ID,SEQ,DISTANCE_M,AVG_DISTANCE_M,BATTERY_V
  Serial.print("DATA,");
  Serial.print(tag_id);
  Serial.print(",");
  Serial.print(MY_ANCHOR_ID);
  Serial.print(",");
  Serial.print(seq);
  Serial.print(",");
  Serial.print(distance, 3);
  Serial.print(",");
  Serial.print(avg, 3);
  if (battery_voltage > 0.0f) {
    Serial.print(",");
    Serial.println(battery_voltage, 3);
  } else {
    Serial.println(",none");
  }

  // UWB 측정은 끝났으므로 busy를 먼저 해제한 뒤 전송
  unlockAnchor();
  sendToRaspberryPi(tag_id, MY_ANCHOR_ID, seq, distance, avg, battery_voltage);
  delay(2);
}
