// TAG1 코드: motion interrupt 발생 시 A1/A2/A3/A4를 순차 ranging
#include <Wire.h>     // I2C 통신용
#include <SPI.h>      // ESP32와 DW3000 간 SPI 통신
#include <dw3000.h>   // DWM3000 UWB 제어
#include <esp_sleep.h>       // ESP32 deep sleep 제어
#include <driver/rtc_io.h>   // LIS3DH INT 핀으로 sleep wake 설정
#include <Adafruit_GFX.h>       // OLED 그래픽 기본 라이브러리
#include <Adafruit_SSD1306.h>   // SSD1306 OLED 제어

// ===== ESP32 <-> DWM3000 핀 연결 =====
#define PIN_SCK   18  // SPI Clock 핀
#define PIN_MISO  19  // SPI MISO 핀
#define PIN_MOSI  23  // SPI MOSI 핀
#define PIN_CS    4   // SPI Chip Select
#define PIN_RST   27  // DWM3000 Reset 핀
#define PIN_IRQ   34  // DWM3000 Interrupt 핀

#define TX_ANT_DLY 16385  // 송신 안테나 지연 보정값
#define RX_ANT_DLY 16385  // 수신 안테나 지연 보정값

#define MSG_TYPE_POLL  0xE0   // Tag가 보내는 POLL 메시지 타입
#define MSG_TYPE_RESP  0xE1   // Anchor가 보내는 RESP 메시지 타입
#define MSG_TYPE_FINAL 0xE2   // Tag가 보내는 FINAL 메시지 타입

#define TAG_ID 0x01
#define ANCHOR1_ID 0x01   // ANCHOR1_ID
#define ANCHOR2_ID 0x02   // ANCHOR2_ID
#define ANCHOR3_ID 0x03   // ANCHOR3_ID
#define ANCHOR4_ID 0x04   // ANCHOR4_ID

#define TAG_PHASE_OFFSET_MS

// 랜덤 백오프 범위(ms)
#define PRE_TX_BACKOFF_MIN  10   // 송신 전 랜덤 대기 최소 (충돌완화)
#define PRE_TX_BACKOFF_MAX  80   // 송신 전 랜덤 대기 최대 (커지면 충돌 감소, 지연 증가)
#define RETRY_BACKOFF_MIN  120   // 실패 후 재시도 최소 대기
#define RETRY_BACKOFF_MAX  450   // 실패 후 재시도 최대 대기

#define TX_DONE_TIMEOUT_MS      80
#define RADIO_RECOVER_RETRY_MS 120
#define MAX_CONSEC_FAILS         3

// motion wake / sleep behavior
#define RESP_TIMEOUT_MS       350   // RESP 수신 대기 시간
#define QUIET_TO_SLEEP_MS    5000   // 마지막 움직임 후 sleep 복귀 시간
#define MAX_AWAKE_MS        15000   // 최대 awake 시간
#define POST_WAKE_STABILIZE_MS 80   // wake 후 안정화 지연

// HYBRID_RANDOM access tuning
#define HYBRID_INITIAL_BACKOFF_MIN_MS  80   // wake 직후 첫 시도 최소 지연
#define HYBRID_INITIAL_BACKOFF_MAX_MS 500   // wake 직후 첫 시도 최대 지연
#define HYBRID_ATTEMPT_GAP_MIN_MS      80   // 성공 후 다음 Anchor까지 최소 간격
#define HYBRID_ATTEMPT_GAP_MAX_MS     180   // 성공 후 다음 Anchor까지 최대 간격

/* ===== ESP32 <-> 배터리 전압 측정 핀 연결 =====*/
#define BATTERY_PIN 32  // 배터리 전압 ADC 핀

const float BAT_R1 = 10000.0;       // 배터리 분압 상단 저항
const float BAT_R2 = 10000.0;       // 배터리 분압 하단 저항
const float BAT_VREF = 3.3;         // ADC 기준 전압
const float BAT_ADC_MAX = 4095.0;   // 12-bit ADC 최대값
const float BAT_CALIBRATION = 1.08; // 멀티미터와 비교해서 맞춘 보정값

/* ===== ESP32 <-> SSD1306 핀 연결 ====== */
#define OLED_SDA_PIN  16    // OLED I2C SDA
#define OLED_SCL_PIN  17    // OLED I2C SCL
#define SCREEN_WIDTH  128   // OLED 가로 해상도
#define SCREEN_HEIGHT 64    // OLED 세로 해상도
#define OLED_ADDR     0x3C  // OLED I2C 주소

bool oledReady = false;         // OLED 초기화 성공 여부
TwoWire OLEDWire = TwoWire(1);  // OLED 전용 I2C 버스
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &OLEDWire, -1); // OLED 화면 객체

/* ===== ESP32 <-> LIS3DH 핀 연결 ===== */
#define SDA_PIN     21    // LIS3DH I2C SDA
#define SCL_PIN     22    // LIS3DH I2C SCL
#define PIN_LIS_INT 33    // LIS3DH INT1 핀
#define LIS_ADDR    0x18  // LIS3DH I2C 주소

// LIS3DH registers
#define REG_WHO_AM_I      0x0F  // LIS3DH ID 확인 레지스터
#define REG_CTRL1         0x20
#define REG_CTRL2         0x21
#define REG_CTRL3         0x22
#define REG_CTRL4         0x23
#define REG_CTRL5         0x24
#define REG_CTRL6         0x25
#define REG_REFERENCE     0x26  // interrupt 안정화 (기준값)
#define REG_INT1_CFG      0x30  // INT1 조건 설정
#define REG_INT1_SRC      0x31  // INT1 발생 원인 확인/clear
#define REG_INT1_THS      0x32  // 움직임 threshold (낮으면 민감/높으면 둔감)
#define REG_INT1_DURATION 0x33  // interrupt 지속 조건

// LIS3DH interrupt tuning
// 이 값으로 먼저 안정화한 뒤 필요하면 ODR/threshold 튜닝
#define LIS_CTRL1_VALUE    0x57  // LIS3DH ODR/축 활성 설정
#define LIS_CTRL2_VALUE    0x09  // 정적 중력 영향 제거 목적
#define LIS_CTRL3_VALUE    0x40  // INT1 핀활성화
#define LIS_CTRL4_VALUE    0x00  // 민감도 영향
#define LIS_CTRL5_VALUE    0x08  // interrupt latch 설정
#define LIS_CTRL6_VALUE    0x00  // INT2 관련 설정
#define LIS_INT1_THS_VALUE 0x08  // 움직임 감지 임계값 (작으면 더 민감)
#define LIS_INT1_DUR_VALUE 0x00  // 감지 지속 시간 (즉시 감지에 가까움)
#define LIS_INT1_CFG_VALUE 0x2A  // X/Y/Z 움직임 조건

// DWM3000 설정값
static dwt_config_t config = {
  5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
  DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
  (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

// Anchor에 보내는 POLL
static uint8_t tx_poll_msg[]  = {MSG_TYPE_POLL, 0x00, TAG_ID, 0, 0, 0}; 
// timestamp와 배터리 전압 포함 FINAL
static uint8_t tx_final_msg[] = {
  MSG_TYPE_FINAL, 0x00, TAG_ID,
  0,0,0,0,
  0,0,0,0,
  0,0,0,0,
  0,0,0,0
};

static uint8_t current_target_anchor_id = ANCHOR1_ID; // 현재 ranging할 Anchor ID
static uint8_t rx_buffer[128];            // RESP 수신 버퍼
RTC_DATA_ATTR uint8_t frame_seq_nb = 0;   // ranging sequence 번호
RTC_DATA_ATTR uint32_t bootCount = 0;     // deep sleep 이후 boot 횟수 유지
static uint16_t rx_len = 0;               // 수신 프레임 길이
static uint64_t last_rx_ts = 0;           // RESP 수신 timestamp

static int last_slot = -1;
static unsigned long next_try_ms = 0;       // 다음 ranging 시도 가능 시간
uint8_t next_anchor_offset = 0;             // 다음 Anchor 순서 index
static uint8_t consecutive_fail_count = 0;  // 연속 실패 횟수
static bool radio_ready = false;            // DW3000 사용 가능 여부
static bool awake_mode = false;             // 현재 깨어나서 ranging 중인지
static unsigned long awakeStartMs = 0;      // 깨어난 시작 시간
static unsigned long lastMotionIrqMs = 0;   // 마지막 움직임 감지 시간
volatile bool motion_irq_flag = false;      // interrupt 발생 표시

struct BatteryReading {
  float raw;
  float adcVoltage;
  float batteryVoltage;
  int percent;
};

// ===== 배터리 전압 1회 전송 제어 =====
// 움직임으로 deep sleep에서 깨어난 직후 배터리를 1번만 측정하고,
// 깨어있는 동안 첫 번째로 성공한 FINAL 메시지에만 이 값을 실어 보낸다.
static BatteryReading wakeBattery;
static bool wakeBatteryReady = false;
static bool wakeBatterySentToAnchor = false;

/* ========== ========== ===== Tag 함수 ===== ========== ========== */
// ========================= 배터리 출력 =========================
// 배터리 전압 측정
BatteryReading readBattery() {
  long sum = 0;

  // ADC 값을 여러 번 읽어서 평균내기
  for (int i = 0; i < 8; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(1);
  }

  BatteryReading b;
  b.raw = sum / 8.0;
  b.adcVoltage = (b.raw / BAT_ADC_MAX) * BAT_VREF;

  // 1:1 분압 기준: ADC 전압의 2배가 실제 배터리 전압
  b.batteryVoltage = b.adcVoltage * ((BAT_R1 + BAT_R2) / BAT_R2);

  // 멀티미터 기준 보정값 적용
  b.batteryVoltage *= BAT_CALIBRATION;
  b.percent = voltageToPercent(b.batteryVoltage);

  return b;
}

// 배터리 퍼센트 계산
int voltageToPercent(float voltage) {
  if (voltage >= 4.15) return 100;
  if (voltage <= 3.30) return 0;

  return (int)(((voltage - 3.30) / (4.15 - 3.30)) * 100.0);
}

// OLED 준비
void initOLED() {
  OLEDWire.begin(OLED_SDA_PIN, OLED_SCL_PIN, 100000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    oledReady = false;
    return;
  }

  oledReady = true;
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TAG1 OLED READY");
  display.display();
}

// 배터리 화면 업데이트
void updateBatteryOLED(const BatteryReading &b) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(WHITE);

  // 하단 태그 표시
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print("TAG1 BATTERY");

  // 가운데 배터리 퍼센트 크게 표시
  display.setTextSize(3);
  display.setCursor(18, 22);
  display.print(b.percent);
  display.print("%");

  display.display();
}

// sleep 전 OLED sleep 상태 표시
void showSleepOLED() {
  if (!oledReady) return;

  BatteryReading b = readBattery();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.println("TAG1 SLEEP");
  display.setTextSize(3);
  display.setCursor(18, 22);
  display.print(b.percent);
  display.print("%");
  display.display();
  delay(250);
}

// 초기 배터리 읽고 Serial/OLED 출력
void printBatteryStatus() {
  BatteryReading b = readBattery();

  Serial.print("TAG1 BAT=");
  Serial.print(b.batteryVoltage, 2);
  Serial.print("V PCT=");
  Serial.print(b.percent);
  Serial.println("%");

  updateBatteryOLED(b);   // OLED 화면 표시
}

// min~max 랜덤값 반환으로 충돌 완화 대기시간 생성
unsigned long randBetween(unsigned long min_v, unsigned long max_v) {
  if (max_v <= min_v) return min_v;
  return (unsigned long)random((long)min_v, (long)(max_v + 1));
}

// 움직임 interrupt 표시
void IRAM_ATTR onMotionIRQ() {
  motion_irq_flag = true;
}
// 실패시 다음 시도 시간을 랜덤 지연
void scheduleRetryBackoff() {
  next_try_ms = millis() + randBetween(RETRY_BACKOFF_MIN, RETRY_BACKOFF_MAX);
}

// LIS3DH 제어 (I2C register 읽기)
bool readReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(LIS_ADDR, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

// LIS3DH 제어 (I2C register 쓰기)
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

// latched interrupt 해제
uint8_t clearLIS3DHInterrupt() {
  uint8_t src = 0;
  readReg(REG_INT1_SRC, src);  // latched INT1 clear
  return src;
}

// 움직임 감지 설정
bool initLIS3DHMotionInterrupt() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);

  uint8_t whoami = 0;
  if (!readReg(REG_WHO_AM_I, whoami)) {
    Serial.println("LIS3DH not found");
    return false;
  }
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoami, HEX);
  if (whoami != 0x33) {
    Serial.println("Wrong LIS3DH response");
    return false;
  }

  if (!writeReg(REG_CTRL1, LIS_CTRL1_VALUE)) return false;
  if (!writeReg(REG_CTRL2, LIS_CTRL2_VALUE)) return false;
  if (!writeReg(REG_CTRL3, LIS_CTRL3_VALUE)) return false;
  if (!writeReg(REG_CTRL4, LIS_CTRL4_VALUE)) return false;
  if (!writeReg(REG_CTRL5, LIS_CTRL5_VALUE)) return false;
  if (!writeReg(REG_CTRL6, LIS_CTRL6_VALUE)) return false;
  if (!writeReg(REG_INT1_THS, LIS_INT1_THS_VALUE)) return false;
  if (!writeReg(REG_INT1_DURATION, LIS_INT1_DUR_VALUE)) return false;

  // HP filter reference set + old latched interrupt clear
  uint8_t dummy = 0;
  readReg(REG_REFERENCE, dummy);
  clearLIS3DHInterrupt();

  if (!writeReg(REG_INT1_CFG, LIS_INT1_CFG_VALUE)) return false;
  clearLIS3DHInterrupt();
  return true;
}

// deep sleep wake 준비
void prepareWakePinForSleep() {
  detachInterrupt(digitalPinToInterrupt(PIN_LIS_INT));
  pinMode(PIN_LIS_INT, INPUT);

  // ext0 wake: RTC GPIO only, HIGH level wake
  rtc_gpio_pullup_dis((gpio_num_t)PIN_LIS_INT);
  rtc_gpio_pulldown_en((gpio_num_t)PIN_LIS_INT);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_LIS_INT, 1);
}

// interrupt clear 후 핀 LOW인지 확인 (바로 다시 깨는 문제 방지)
bool readyToSleepNow() {
  clearLIS3DHInterrupt();
  delay(5);
  return (digitalRead(PIN_LIS_INT) == LOW);
}

// OLED 표시, wake pin 설정, sleep 시작 (절전모드 진입)
void goDeepSleepNow(const char *reason) {
  Serial.print("SLEEP: ");
  Serial.println(reason);
  showSleepOLED();

  prepareWakePinForSleep();

  if (!readyToSleepNow()) {
    Serial.println("INT still HIGH -> stay awake");
    awake_mode = true;
    awakeStartMs = millis();
    lastMotionIrqMs = millis();

    rtc_gpio_deinit((gpio_num_t)PIN_LIS_INT);
    pinMode(PIN_LIS_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_LIS_INT), onMotionIRQ, RISING);

    if (!radio_ready) {
      delay(POST_WAKE_STABILIZE_MS);
      initDW3000();
    }
    return;
  }

  Serial.flush();
  esp_deep_sleep_start();
}

// DWM3000 reset
void resetDW3000() {
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);
  pinMode(PIN_RST, INPUT_PULLUP);
  delay(50);
}

// 이전 송수신 상태 제거
void clearStatusFlags() {
  dwt_write32bitreg(SYS_STATUS_ID,
    SYS_STATUS_RXFCG_BIT_MASK |
    SYS_STATUS_ALL_RX_ERR |
    SYS_STATUS_ALL_RX_TO |
    SYS_STATUS_TXFRS_BIT_MASK);
}

// DWM3000 재초기화 (오류복구)
bool reinitDW3000Runtime() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_IRQ, INPUT);

  resetDW3000();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_CS);
  delay(20);

  if (!dwt_checkidlerc()) return false;

  dwt_softreset();
  delay(50);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) return false;

  dwt_configure(&config);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  clearStatusFlags();
  radio_ready = true;
  return true;
}

// UWB 오류 회복
void recoverDW3000Runtime(const char *reason) {
  Serial.print("RADIO RECOVER: ");
  Serial.println(reason);
  if (!reinitDW3000Runtime()) {
    Serial.println("RADIO RECOVER FAIL");
    radio_ready = false;
  }
  next_try_ms = millis() + RADIO_RECOVER_RETRY_MS;
}

// 실패 카운트 증가, 필요 시 recover (실패관리)
void markFail(const char *reason, bool recover_now = false) {
  consecutive_fail_count++;
  if (recover_now || consecutive_fail_count >= MAX_CONSEC_FAILS) {
    recoverDW3000Runtime(reason);
    consecutive_fail_count = 0;
  } else {
    scheduleRetryBackoff();
  }
}

// 실패 카운트 0으로 초기화 (정상상태복귀)
void markSuccess() {
  consecutive_fail_count = 0;
}

// SPI, DW3000 초기화
bool initDW3000() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_IRQ, INPUT);

  resetDW3000();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_CS);
  delay(100);

  if (!dwt_checkidlerc()) {
    Serial.println("TAG1 ERROR");
    radio_ready = false;
    return false;
  }

  dwt_softreset();
  delay(100);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("INIT FAIL");
    radio_ready = false;
    return false;
  }

  dwt_configure(&config);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  clearStatusFlags();
  radio_ready = true;
  return true;
}

// FINAL에 넣을 POLL TX 시각
uint64_t getTxTimestampU64() {
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  dwt_readtxtimestamp(ts_tab);
  for (int i = 4; i >= 0; i--) ts = (ts << 8) | ts_tab[i];
  return ts;
}

// FINAL에 넣을 RESP RX 시각
uint64_t getRxTimestampU64() {
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  dwt_readrxtimestamp(ts_tab);
  for (int i = 4; i >= 0; i--) ts = (ts << 8) | ts_tab[i];
  return ts;
}

// FINAL timestamp 삽입
void write_u32_le(uint8_t *buf, uint32_t val) {
  buf[0] = val & 0xFF;
  buf[1] = (val >> 8) & 0xFF;
  buf[2] = (val >> 16) & 0xFF;
  buf[3] = (val >> 24) & 0xFF;
}

// FINAL 배터리 전압 삽입
void write_u16_le(uint8_t *buf, uint16_t val) {
  buf[0] = val & 0xFF;
  buf[1] = (val >> 8) & 0xFF;
}

// POLL, FINAL 전송
bool sendFrame(uint8_t *msg, uint16_t len) {
  if (!radio_ready) return false;

  clearStatusFlags();
  dwt_writetxdata(len, msg, 0);
  dwt_writetxfctrl(len, 0, 0);

  if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) return false;

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

// 유효한 RESP 수신
bool waitForResp(uint16_t timeout) {
  if (!radio_ready) return false;

  uint32_t status;
  clearStatusFlags();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  unsigned long t0 = millis();

  while (millis() - t0 < timeout) {
    status = dwt_read32bitreg(SYS_STATUS_ID);

    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
      rx_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
      dwt_readrxdata(rx_buffer, rx_len, 0);
      last_rx_ts = getRxTimestampU64();
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

      if (rx_buffer[0] == MSG_TYPE_RESP &&
          rx_buffer[1] == frame_seq_nb &&
          rx_buffer[2] == TAG_ID &&
          rx_buffer[3] == current_target_anchor_id) {
        return true;
      }
    }

    if (status & SYS_STATUS_ALL_RX_ERR) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    if (status & SYS_STATUS_ALL_RX_TO) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
  }

  return false;
}

// awake 시간 연장
void consumeMotionIrqIfAny() {
  if (motion_irq_flag || digitalRead(PIN_LIS_INT) == HIGH) {
    motion_irq_flag = false;
    uint8_t src = clearLIS3DHInterrupt();
    if (src & 0x40) {  // IA bit
      lastMotionIrqMs = millis();
      Serial.print("MOVE IRQ SRC=0x");
      Serial.println(src, HEX);
    }
  }
}

// 움직임 후 측정 준비
void setupAwakeModeAfterMotionWake() {
  awake_mode = true;
  awakeStartMs = millis();
  lastMotionIrqMs = millis();
  // HYBRID initial backoff: 여러 태그가 동시에 wake 될 때 시작 시점 분산
  next_try_ms = millis() + TAG_PHASE_OFFSET_MS + randBetween(HYBRID_INITIAL_BACKOFF_MIN_MS, HYBRID_INITIAL_BACKOFF_MAX_MS);
  last_slot = -1;
  motion_irq_flag = false;

  rtc_gpio_deinit((gpio_num_t)PIN_LIS_INT);  // wake pin -> normal GPIO 복귀
  pinMode(PIN_LIS_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_LIS_INT), onMotionIRQ, RISING);

  clearLIS3DHInterrupt();
  delay(POST_WAKE_STABILIZE_MS);

  // deep sleep에서 움직임으로 깨어난 직후 배터리 전압을 1번만 측정한다.
  // 이후 깨어있는 동안에는 readBattery()를 반복 호출하지 않는다.
  wakeBattery = readBattery();
  wakeBatteryReady = true;
  wakeBatterySentToAnchor = false;

  Serial.print("TAG1 WAKE BAT=");
  Serial.print(wakeBattery.batteryVoltage, 2);
  Serial.print("V PCT=");
  Serial.print(wakeBattery.percent);
  Serial.println("%");
  updateBatteryOLED(wakeBattery);

  if (!initDW3000()) {
    Serial.println("DW3000 init failed after wake");
    goDeepSleepNow("dw init fail");
    return;
  }

  Serial.println("TAG1 AWAKE by motion");
}

// sleep 복귀 판단
bool awakeWindowExpired() {
  unsigned long now = millis();
  if ((now - lastMotionIrqMs) >= QUIET_TO_SLEEP_MS) return true;
  if ((now - awakeStartMs) >= MAX_AWAKE_MS) return true;
  return false;
}

// Anchor 선택, POLL, RESP, FINAL, 배터리 포함 전송
void doOneRangingAttemptIfPossible() {
  consumeMotionIrqIfAny();

  if (!radio_ready) {
    recoverDW3000Runtime("radio not ready");
    return;
  }

  if (millis() < next_try_ms) return;

  // 내부에서 다음 코드로 Anchor를 순서대로 선택
  if (next_anchor_offset == 0) current_target_anchor_id = ANCHOR1_ID;
  else if (next_anchor_offset == 1) current_target_anchor_id = ANCHOR2_ID;
  else if (next_anchor_offset == 2) current_target_anchor_id = ANCHOR3_ID;
  else current_target_anchor_id = ANCHOR4_ID;

  unsigned long pre_backoff = randBetween(PRE_TX_BACKOFF_MIN, PRE_TX_BACKOFF_MAX);
  delay(pre_backoff);
  consumeMotionIrqIfAny();

  Serial.print("TAG1 HYBRID SEND POLL to A");
  Serial.print(current_target_anchor_id);
  Serial.print(" seq=");
  Serial.println(frame_seq_nb);

  tx_poll_msg[1] = frame_seq_nb;
  tx_poll_msg[2] = TAG_ID;
  tx_poll_msg[3] = current_target_anchor_id;

  tx_final_msg[1] = frame_seq_nb;
  tx_final_msg[2] = TAG_ID;
  tx_final_msg[11] = current_target_anchor_id;

  bool ok = false;

  if (!sendFrame(tx_poll_msg, sizeof(tx_poll_msg))) {
    Serial.println("TAG1 POLL fail");
    markFail("POLL fail");
  } else {
    uint64_t poll_tx_ts = getTxTimestampU64();

    if (!waitForResp(RESP_TIMEOUT_MS)) {
      Serial.print("TAG1 RESP timeout from A");
      Serial.println(current_target_anchor_id);
      markFail("RESP timeout");
    } else {
      uint64_t resp_rx_ts = last_rx_ts;

      write_u32_le(&tx_final_msg[3], (uint32_t)poll_tx_ts);
      write_u32_le(&tx_final_msg[7], (uint32_t)resp_rx_ts);
     
      // 배터리 전압은 wake 직후 측정한 값을 첫 번째 성공 FINAL에만 실어 보낸다.
      // 이미 한 번 보낸 뒤에는 0mV를 넣어서 Anchor가 battery 없음으로 처리하게 한다.
      bool includeBatteryThisFinal = (wakeBatteryReady && !wakeBatterySentToAnchor);
      uint16_t batteryMv = 0;
      if (includeBatteryThisFinal) {
        batteryMv = (uint16_t)constrain((int)round(wakeBattery.batteryVoltage * 1000.0f), 0, 65535);
      }

      write_u16_le(&tx_final_msg[15], batteryMv);
      tx_final_msg[17] = 0;
      tx_final_msg[18] = 0;

      Serial.print("TAG1 FINAL_SIZE=");
      Serial.print(sizeof(tx_final_msg));
      Serial.print(" UWB_BAT_V=");
      Serial.print(batteryMv);
      Serial.print("mV, sent_once=");
      Serial.println(wakeBatterySentToAnchor ? "yes" : "no");

      delay(2);  // FINAL 전 짧은 안정화만 유지

      if (!sendFrame(tx_final_msg, sizeof(tx_final_msg))) {
        Serial.println("TAG1 FINAL fail");
        markFail("FINAL fail", true);
      } else {
        Serial.print("TAG1-A");
        Serial.print(current_target_anchor_id);
        Serial.println(" OK");

        if (includeBatteryThisFinal) {
          wakeBatterySentToAnchor = true;
          Serial.print("TAG1 BAT SENT ONCE=");
          Serial.print(wakeBattery.batteryVoltage, 2);
          Serial.print("V PCT=");
          Serial.print(wakeBattery.percent);
          Serial.println("%");
        } else {
          Serial.println("TAG1 BAT SKIP: already sent in this awake cycle");
        }

        // OLED는 wake 직후 측정한 값을 유지한다.
        if (wakeBatteryReady) updateBatteryOLED(wakeBattery);

        markSuccess();
        ok = true;
      }
    }
  }

  frame_seq_nb++;
  next_anchor_offset = (next_anchor_offset + 1) % 4;

  if (ok) {
    next_try_ms = millis() + randBetween(HYBRID_ATTEMPT_GAP_MIN_MS, HYBRID_ATTEMPT_GAP_MAX_MS);
  } else {
    next_try_ms = millis() + randBetween(RETRY_BACKOFF_MIN, RETRY_BACKOFF_MAX);
  }
}

// wake 원인 출력
void printWakeReason() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.print("Wake reason: ");
  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("EXT0 / LIS3DH motion"); break;
    case ESP_SLEEP_WAKEUP_UNDEFINED: Serial.println("Power on / reset"); break;
    default:
      Serial.println((int)cause);
      break;
  }
}

void setup() {
  Serial.begin(115200);   //Serial 시작
  delay(300);             // 0.3초 대기
  Serial.println("TAG1 UWB FINAL_MV ORDERED_ANCHOR v2 - FINAL_SIZE=" + String(sizeof(tx_final_msg)));

  pinMode(BATTERY_PIN, INPUT);    // 배터리 ADC 핀 설정
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);   // 배터리 전압 측정

  initOLED();             // OLED 초기화
  printBatteryStatus();   // 배터리 상태 출력

  bootCount++;      // 부팅 횟수 증가
  randomSeed((uint32_t)micros() ^ ((uint32_t)TAG_ID << 16) ^ bootCount); // 랜덤 초기화

  printWakeReason();    // Wake-up 원인 출력
  Serial.print("Boot count = ");
  Serial.println(bootCount);

  if (!initLIS3DHMotionInterrupt()) {
    // LIS3DH 움직임 센서 초기화
    Serial.println("LIS3DH init/config failed");
    delay(1000);
    return;
  }

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause(); // Wake-up 원인 확인
  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    setupAwakeModeAfterMotionWake();  // 움직임으로 깨어났으면 UWB 측정 모드 진입
  } else {
    goDeepSleepNow("wait motion");  // 일반 전원/reset이면 deep sleep 진입
  }
}

void loop() {
  if (!awake_mode) {
    delay(100);
    return;
  }

  consumeMotionIrqIfAny();
  doOneRangingAttemptIfPossible();

  if (awakeWindowExpired()) {
    awake_mode = false;
    radio_ready = false;
    goDeepSleepNow("quiet timeout");
    return;
  }

  delay(2);
}
