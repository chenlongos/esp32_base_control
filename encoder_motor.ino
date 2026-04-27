// ============================================================
//  ESP32-C3 SuperMini + DRV8833 x2 + 霍尔编码�?TT 马达
//  UART 控制协议 V1.0  |  波特�?115200
// ============================================================

// --- 引脚定义 ---
#define LED_PIN    8
#define IN1_PIN    4
#define IN2_PIN    3
#define ENC_A      5
#define ENC_B      6
#define IN1_PIN_2  1
#define IN2_PIN_2  2
#define ENC_A_2    7
#define ENC_B_2   10

// --- PWM 默认参数 ---
#define PWM_BITS          8
#define PWM_FREQ_DEFAULT  20000
#define PPR_DEFAULT       4680

// --- 协议：命令字 ---
#define CMD_INIT        0x01
#define CMD_CONFIG      0x02
#define CMD_SET_SPEED   0x10
#define CMD_STOP        0x11
#define CMD_BRAKE       0x12
#define CMD_GET_RPM     0x20
#define CMD_GET_STATUS  0x21
#define CMD_RESET       0xFF

// --- 协议：响应字 ---
#define RSP_ACK         0x80
#define RSP_NACK        0x81
#define RSP_RPM_DATA    0x90
#define RSP_STATUS      0x91

// --- 错误�?---
#define ERR_WRONG_STATE   0x01
#define ERR_BAD_CHECKSUM  0x02
#define ERR_INVALID_PARAM 0x03
#define ERR_UNKNOWN_CMD   0x04

// --- 帧头 ---
#define FRAME_H1  0xAA
#define FRAME_H2  0x55

// ============================================================
//  系统状�?// ============================================================
enum SysState : uint8_t { UNINIT = 0, IDLE = 1, READY = 2, RUNNING = 3, SYS_ERROR = 4 };
SysState sysState = UNINIT;

// --- 配置（可�?CONFIG 命令修改�?--
uint16_t cfg_ppr      = PPR_DEFAULT;
uint16_t cfg_pwm_freq = PWM_FREQ_DEFAULT;

// ============================================================
//  编码器（中断，需 IRAM�?// ============================================================
volatile long encoderCount  = 0;
volatile long encoderCount2 = 0;

void IRAM_ATTR encA_ISR()  { encoderCount  += (digitalRead(ENC_A)   != digitalRead(ENC_B))   ? 1 : -1; }
void IRAM_ATTR encB_ISR()  { encoderCount  += (digitalRead(ENC_A)   == digitalRead(ENC_B))   ? 1 : -1; }
void IRAM_ATTR encA2_ISR() { encoderCount2 += (digitalRead(ENC_A_2) != digitalRead(ENC_B_2)) ? 1 : -1; }
void IRAM_ATTR encB2_ISR() { encoderCount2 += (digitalRead(ENC_A_2) == digitalRead(ENC_B_2)) ? 1 : -1; }

// --- RPM 计算 ---
unsigned long lastRpmTime = 0;
long lastCnt1 = 0, lastCnt2 = 0;
int16_t rpm1 = 0, rpm2 = 0;

// ============================================================
//  LED 状�?// ============================================================
unsigned long lastLedTime = 0;
uint8_t ledPhase = 0;

// ============================================================
//  串口接收状态机
// ============================================================
enum RxState : uint8_t { RX_H1, RX_H2, RX_CMD, RX_LEN, RX_PAYLOAD, RX_CHK };
RxState rxState = RX_H1;
uint8_t rxCmd, rxLen, rxIdx;
uint8_t rxBuf[16];

// ============================================================
//  初始化辅�?// ============================================================
void initMotorPins(int p1, int p2) {
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  analogWriteFrequency(p1, cfg_pwm_freq);
  analogWriteFrequency(p2, cfg_pwm_freq);
  analogWriteResolution(p1, PWM_BITS);
  analogWriteResolution(p2, PWM_BITS);
  analogWrite(p1, 0);
  analogWrite(p2, 0);
}

void initPWM() {
  initMotorPins(IN1_PIN,   IN2_PIN);
  initMotorPins(IN1_PIN_2, IN2_PIN_2);
}

void initEncoders() {
  pinMode(ENC_A,   INPUT); pinMode(ENC_B,   INPUT);
  pinMode(ENC_A_2, INPUT); pinMode(ENC_B_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A),   encA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B),   encB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_2), encA2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_2), encB2_ISR, CHANGE);
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // 上电默认灭（active low）
  initPWM();
  initEncoders();
  lastRpmTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 接收串口数据
  while (Serial.available()) {
    processByte((uint8_t)Serial.read());
  }

  // �?100ms 更新 RPM
  if (now - lastRpmTime >= 100) {
    noInterrupts();
    long s1 = encoderCount, s2 = encoderCount2;
    interrupts();
    float dt = (now - lastRpmTime) / 1000.0f;
    rpm1 = (int16_t)(((s1 - lastCnt1) / (float)cfg_ppr) / dt * 60.0f);
    rpm2 = (int16_t)(((s2 - lastCnt2) / (float)cfg_ppr) / dt * 60.0f);
    lastCnt1 = s1; lastCnt2 = s2;
    lastRpmTime = now;
  }

  updateLed(now);
}

// ============================================================
//  LED 显示逻辑
// ============================================================
void updateLed(unsigned long now) {
  switch (sysState) {
    case UNINIT:
      digitalWrite(LED_PIN, HIGH);  // active low：HIGH=灭
      break;
    case IDLE:
      digitalWrite(LED_PIN, LOW);   // active low：LOW=亮（常亮）
      break;
    case READY:
      if (now - lastLedTime >= 500) {
        lastLedTime = now;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // 慢闪 500ms
      }
      break;
    case RUNNING:
      if (now - lastLedTime >= 100) {
        lastLedTime = now;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // 快闪 100ms
      }
      break;
    case SYS_ERROR: {
      // 双闪：亮100 �?00 �?00 �?00
      unsigned long interval = (ledPhase < 3) ? 100 : 700;
      if (now - lastLedTime >= interval) {
        lastLedTime = now;
        ledPhase = (ledPhase + 1) % 4;
        digitalWrite(LED_PIN, (ledPhase % 2 == 0) ? HIGH : LOW);
      }
      break;
    }
  }
}

// ============================================================
//  串口接收状态机
// ============================================================
void processByte(uint8_t b) {
  switch (rxState) {
    case RX_H1:
      if (b == FRAME_H1) rxState = RX_H2;
      break;
    case RX_H2:
      rxState = (b == FRAME_H2) ? RX_CMD : RX_H1;
      break;
    case RX_CMD:
      rxCmd = b;
      rxState = RX_LEN;
      break;
    case RX_LEN:
      rxLen = b;
      rxIdx = 0;
      rxState = (rxLen == 0) ? RX_CHK : RX_PAYLOAD;
      break;
    case RX_PAYLOAD:
      if (rxIdx < sizeof(rxBuf)) rxBuf[rxIdx++] = b;
      if (rxIdx >= rxLen) rxState = RX_CHK;
      break;
    case RX_CHK: {
      uint8_t chk = rxCmd ^ rxLen;
      for (uint8_t i = 0; i < rxLen; i++) chk ^= rxBuf[i];
      if (chk == b) handleCommand(rxCmd, rxBuf, rxLen);
      else          sendNack(rxCmd, ERR_BAD_CHECKSUM);
      rxState = RX_H1;
      break;
    }
  }
}

// ============================================================
//  命令处理
// ============================================================
void handleCommand(uint8_t cmd, uint8_t *p, uint8_t len) {
  switch (cmd) {

    case CMD_INIT:
      motorCoast(0); motorCoast(1);
      encoderCount = 0; encoderCount2 = 0;
      lastCnt1 = 0;   lastCnt2 = 0;
      sysState = IDLE;
      sendAck(cmd);
      break;

    case CMD_CONFIG:
      if (sysState != IDLE) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 4)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      cfg_ppr      = (uint16_t)(p[0] << 8 | p[1]);
      cfg_pwm_freq = (uint16_t)(p[2] << 8 | p[3]);
      if (cfg_ppr == 0 || cfg_pwm_freq == 0) { sendNack(cmd, ERR_INVALID_PARAM); return; }
      initPWM();
      sysState = READY;
      sendAck(cmd);
      break;

    case CMD_SET_SPEED: {
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 3)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      uint8_t mid  = p[0];
      int16_t spd  = (int16_t)((p[1] << 8) | p[2]);
      spd = (int16_t)constrain((int)spd, -255, 255);
      if (mid > 1) { sendNack(cmd, ERR_INVALID_PARAM); return; }
      setMotorSpeed(mid, spd);
      sysState = RUNNING;
      sendAck(cmd);
      break;
    }

    case CMD_STOP:
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 1)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      if (p[0] > 1)         { sendNack(cmd, ERR_INVALID_PARAM); return; }
      motorCoast(p[0]);
      sendAck(cmd);
      break;

    case CMD_BRAKE:
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 1)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      if (p[0] > 1)         { sendNack(cmd, ERR_INVALID_PARAM); return; }
      motorBrake(p[0]);
      sendAck(cmd);
      break;

    case CMD_GET_RPM: {
      if (sysState < IDLE) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 1)         { sendNack(cmd, ERR_INVALID_PARAM); return; }
      uint8_t mid = p[0];
      if (mid == 0)      { sendRpm(0, rpm1); }
      else if (mid == 1) { sendRpm(1, rpm2); }
      else if (mid == 2) { sendRpm(0, rpm1); sendRpm(1, rpm2); }
      else               { sendNack(cmd, ERR_INVALID_PARAM); }
      break;
    }

    case CMD_GET_STATUS:
      sendStatus();
      break;

    case 0x30: {  // DEBUG: 读取编码器原始计数
      noInterrupts();
      long c1 = encoderCount, c2 = encoderCount2;
      interrupts();
      uint8_t buf[8];
      buf[0] = (uint8_t)(c1 >> 24); buf[1] = (uint8_t)(c1 >> 16);
      buf[2] = (uint8_t)(c1 >> 8);  buf[3] = (uint8_t)(c1 & 0xFF);
      buf[4] = (uint8_t)(c2 >> 24); buf[5] = (uint8_t)(c2 >> 16);
      buf[6] = (uint8_t)(c2 >> 8);  buf[7] = (uint8_t)(c2 & 0xFF);
      sendFrame(0x30, buf, 8);
      break;
    }

    case CMD_RESET:
      motorCoast(0); motorCoast(1);
      sysState = UNINIT;
      sendAck(cmd);
      break;

    default:
      sendNack(cmd, ERR_UNKNOWN_CMD);
      break;
  }
}

// ============================================================
//  发送帧
// ============================================================
void sendFrame(uint8_t cmd, uint8_t *payload, uint8_t len) {
  uint8_t chk = cmd ^ len;
  for (uint8_t i = 0; i < len; i++) chk ^= payload[i];
  Serial.write(FRAME_H1);
  Serial.write(FRAME_H2);
  Serial.write(cmd);
  Serial.write(len);
  if (len > 0) Serial.write(payload, len);
  Serial.write(chk);
}

void sendAck(uint8_t ackedCmd) {
  sendFrame(RSP_ACK, &ackedCmd, 1);
}

void sendNack(uint8_t ackedCmd, uint8_t err) {
  uint8_t p[2] = { ackedCmd, err };
  sendFrame(RSP_NACK, p, 2);
}

void sendRpm(uint8_t mid, int16_t rpm) {
  uint8_t p[3] = { mid, (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF) };
  sendFrame(RSP_RPM_DATA, p, 3);
}

void sendStatus() {
  uint8_t p[5] = {
    (uint8_t)sysState,
    (uint8_t)(rpm1 >> 8), (uint8_t)(rpm1 & 0xFF),
    (uint8_t)(rpm2 >> 8), (uint8_t)(rpm2 & 0xFF)
  };
  sendFrame(RSP_STATUS, p, 5);
}

// ============================================================
//  电机驱动
// ============================================================
void setMotorSpeed(uint8_t mid, int16_t speed) {
  int s = abs((int)speed);
  if (mid == 0) { speed >= 0 ? motorForward(s)  : motorReverse(s);  }
  else          { speed >= 0 ? motorForward2(s) : motorReverse2(s); }
}

void motorCoast(uint8_t mid) {
  if (mid == 0) { analogWrite(IN1_PIN,   0); analogWrite(IN2_PIN,   0); }
  else          { analogWrite(IN1_PIN_2, 0); analogWrite(IN2_PIN_2, 0); }
}

void motorBrake(uint8_t mid) {
  if (mid == 0) { analogWrite(IN1_PIN,   255); analogWrite(IN2_PIN,   255); }
  else          { analogWrite(IN1_PIN_2, 255); analogWrite(IN2_PIN_2, 255); }
}

void motorForward(int s)  { analogWrite(IN1_PIN,   s); analogWrite(IN2_PIN,   0); }
void motorReverse(int s)  { analogWrite(IN1_PIN,   0); analogWrite(IN2_PIN,   s); }
void motorForward2(int s) { analogWrite(IN1_PIN_2, s); analogWrite(IN2_PIN_2, 0); }
void motorReverse2(int s) { analogWrite(IN1_PIN_2, 0); analogWrite(IN2_PIN_2, s); }

