// ============================================================
//  ESP32-C3 SuperMini + DRV8833 x2 + 霍尔编码器TT 马达
//  UART 控制协议 V1.0  |  波特率115200
// ============================================================

// --- 编译配置（切换配置时修改这里） ---
// #define CONFIG_ID  0   // 配置0：调试
#define CONFIG_ID  1   // 配置1：生产

// --- 引脚定义 ---
#define LED_PIN    8
#ifdef CONFIG_ID
#if CONFIG_ID == 0
// 配置0：调试引脚
#define IN1_PIN    4
#define IN2_PIN    3
#define ENC_A      5
#define ENC_B      6
#define IN1_PIN_2  1
#define IN2_PIN_2  2
#define ENC_A_2    7
#define ENC_B_2   10
#elif CONFIG_ID == 1
// 配置1：生产引脚（默认）
#define IN1_PIN    4
#define IN2_PIN    3
#define ENC_A      5
#define ENC_B      6
#define IN1_PIN_2  1
#define IN2_PIN_2  2
#define ENC_A_2    7
#define ENC_B_2   10
#endif
#endif

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

// --- 错误---
#define ERR_WRONG_STATE   0x01
#define ERR_BAD_CHECKSUM  0x02
#define ERR_INVALID_PARAM 0x03
#define ERR_UNKNOWN_CMD   0x04

// --- 双串口 ---
// Serial  = USB CDC (GPIO18/19 USB口)
// SerialUART0 = 硬件 UART0 (GPIO20 RX / GPIO21 TX)
HardwareSerial SerialUART0(0);

// --- 帧头 ---
#define FRAME_H1  0xAA
#define FRAME_H2  0x55

// ============================================================
//  系统状态// ============================================================
enum SysState : uint8_t { UNINIT = 0, IDLE = 1, READY = 2, RUNNING = 3, SYS_ERROR = 4 };
SysState sysState = UNINIT;

// --- 配置（可配置CONFIG 命令修改--
uint16_t cfg_ppr      = PPR_DEFAULT;
uint16_t cfg_pwm_freq = PWM_FREQ_DEFAULT;

// ============================================================
//  编码器（中断，需 IRAM// ============================================================
volatile long encoderCount  = 0;
volatile long encoderCount2 = 0;
volatile unsigned long isrCalls1 = 0, isrCalls2 = 0;

void IRAM_ATTR encA_ISR()  { encoderCount  += (digitalRead(ENC_A)   != digitalRead(ENC_B))   ? 1 : -1; isrCalls1++; }
void IRAM_ATTR encB_ISR()  { encoderCount  += (digitalRead(ENC_A)   == digitalRead(ENC_B))   ? 1 : -1; isrCalls1++; }
void IRAM_ATTR encA2_ISR() { encoderCount2 += (digitalRead(ENC_A_2) != digitalRead(ENC_B_2)) ? 1 : -1; isrCalls2++; }
void IRAM_ATTR encB2_ISR() { encoderCount2 += (digitalRead(ENC_A_2) == digitalRead(ENC_B_2)) ? 1 : -1; isrCalls2++; }

// --- RPM 计算 ---
unsigned long lastRpmTime = 0;
long lastCnt1 = 0, lastCnt2 = 0;
int16_t rpm1 = 0, rpm2 = 0;

// ============================================================
//  LED 状态// ============================================================
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
//  初始化辅助// ============================================================
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
  pinMode(ENC_A,   INPUT_PULLUP); pinMode(ENC_B,   INPUT_PULLUP);
  pinMode(ENC_A_2, INPUT_PULLUP); pinMode(ENC_B_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A),   encA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B),   encB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_2), encA2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_2), encB2_ISR, CHANGE);
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);                          // USB CDC
  SerialUART0.begin(115200, SERIAL_8N1, 20, 21); // 硬件 UART0
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // 上电默认灭（active low）
  initPWM();
  initEncoders();
  lastRpmTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 接收串口数据（双路）
  while (Serial.available()) {
    processByte((uint8_t)Serial.read());
  }
  while (SerialUART0.available()) {
    processByte((uint8_t)SerialUART0.read());
  }

  // 20ms 更新 RPM
  if (now - lastRpmTime >= 20) {
    noInterrupts();
    long s1 = encoderCount, s2 = encoderCount2;
    interrupts();
    float dt = (now - lastRpmTime) / 1000.0f;
    rpm1 = (int16_t)(((s1 - lastCnt1) / (float)cfg_ppr) / dt * 60.0f);
    rpm2 = (int16_t)(((s2 - lastCnt2) / (float)cfg_ppr) / dt * 60.0f);
    if (rpm1 > -5 && rpm1 < 5) rpm1 = 0;
    if (rpm2 > -5 && rpm2 < 5) rpm2 = 0;
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
      // 双闪：亮100 00 00 00
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

    case 0x30: {  // DEBUG: 编码器计数 + ISR 触发次数
      noInterrupts();
      long c1 = encoderCount, c2 = encoderCount2;
      unsigned long i1 = isrCalls1, i2 = isrCalls2;
      interrupts();
      uint8_t buf[16];
      buf[0]  = (uint8_t)(c1 >> 24); buf[1]  = (uint8_t)(c1 >> 16);
      buf[2]  = (uint8_t)(c1 >> 8);  buf[3]  = (uint8_t)(c1 & 0xFF);
      buf[4]  = (uint8_t)(c2 >> 24); buf[5]  = (uint8_t)(c2 >> 16);
      buf[6]  = (uint8_t)(c2 >> 8);  buf[7]  = (uint8_t)(c2 & 0xFF);
      buf[8]  = (uint8_t)(i1 >> 24); buf[9]  = (uint8_t)(i1 >> 16);
      buf[10] = (uint8_t)(i1 >> 8);  buf[11] = (uint8_t)(i1 & 0xFF);
      buf[12] = (uint8_t)(i2 >> 24); buf[13] = (uint8_t)(i2 >> 16);
      buf[14] = (uint8_t)(i2 >> 8);  buf[15] = (uint8_t)(i2 & 0xFF);
      sendFrame(0x30, buf, 16);
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

  uint8_t buf[64];
  uint8_t idx = 0;
  buf[idx++] = FRAME_H1;
  buf[idx++] = FRAME_H2;
  buf[idx++] = cmd;
  buf[idx++] = len;
  for (uint8_t i = 0; i < len; i++) buf[idx++] = payload[i];
  buf[idx++] = chk;

  Serial.write(buf, idx);       // USB CDC
  SerialUART0.write(buf, idx);  // 硬件 UART0
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

