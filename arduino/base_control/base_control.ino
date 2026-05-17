// ============================================================
//  ESP32-C3 SuperMini + DRV8833 x2 + 霍尔编码器TT 马达
//  UART 控制协议 V1.0  |  波特率115200
// ============================================================

// --- 引脚定义 (AKA-00) ---
#define LED_PIN    8
#define IN1_PIN    2
#define IN2_PIN    1
#define ENC_A      7   // 电机0 编码器（对应 IN1/IN2 驱动的电机）
#define ENC_B      10
#define IN1_PIN_2  3
#define IN2_PIN_2  4
#define ENC_A_2    5   // 电机1 编码器（对应 IN1_2/IN2_2 驱动的电机）
#define ENC_B_2    6

// --- PWM 默认参数 ---
#define PWM_BITS          8
#define PWM_FREQ_DEFAULT  20000
#define PPR_DEFAULT       4680

// --- 前向声明（Arduino 自动原型生成需要） ---
struct PIDController;

// --- 协议：命令字 ---
#define CMD_INIT        0x01
#define CMD_CONFIG      0x02
#define CMD_SET_SPEED   0x10
#define CMD_STOP        0x11
#define CMD_BRAKE       0x12
#define CMD_GET_RPM     0x20
#define CMD_GET_STATUS  0x21
#define CMD_SET_SPEEDS  0x13  // 新增：同时设置双电机速度
#define CMD_SET_PID     0x14  // 设置PID参数
#define CMD_GET_PID     0x15  // 读取PID参数
#define CMD_AUTO_TUNE   0x16  // 自动整定PID
#define CMD_RESET       0xFF

// --- 协议：响应字 ---
#define RSP_ACK         0x80
#define RSP_NACK        0x81
#define RSP_RPM_DATA    0x90
#define RSP_STATUS      0x91
#define RSP_PID_DATA    0x92  // PID参数响应

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
enum SysState : uint8_t { UNINIT = 0, IDLE = 1, READY = 2, RUNNING = 3, SYS_ERROR = 4, AUTO_TUNE = 5 };
SysState sysState = UNINIT;

// --- 自动整定状态（继电器反馈法） ---
enum AutoTuneState : uint8_t {
  AT_IDLE = 0, AT_RAMP = 1, AT_HIGH = 2, AT_LOW = 3, AT_DONE = 4, AT_TIMEOUT = 5
};
AutoTuneState atState = AT_IDLE;
uint8_t  atMotorId;          // 当前整定的电机 ID
uint16_t atTestPwm = 200;    // 测试 PWM 值（非全速，留裕量）
float    atTargetRpm;        // 目标 RPM
float    atHysteresis;       // 滞环 = target * 0.15
int      atCycleCount;       // 已完成的振荡周期数
float    atPeakRpm;          // HIGH 阶段的峰值
float    atValleyRpm;        // LOW 阶段的谷值
float    atAmplitudeSum;     // 振幅累加
int      atAmpCount;         // 振幅采样计数
unsigned long atCrossTime;   // 上次穿越 target 的时间（测 Tu）
float    atTuSum;            // 振荡周期累加
unsigned long atStartTime;   // 总计时

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
//  PID 控制器（参考 DB20_3 累积式 PID）
// ============================================================

// --- PWM / PID 常数 ---
#define PWM_RPM_MAX 150.0f
#define PID_KP 0.8f   // P-only 比例增益（前馈承担主力，P 只做加速/减速修正）
#define PID_KI 0.10f  // 积分项：低速时填补前馈误差，积分上限×Ki=30PWM
#define PID_KD 0.01f  // 微分（未使用）
#define PID_INTEGRAL_MAX 300
#define PID_OUTPUT_MAX 255  // 最大输出
#define MIN_USEFUL_RPM  10.0f  // 电机能稳定运转的最低 RPM
#define FRICTION_BOOST  10     // 突破静摩擦的额外 PWM
#define FF_GAIN         2.0f   // 前馈增益: 1.0=线性估算, 实测偏低故×2
#define RPM_DEADZONE 3.0f

struct PIDController {
  float target_rpm;
  float Kp, Ki, Kd;
  float integral;
  float prev_error;
  float output_f;  // 浮点累积输出
  int output;      // 整型输出，供 PWM 使用
};

PIDController pid1 = {0, PID_KP, PID_KI, PID_KD, 0, 0, 0, 0};
PIDController pid2 = {0, PID_KP, PID_KI, PID_KD, 0, 0, 0, 0};

// PWM 转 RPM（线性映射，参考 DB20_1）
int pwmToRpm(int pwm) {
  if (pwm <= 0) return 0;
  return (int)(pwm * PWM_RPM_MAX / 255.0f);
}

// RPM 转 PWM（线性映射，参考 DB20_1）
int rpmToPwm(float rpm) {
  if (rpm <= 0) return 0;
  return (int)(rpm * 255.0f / PWM_RPM_MAX);
}

// 前馈 + P + 小 I（I 只填补前馈估算误差，不主导）
void computePID(PIDController* pid, float target_rpm, float current_rpm) {
  float abs_target = fabs(target_rpm);
  float abs_current = fabs(current_rpm);
  float sign = (target_rpm > 0) ? 1.0f : ((target_rpm < 0) ? -1.0f : 0.0f);
  float bias = target_rpm - current_rpm;

  // 前馈
  float ff = sign * rpmToPwm(abs_target) * FF_GAIN;

  // 静摩擦突破：仅电机不转时激活
  float friction_boost = 0;
  if (abs_current < 2.0f && abs_target > RPM_DEADZONE) {
    float pwm_need = rpmToPwm(MIN_USEFUL_RPM) + FRICTION_BOOST;
    if (rpmToPwm(abs_target) < pwm_need) {
      friction_boost = sign * (pwm_need - rpmToPwm(abs_target));
    }
  }

  // 积分：输出未饱和时才累加，上限 200
  bool sat_high = (pid->output >= PID_OUTPUT_MAX && bias > 0);
  bool sat_low  = (pid->output <= -PID_OUTPUT_MAX && bias < 0);
  if (!sat_high && !sat_low) {
    pid->integral += bias;
  }
  if (pid->integral > PID_INTEGRAL_MAX) pid->integral = PID_INTEGRAL_MAX;
  if (pid->integral < -PID_INTEGRAL_MAX) pid->integral = -PID_INTEGRAL_MAX;

  // P + I 修正
  float p_term = PID_KP * bias;
  float i_term = PID_KI * pid->integral;

  float total = ff + friction_boost + p_term + i_term;
  pid->output = (int)total;

  if (pid->output > PID_OUTPUT_MAX) pid->output = PID_OUTPUT_MAX;
  if (pid->output < -PID_OUTPUT_MAX) pid->output = -PID_OUTPUT_MAX;
}

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

  // 50ms 更新 RPM（更长采样 = 更少噪声）
  if (now - lastRpmTime >= 50) {
    noInterrupts();
    long s1 = encoderCount, s2 = encoderCount2;
    interrupts();
    float dt = (now - lastRpmTime) / 1000.0f;
    int16_t rpm1_raw = (int16_t)(((s1 - lastCnt1) / (float)cfg_ppr) / dt * 60.0f);
    int16_t rpm2_raw = (int16_t)(((s2 - lastCnt2) / (float)cfg_ppr) / dt * 60.0f);
    // M0 编码器方向与 PWM 正向相反，M1 方向一致
    rpm1 = -rpm1_raw;
    rpm2 = rpm2_raw;
    if (rpm1 > -5 && rpm1 < 5) rpm1 = 0;
    if (rpm2 > -5 && rpm2 < 5) rpm2 = 0;
    lastCnt1 = s1; lastCnt2 = s2;
    lastRpmTime = now;
    if (sysState >= READY) {
      runMotorControl(dt);
    }
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
      pid1.integral = 0; pid1.output = 0; pid1.output_f = 0; pid1.prev_error = 0;
      pid2.integral = 0; pid2.output = 0; pid2.output_f = 0; pid2.prev_error = 0;
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
      spd = (int16_t)constrain((int)spd, -100, 100);
      if (mid > 1) { sendNack(cmd, ERR_INVALID_PARAM); return; }
      setMotorSpeed(mid, spd);
      sysState = RUNNING;
      sendAck(cmd);
      break;
    }

    case CMD_SET_SPEEDS: {
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 4)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      int16_t spd1 = (int16_t)((p[0] << 8) | p[1]);
      int16_t spd2 = (int16_t)((p[2] << 8) | p[3]);
      spd1 = (int16_t)constrain((int)spd1, -100, 100);
      spd2 = (int16_t)constrain((int)spd2, -100, 100);
      setMotorSpeed(0, spd1);
      setMotorSpeed(1, spd2);
      sysState = RUNNING;
      sendAck(cmd);
      break;
    }

    case CMD_STOP:
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 1)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      if (p[0] > 2)         { sendNack(cmd, ERR_INVALID_PARAM); return; }
      if (p[0] == 2)        { motorCoast(0); motorCoast(1); }
      else                  { motorCoast(p[0]); }
      sendAck(cmd);
      break;

    case CMD_BRAKE:
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 1)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      if (p[0] > 2)         { sendNack(cmd, ERR_INVALID_PARAM); return; }
      if (p[0] == 2)        { motorBrake(0); motorBrake(1); }
      else                  { motorBrake(p[0]); }
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

    case CMD_SET_PID: {
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 7)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      uint8_t mid = p[0];
      if (mid > 1)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      int16_t kp = (int16_t)((p[1] << 8) | p[2]);
      int16_t ki = (int16_t)((p[3] << 8) | p[4]);
      int16_t kd = (int16_t)((p[5] << 8) | p[6]);
      PIDController* pid = (mid == 0) ? &pid1 : &pid2;
      pid->Kp = kp / 100.0f;
      pid->Ki = ki / 100.0f;
      pid->Kd = kd / 100.0f;
      sendAck(cmd);
      break;
    }

    case CMD_GET_PID: {
      if (sysState < IDLE) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 1)         { sendNack(cmd, ERR_INVALID_PARAM); return; }
      uint8_t mid = p[0];
      if (mid > 1)         { sendNack(cmd, ERR_INVALID_PARAM); return; }
      PIDController* pid = (mid == 0) ? &pid1 : &pid2;
      uint8_t buf[6];
      buf[0] = (uint8_t)((int)(pid->Kp * 100) >> 8);
      buf[1] = (uint8_t)((int)(pid->Kp * 100) & 0xFF);
      buf[2] = (uint8_t)((int)(pid->Ki * 100) >> 8);
      buf[3] = (uint8_t)((int)(pid->Ki * 100) & 0xFF);
      buf[4] = (uint8_t)((int)(pid->Kd * 100) >> 8);
      buf[5] = (uint8_t)((int)(pid->Kd * 100) & 0xFF);
      sendFrame(RSP_PID_DATA, buf, 6);
      break;
    }

    case CMD_AUTO_TUNE: {
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 3)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      uint8_t mid = p[0];
      if (mid > 1)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      atMotorId   = mid;
      atTargetRpm = (float)((p[1] << 8) | p[2]);
      if (atTargetRpm <= 0) { sendNack(cmd, ERR_INVALID_PARAM); return; }
      // 测试 PWM：目标 RPM 对应的 PWM * 1.2 倍, 至少 80
      atTestPwm = (uint16_t)(atTargetRpm * 255.0f / PWM_RPM_MAX * 1.2f);
      if (atTestPwm < 80) atTestPwm = 80;
      if (atTestPwm > 220) atTestPwm = 220;
      atHysteresis = atTargetRpm * 0.15f;
      if (atHysteresis < 3.0f) atHysteresis = 3.0f;  // 最小滞环 3 RPM
      // 整定前先停两电机
      motorCoast(0); motorCoast(1);
      // 初始化对应电机的 PID
      if (mid == 0) { pid1.target_rpm = atTargetRpm; }
      else          { pid2.target_rpm = atTargetRpm; }
      atState = AT_RAMP;
      atStartTime = millis();
      sysState = AUTO_TUNE;
      sendAck(cmd);
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

    case 0x31: {  // RAW PWM 测试：直接驱动，绕开 PID
      if (sysState < READY) { sendNack(cmd, ERR_WRONG_STATE); return; }
      if (len < 3)          { sendNack(cmd, ERR_INVALID_PARAM); return; }
      uint8_t mid = p[0];
      uint16_t pwm = (p[1] << 8) | p[2];
      if (mid > 1 || pwm > 255) { sendNack(cmd, ERR_INVALID_PARAM); return; }
      // 先停两电机
      analogWrite(IN1_PIN, 0); analogWrite(IN2_PIN, 0);
      analogWrite(IN1_PIN_2, 0); analogWrite(IN2_PIN_2, 0);
      delay(100);
      // 清编码器
      noInterrupts();
      encoderCount = 0; encoderCount2 = 0;
      interrupts();
      lastCnt1 = 0; lastCnt2 = 0;
      lastRpmTime = millis();
      // 施加 PWM
      if (mid == 0) motorForward(pwm);
      else          motorForward2(pwm);
      delay(500);  // 等电机稳定
      // 读 RPM（不取反，原始方向）
      noInterrupts();
      long s1 = encoderCount, s2 = encoderCount2;
      interrupts();
      // 停电机
      analogWrite(IN1_PIN, 0); analogWrite(IN2_PIN, 0);
      analogWrite(IN1_PIN_2, 0); analogWrite(IN2_PIN_2, 0);
      // 返回原始编码器计数（未取反）
      uint8_t buf[12];
      buf[0] = mid; buf[1] = (uint8_t)(pwm);
      buf[2] = (uint8_t)(s1 >> 24); buf[3] = (uint8_t)(s1 >> 16);
      buf[4] = (uint8_t)(s1 >> 8);  buf[5] = (uint8_t)(s1 & 0xFF);
      buf[6] = (uint8_t)(s2 >> 24); buf[7] = (uint8_t)(s2 >> 16);
      buf[8] = (uint8_t)(s2 >> 8);  buf[9] = (uint8_t)(s2 & 0xFF);
      // 计算 RPM（未取反）
      int16_t raw_rpm1 = (int16_t)((s1 / (float)cfg_ppr) / 0.5f * 60.0f);
      int16_t raw_rpm2 = (int16_t)((s2 / (float)cfg_ppr) / 0.5f * 60.0f);
      buf[10] = (uint8_t)(raw_rpm1 >> 8); buf[11] = (uint8_t)(raw_rpm1 & 0xFF);
      sendFrame(0x31, buf, 12);
      // 恢复 PID 状态
      lastRpmTime = millis();
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
  if (mid > 1) return;
  // speed: -100~100 → target_rpm: -150~150
  float target_rpm = speed * PWM_RPM_MAX / 100.0f;
  if (mid == 0) { pid1.target_rpm = target_rpm; }
  else          { pid2.target_rpm = target_rpm; }
}

void runMotorControl(float dt) {
  if (sysState == AUTO_TUNE) { runAutoTune(dt); return; }
  // Motor 1
  if (abs(pid1.target_rpm) < RPM_DEADZONE) {
    pid1.integral = 0;
    pid1.output = 0;
    analogWrite(IN1_PIN, 0); analogWrite(IN2_PIN, 0);
  } else {
    computePID(&pid1, pid1.target_rpm, (float)rpm1);
    int pwm = abs(pid1.output);
    if (pid1.output > 0) motorForward(pwm);
    else                 motorReverse(pwm);
  }
  // Motor 2
  if (abs(pid2.target_rpm) < RPM_DEADZONE) {
    pid2.integral = 0;
    pid2.output = 0;
    analogWrite(IN1_PIN_2, 0); analogWrite(IN2_PIN_2, 0);
  } else {
    computePID(&pid2, pid2.target_rpm, (float)rpm2);
    int pwm = abs(pid2.output);
    if (pid2.output > 0) motorForward2(pwm);
    else                 motorReverse2(pwm);
  }
}

void runAutoTune(float dt) {
  float rpm = (atMotorId == 0) ? (float)rpm1 : (float)rpm2;
  unsigned long now = millis();
  uint8_t in1, in2;

  if (atMotorId == 0) { in1 = IN1_PIN; in2 = IN2_PIN; }
  else                { in1 = IN1_PIN_2; in2 = IN2_PIN_2; }

  // 超时保护：20 秒
  if (now - atStartTime > 20000) {
    atState = AT_TIMEOUT;
  }

  switch (atState) {
    case AT_RAMP:
      // 施加测试 PWM，等待 RPM 达到目标
      analogWrite(in1, atTestPwm); analogWrite(in2, 0);
      if (rpm >= atTargetRpm) {
        atPeakRpm = rpm;
        atCycleCount = 0;
        atAmplitudeSum = 0; atAmpCount = 0; atTuSum = 0;
        atCrossTime = now;
        atState = AT_HIGH;
      } else if (now - atStartTime > 5000) {
        // 5 秒内未达到目标转速，可能 PWM 不足，降级处理
        atState = AT_TIMEOUT;
      }
      break;

    case AT_HIGH:
      // 滑行减速，等待 RPM 降到 target - hyst
      analogWrite(in1, 0); analogWrite(in2, 0);  // coast
      if (rpm > atPeakRpm) atPeakRpm = rpm;
      if (rpm <= atTargetRpm - atHysteresis) {
        atValleyRpm = rpm;
        atAmplitudeSum += (atPeakRpm - atValleyRpm);
        atAmpCount++;
        float halfTu = (now - atCrossTime) / 1000.0f;
        atTuSum += halfTu * 2.0f;
        atCrossTime = now;
        atState = AT_LOW;
      }
      break;

    case AT_LOW:
      // 重新加速，等待 RPM 升到 target + hyst
      analogWrite(in1, atTestPwm); analogWrite(in2, 0);
      if (rpm < atValleyRpm) atValleyRpm = rpm;
      if (rpm >= atTargetRpm + atHysteresis) {
        atPeakRpm = rpm;
        atCycleCount++;
        float halfTu = (now - atCrossTime) / 1000.0f;
        atTuSum += halfTu * 2.0f;
        atCrossTime = now;
        if (atCycleCount >= 3) {  // 3 个周期即可
          atState = AT_DONE;
        } else {
          atState = AT_HIGH;
        }
      }
      break;

    case AT_DONE: {
      analogWrite(in1, 0); analogWrite(in2, 0);

      float avgAmplitude = (atAmpCount > 0) ? atAmplitudeSum / atAmpCount : atTargetRpm;
      float avgTu = (atCycleCount > 0) ? atTuSum / atCycleCount : 0.5f;

      // 继电器法临界增益
      float Kc = 4.0f * atTestPwm / (3.14159f * avgAmplitude);
      if (Kc < 0.05f) Kc = 0.05f;
      if (Kc > 50.0f) Kc = 50.0f;

      // 保守 Ziegler-Nichols
      float kp = 0.45f * Kc;
      float ki = (avgTu > 0.01f) ? (2.0f * kp) / avgTu : 0.0f;
      float kd = (avgTu > 0.01f) ? kp * avgTu / 8.0f : 0.0f;

      PIDController* pid = (atMotorId == 0) ? &pid1 : &pid2;
      pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
      pid->integral = 0; pid->prev_error = 0; pid->output = 0; pid->output_f = 0;
      pid->target_rpm = 0;

      // 发送结果 (7 bytes: mid + kp + ki + kd)
      uint8_t buf[7];
      buf[0] = atMotorId;
      int16_t kp_i = (int16_t)(kp * 100); buf[1] = (uint8_t)(kp_i >> 8); buf[2] = (uint8_t)(kp_i & 0xFF);
      int16_t ki_i = (int16_t)(ki * 100); buf[3] = (uint8_t)(ki_i >> 8); buf[4] = (uint8_t)(ki_i & 0xFF);
      int16_t kd_i = (int16_t)(kd * 100); buf[5] = (uint8_t)(kd_i >> 8); buf[6] = (uint8_t)(kd_i & 0xFF);
      sendFrame(0x93, buf, 7);
      sysState = READY;
      atState = AT_IDLE;
      break;
    }

    case AT_TIMEOUT:
    default:
      analogWrite(in1, 0); analogWrite(in2, 0);
      // 超时也返回一组安全的默认参数
      uint8_t buf[7];
      buf[0] = atMotorId;
      int16_t kp_i = (int16_t)(PID_KP * 100); buf[1] = (uint8_t)(kp_i >> 8); buf[2] = (uint8_t)(kp_i & 0xFF);
      int16_t ki_i = (int16_t)(PID_KI * 100); buf[3] = (uint8_t)(ki_i >> 8); buf[4] = (uint8_t)(ki_i & 0xFF);
      int16_t kd_i = (int16_t)(PID_KD * 100); buf[5] = (uint8_t)(kd_i >> 8); buf[6] = (uint8_t)(kd_i & 0xFF);
      sendFrame(0x93, buf, 7);
      sysState = READY;
      atState = AT_IDLE;
      break;
  }
}

void motorCoast(uint8_t mid) {
  if (mid == 0) {
    analogWrite(IN1_PIN, 0); analogWrite(IN2_PIN, 0);
    pid1.target_rpm = 0; pid1.integral = 0; pid1.output = 0; pid1.output_f = 0; pid1.prev_error = 0;
  } else {
    analogWrite(IN1_PIN_2, 0); analogWrite(IN2_PIN_2, 0);
    pid2.target_rpm = 0; pid2.integral = 0; pid2.output = 0; pid2.output_f = 0; pid2.prev_error = 0;
  }
}

void motorBrake(uint8_t mid) {
  if (mid == 0) {
    analogWrite(IN1_PIN, 255); analogWrite(IN2_PIN, 255);
    pid1.target_rpm = 0; pid1.integral = 0; pid1.output = 0; pid1.output_f = 0; pid1.prev_error = 0;
  } else {
    analogWrite(IN1_PIN_2, 255); analogWrite(IN2_PIN_2, 255);
    pid2.target_rpm = 0; pid2.integral = 0; pid2.output = 0; pid2.output_f = 0; pid2.prev_error = 0;
  }
}

void motorForward(int s)  { analogWrite(IN1_PIN,   s); analogWrite(IN2_PIN,   0); }
void motorReverse(int s)  { analogWrite(IN1_PIN,   0); analogWrite(IN2_PIN,   s); }
void motorForward2(int s) { analogWrite(IN1_PIN_2, 0); analogWrite(IN2_PIN_2, s); }
void motorReverse2(int s) { analogWrite(IN1_PIN_2, s); analogWrite(IN2_PIN_2, 0); }

