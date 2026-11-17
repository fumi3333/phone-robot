// 2026-11-16 as-built v2 版
// - A=前ステアリング(GPIO16/17)、B=後ろ駆動(GPIO18/19)
// - シリアル受信を行バッファ方式にして途中切れ・取りこぼしを解消
// - ステアリングは LEDC PWM ではなく digitalWrite(HIGH/LOW) の完全直流 100% 出力に変更

/**
 * ジョーゼン 1/16 トヨタ86 (JRVC015-RD) 自動運転ファームウェア v2.2 (as-built)
 */

#include <Arduino.h>

// --- ピン配置 (ESP32-WROOM-32 安全ピン) ---
const int PIN_DRIVE_IN1 = 18; // 走行 前進 (ドライバ IN3 に接続)
const int PIN_DRIVE_IN2 = 19; // 走行 後退 (ドライバ IN4 に接続)
const int PIN_STEER_IN3 = 16; // 操舵 左折 (ドライバ IN1 に接続)
const int PIN_STEER_IN4 = 17; // 操舵 右折 (ドライバ IN2 に接続)

// --- PWM 設定 (走行用 LEDC) ---
const int PWM_FREQ = 1000;    // 1kHz
const int PWM_RES  = 8;       // 8bit (0 - 255)
const int CH_DRIVE1 = 0;
const int CH_DRIVE2 = 1;

// 室内走行用 安全スピード上限 (最大255フルパワー)
const int MAX_DRIVE_PWM = 255;

// 通信途絶監視タイマー
unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT_MS = 400;

// 車体電池電圧テレメトリ (OpenBot 互換: v,<mV>\n)
// 分圧抵抗を接続したら PIN_BATTERY_ADC を 34 等に変更して有効化する (未接続時は -1)
const int PIN_BATTERY_ADC = -1;
const float BATTERY_DIVIDER_RATIO = 4.0f; // 例: 30kΩ + 10kΩ 分圧なら 4.0
unsigned long lastBatteryTx = 0;

// シリアル行受信バッファ
char rxBuf[48];
uint8_t rxLen = 0;

// LEDC 出力ヘルパー: core 3.x の ledcWrite はピン番号、2.x はチャンネル番号を取る
void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void emergencyStop() {
  pwmWrite(PIN_DRIVE_IN1, CH_DRIVE1, 0);
  pwmWrite(PIN_DRIVE_IN2, CH_DRIVE2, 0);
  digitalWrite(PIN_STEER_IN3, LOW);
  digitalWrite(PIN_STEER_IN4, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(15);

  // 走行モーター: LEDC PWM
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_DRIVE_IN1, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_DRIVE_IN2, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(CH_DRIVE1, PWM_FREQ, PWM_RES);
    ledcSetup(CH_DRIVE2, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_DRIVE_IN1, CH_DRIVE1);
    ledcAttachPin(PIN_DRIVE_IN2, CH_DRIVE2);
  #endif

  // 操舵モーター: 完全直流 100% 出力 (digitalWrite)
  pinMode(PIN_STEER_IN3, OUTPUT);
  pinMode(PIN_STEER_IN4, OUTPUT);

  emergencyStop();
  Serial.println("TOYOTA86_ESP32_ASBUILT_V2_ONLINE");
}

// 走行モーター制御 (speed: -255 〜 +255)
void setDrive(int speed) {
  speed = constrain(speed, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  if (speed > 0) {
    pwmWrite(PIN_DRIVE_IN1, CH_DRIVE1, speed);
    pwmWrite(PIN_DRIVE_IN2, CH_DRIVE2, 0);
  } else if (speed < 0) {
    pwmWrite(PIN_DRIVE_IN1, CH_DRIVE1, 0);
    pwmWrite(PIN_DRIVE_IN2, CH_DRIVE2, -speed);
  } else {
    pwmWrite(PIN_DRIVE_IN1, CH_DRIVE1, 0);
    pwmWrite(PIN_DRIVE_IN2, CH_DRIVE2, 0);
  }
}

// 操舵モーター制御 (steer: 負=左, 正=右, 0=中立)
// トイラジコンの振り切りステアリングは純正プロポ同様、完全直流100% (HIGH/LOW) でバネを押し切る
void setSteer(int steer) {
  if (steer < 0) {
    digitalWrite(PIN_STEER_IN3, HIGH); // 左 100% 直流フルパワー
    digitalWrite(PIN_STEER_IN4, LOW);
  } else if (steer > 0) {
    digitalWrite(PIN_STEER_IN3, LOW);
    digitalWrite(PIN_STEER_IN4, HIGH); // 右 100% 直流フルパワー
  } else {
    digitalWrite(PIN_STEER_IN3, LOW);
    digitalWrite(PIN_STEER_IN4, LOW);
  }
}

void handleLine(const char* line) {
  lastCmdTime = millis();
  if (line[0] == 'c' && line[1] == ',') {
    int drive = 0;
    int steer = 0;
    if (sscanf(line + 2, "%d,%d", &drive, &steer) == 2) {
      setDrive(drive);
      setSteer(steer);
    }
    return;
  }
  switch (line[0]) {
    case 'f': setDrive(150); setSteer(0); break;
    case 'b': setDrive(-140); setSteer(0); break;
    case 'l': setDrive(140); setSteer(-230); break;
    case 'r': setDrive(140); setSteer(230); break;
    case 's': emergencyStop(); break;
  }
}

void loop() {
  // 1. 改行まで1行溜めてから確実にパースする
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        handleLine(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      // バッファ溢れ時は破棄してリセット
      rxLen = 0;
    }
  }

  // 2. フェイルセーフ監視 (通信途絶時は即停止)
  if (millis() - lastCmdTime > TIMEOUT_MS) {
    emergencyStop();
  }

  // 3. 車体電池電圧テレメトリ (分圧抵抗接続時のみ 500ms 周期で送信)
  if (PIN_BATTERY_ADC >= 0 && millis() - lastBatteryTx >= 500) {
    lastBatteryTx = millis();
    int mv = (int)(analogReadMilliVolts(PIN_BATTERY_ADC) * BATTERY_DIVIDER_RATIO);
    Serial.print("v,");
    Serial.println(mv);
  }

  delay(5);
}
