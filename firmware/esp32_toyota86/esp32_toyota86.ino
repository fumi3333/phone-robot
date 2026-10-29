/**
 * ジョーゼン 1/16 トヨタ86 (JRVC015-RD) 自動運転ファームウェア v2.1 (ESP32完全対応版)
 *
 * 【重要修正】
 * 1. ESP32で安全に使える GPIO 16, 17, 18, 19 にピンを再配置 (GPIO 6-11のSPIフラッシュ衝突を回避)
 * 2. ESP32のハードウェアPWM (LEDC) を使用
 * 3. 電源分離対応: ESP32はスマホのUSB-Cから給電、モーターはラジコンの単3×6本から給電 (GND共通)
 * 4. 速度リミッター: 室内安全のため最大速度を PWM 160 に制限
 * 5. 通信途絶時の自動緊急ブレーキ (400ms Watchdog)
 */

#include <Arduino.h>

// --- ピン配置 (ESP32-WROOM-32 安全ピン) ---
const int PIN_DRIVE_IN1 = 16; // 走行 前進 (L298N IN1)
const int PIN_DRIVE_IN2 = 17; // 走行 後退 (L298N IN2)
const int PIN_STEER_IN3 = 18; // 操舵 左折 (L298N IN3)
const int PIN_STEER_IN4 = 19; // 操舵 右折 (L298N IN4)

// --- PWM 設定 (ESP32 LEDC) ---
const int PWM_FREQ = 1000;    // 1kHz
const int PWM_RES  = 8;       // 8bit (0 - 255)
const int CH_DRIVE1 = 0;
const int CH_DRIVE2 = 1;
const int CH_STEER1 = 2;
const int CH_STEER2 = 3;

// 室内走行用 安全スピード上限 (最大255中、160に制限)
const int MAX_DRIVE_PWM = 160;

// 通信途絶監視タイマー
unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT_MS = 400;

// ステアリング過熱保護 (トイラジコンの振り切りステアリングを過熱から守る)
unsigned long steerStartTime = 0;
const unsigned long MAX_STEER_HOLD_MS = 600; // 0.6秒以上連続でハンドルを切り続けない

void emergencyStop() {
  ledcWrite(CH_DRIVE1, 0);
  ledcWrite(CH_DRIVE2, 0);
  ledcWrite(CH_STEER1, 0);
  ledcWrite(CH_STEER2, 0);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(15); // パース待機を15msに短縮 (1000msブロッキングによるウォッチドッグ誤作動を防止)

  // LEDC チャンネル設定 (ESP32 core 2.x/3.x 互換)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_DRIVE_IN1, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_DRIVE_IN2, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_STEER_IN3, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_STEER_IN4, PWM_FREQ, PWM_RES);
  #else
    ledcSetup(CH_DRIVE1, PWM_FREQ, PWM_RES);
    ledcSetup(CH_DRIVE2, PWM_FREQ, PWM_RES);
    ledcSetup(CH_STEER1, PWM_FREQ, PWM_RES);
    ledcSetup(CH_STEER2, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_DRIVE_IN1, CH_DRIVE1);
    ledcAttachPin(PIN_DRIVE_IN2, CH_DRIVE2);
    ledcAttachPin(PIN_STEER_IN3, CH_STEER1);
    ledcAttachPin(PIN_STEER_IN4, CH_STEER2);
  #endif

  emergencyStop();
  Serial.println("TOYOTA86_ESP32_LEDC_ONLINE");
}

// 走行モーター制御 (speed: -255 〜 +255)
void setDrive(int speed) {
  speed = constrain(speed, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  if (speed > 0) {
    ledcWrite(CH_DRIVE1, speed);
    ledcWrite(CH_DRIVE2, 0);
  } else if (speed < 0) {
    ledcWrite(CH_DRIVE1, 0);
    ledcWrite(CH_DRIVE2, -speed);
  } else {
    ledcWrite(CH_DRIVE1, 0);
    ledcWrite(CH_DRIVE2, 0);
  }
}

// 操舵モーター制御 (steer: -255 〜 +255, 負=左, 正=右)
void setSteer(int steer) {
  steer = constrain(steer, -255, 255);
  
  // ステアリング保持時間が長すぎる場合は力を弱めて過熱防止
  if (steer != 0) {
    if (steerStartTime == 0) steerStartTime = millis();
    else if (millis() - steerStartTime > MAX_STEER_HOLD_MS) {
      steer = (steer > 0) ? 120 : -120; // 保持トルクを半分に低減
    }
  } else {
    steerStartTime = 0;
  }

  if (steer < 0) {
    ledcWrite(CH_STEER1, -steer);
    ledcWrite(CH_STEER2, 0);
  } else if (steer > 0) {
    ledcWrite(CH_STEER1, 0);
    ledcWrite(CH_STEER2, steer);
  } else {
    ledcWrite(CH_STEER1, 0);
    ledcWrite(CH_STEER2, 0);
  }
}

void loop() {
  // 1. コマンド受信処理
  if (Serial.available() > 0) {
    char c = Serial.peek();
    lastCmdTime = millis();

    // フォーマット: "c,drive,steer\n" 例: "c,150,-100\n"
    if (c == 'c') {
      Serial.read(); // 'c'
      if (Serial.read() == ',') {
        int drive = Serial.parseInt();
        int steer = Serial.parseInt();
        setDrive(drive);
        setSteer(steer);
        while (Serial.available() && Serial.read() != '\n'); // flush
      }
    } else {
      // 1文字テスト用コマンド (PCシリアルモニタ等)
      char cmd = Serial.read();
      switch (cmd) {
        case 'f': setDrive(150); setSteer(0); break;     // 前進 (安全速度)
        case 'b': setDrive(-140); setSteer(0); break;    // 後退
        case 'l': setDrive(140); setSteer(-230); break;  // 前進左折
        case 'r': setDrive(140); setSteer(230); break;   // 前進右折
        case 's': emergencyStop(); break;                // 停止
      }
    }
  }

  // 2. フェイルセーフ監視 (通信途絶時は即停止)
  if (millis() - lastCmdTime > TIMEOUT_MS) {
    emergencyStop();
  }

  delay(5);
}
