/**
 * ジョーゼン 1/16 トヨタ86 (JRVC015-RD) 自動運転ファームウェア v2.0
 * Phone-as-a-Brain (Galaxy Note 10+) 連携版
 *
 * 特徴:
 * 1. 400ms 通信途絶時の自動緊急ブレーキ (Fail-Safe Watchdog)
 * 2. 急発進・急制動によるスリップやカメラブレを防ぐスムーズ加減速 (Slew Rate Limiting)
 * 3. 前後4WDモーター ＋ ステアリングPWM制御
 *
 * 対応マイコン: ESP32 または Arduino Nano (USBシリアル 115200bps)
 */

// モータードライバー (DRV8833 / L298N) ピン設定
const int PIN_DRIVE_PWM1 = 5;  // 走行 前進 (PWM)
const int PIN_DRIVE_PWM2 = 6;  // 走行 後退 (PWM)
const int PIN_STEER_PWM1 = 9;  // 操舵 左折 (PWM)
const int PIN_STEER_PWM2 = 10; // 操舵 右折 (PWM)

// 速度制御用変数 (スムージング)
int currentDrive = 0;
int targetDrive = 0;
int currentSteer = 0;
int targetSteer = 0;

unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT_MS = 400; // 400ms信号が来なければ自動緊急停止

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DRIVE_PWM1, OUTPUT);
  pinMode(PIN_DRIVE_PWM2, OUTPUT);
  pinMode(PIN_STEER_PWM1, OUTPUT);
  pinMode(PIN_STEER_PWM2, OUTPUT);
  emergencyStop();
  Serial.println("TOYOTA86_FIRMWARE_V2_ONLINE");
}

void emergencyStop() {
  currentDrive = 0;
  targetDrive = 0;
  currentSteer = 0;
  targetSteer = 0;
  analogWrite(PIN_DRIVE_PWM1, 0);
  analogWrite(PIN_DRIVE_PWM2, 0);
  analogWrite(PIN_STEER_PWM1, 0);
  analogWrite(PIN_STEER_PWM2, 0);
}

// ハードウェアへの実際のPWM出力
void applyMotors(int drive, int steer) {
  // 走行
  if (drive > 0) {
    analogWrite(PIN_DRIVE_PWM1, drive);
    analogWrite(PIN_DRIVE_PWM2, 0);
  } else if (drive < 0) {
    analogWrite(PIN_DRIVE_PWM1, 0);
    analogWrite(PIN_DRIVE_PWM2, -drive);
  } else {
    analogWrite(PIN_DRIVE_PWM1, 0);
    analogWrite(PIN_DRIVE_PWM2, 0);
  }

  // 操舵
  if (steer < 0) {
    analogWrite(PIN_STEER_PWM1, -steer);
    analogWrite(PIN_STEER_PWM2, 0);
  } else if (steer > 0) {
    analogWrite(PIN_STEER_PWM1, 0);
    analogWrite(PIN_STEER_PWM2, steer);
  } else {
    analogWrite(PIN_STEER_PWM1, 0);
    analogWrite(PIN_STEER_PWM2, 0);
  }
}

void loop() {
  // 1. スマホからのコマンド受信
  // フォーマット: "c,drive,steer\n" 例: "c,200,-120\n"
  if (Serial.available() > 0) {
    char c = Serial.peek();
    if (c == 'c') {
      Serial.read(); // 'c'
      if (Serial.read() == ',') {
        targetDrive = constrain(Serial.parseInt(), -255, 255);
        targetSteer = constrain(Serial.parseInt(), -255, 255);
        lastCmdTime = millis();
        while (Serial.available() && Serial.read() != '\n'); // flush
      }
    } else {
      char cmd = Serial.read();
      lastCmdTime = millis();
      switch (cmd) {
        case 'f': targetDrive = 190; targetSteer = 0; break;
        case 'b': targetDrive = -170; targetSteer = 0; break;
        case 'l': targetDrive = 180; targetSteer = -210; break;
        case 'r': targetDrive = 180; targetSteer = 210; break;
        case 's': emergencyStop(); break;
      }
    }
  }

  // 2. フェイルセーフ (通信途絶時の自動停止)
  if (millis() - lastCmdTime > TIMEOUT_MS) {
    targetDrive = 0;
    targetSteer = 0;
  }

  // 3. スムーズ加減速 (ステップごとに目標速度へ近づける)
  const int ACCEL_STEP = 15;
  if (currentDrive < targetDrive) currentDrive = min(currentDrive + ACCEL_STEP, targetDrive);
  else if (currentDrive > targetDrive) currentDrive = max(currentDrive - ACCEL_STEP, targetDrive);

  currentSteer = targetSteer; // ステアリングは機敏に応答

  applyMotors(currentDrive, currentSteer);
  delay(10); // 100Hzループ
}
