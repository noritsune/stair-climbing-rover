// stair-climbing-rover.ino
// 6輪全輪ステアローバー実機用 ESP32 ファームウェア。
//
// 操縦感は Unity 製 rover-simulator の「パターンB (B_WideRange)」を実機へ移植したもの:
//   * 各輪は広角（2:1 ギア）サーボを PCA9685 で駆動し、±180° の範囲で直接目標方向を向く。
//   * ステアのスルーレートは 2 倍（2:1 ギア相当）。
//   * 逆転フリップ最適化: 目標まで 90° 超の旋回が必要なときは、モーターを逆転して
//     目標±180° を狙い、サーボの実際の移動量を 90° 未満に抑える。
//   * サーボ追従中はモーター出力を cos(誤差角) でゲーティングし、横滑りを防ぐ。
//
// 起動時は全輪 0°（サーボはセンター）。
//
// コントローラー: DualShock 4（Bluetooth、PS4-esp32 ライブラリ）。
// 受信方式は G:\Documents\Arduino\strandbeest-radio-control\strandbeest-radio-control.ino に準拠。
//
// esp32 ボードパッケージは 3.1.3 を使用すること（それ以降は PS4-esp32 がコンパイルエラー）。
// 詳細: https://forum.arduino.cc/t/compilar-library-ps4-controler/1373660
//
// 必要ライブラリ（Arduino Library Manager でインストール）:
//   * PS4-esp32  (aed3)
//   * Adafruit PWM Servo Driver Library
//
// 操作方法:
//   左スティック Y  : 前進 / 後退
//   左スティック X  : 左右並進（カニ歩き）
//   右スティック X  : 旋回
//   R1（押しっぱなし）: ブースト
//   Options         : 全ステアを 0° にリセンター

#include <Wire.h>
#include <PS4Controller.h>
#include <Adafruit_PWMServoDriver.h>

#include "RoverConfig.h"
#include "RoverKinematics.h"

using rover::BodyTwist;
using rover::WheelCommand;
using rover::KinLimits;

// ---------------------------------------------------------------------------
// 状態変数
// ---------------------------------------------------------------------------
static Adafruit_PWMServoDriver pwm =
    Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);

// 各ステアリングサーボの出力角（度、±180）のソフトウェアモデル。
// スルーレート制限はここで行い、実際に向いている角度をドライブゲートにも使う。
static float currentSteerDeg[WHEEL_COUNT] = { 0, 0, 0, 0, 0, 0 };

static const KinLimits kLimits = {
    MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED_DEG, STICK_DEADZONE
};

static bool prevOptions = false;   // Options ボタンのエッジ検出用
static unsigned long lastUpdateMs = 0;

// ---------------------------------------------------------------------------
// セットアップ
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMotors();
  setupServos();

  PS4.begin(PS4_MAC_ADDRESS);
  Serial.print("DS4 ペアリング MAC: ");
  Serial.println(PS4_MAC_ADDRESS);

  lastUpdateMs = millis();
  Serial.println("ローバー起動完了（パターンB / 広角ステア）");
}

void setupMotors() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    ledcAttach(MOTOR_IN1[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttach(MOTOR_IN2[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  }
  stopAllMotors();
}

void setupServos() {
  Wire.begin(PCA9685_I2C_SDA, PCA9685_I2C_SCL);
  pwm.begin();
  pwm.setOscillatorFrequency(PCA9685_OSC_FREQ);
  pwm.setPWMFreq(SERVO_PWM_FREQ);
  delay(10);

  // 起動時: 全輪 0°（サーボはセンター）
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    currentSteerDeg[i] = 0.0f;
    applySteer(i, 0.0f);
  }
}

// ---------------------------------------------------------------------------
// メインループ
// ---------------------------------------------------------------------------
void loop() {
  if (!PS4.isConnected()) {
    stopAllMotors();           // フェイルセーフ: 駆動を停止、ステアは保持
    Serial.println("DS4 接続待ち...");
    delay(100);
    lastUpdateMs = millis();   // 再接続後に dt が大きくなるのを防ぐ
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  lastUpdateMs = now;
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > MAX_DT_SEC) dt = MAX_DT_SEC;

  // --- コントローラー読み取り ----------------------------------------------
  // PS4-esp32 のスティック値は int8 (-128..127)。-1..1 に正規化する。
  float leftX  = normStick(PS4.LStickX());
  float leftY  = normStick(PS4.LStickY());
  float rightX = normStick(PS4.RStickX());
  float rightY = normStick(PS4.RStickY());
  float speedScale = PS4.R1() ? BOOST_MULTIPLIER : 1.0f;

  // Options ボタン: 全ステアを 0° にリセンター（立ち上がりエッジのみ）
  bool options = PS4.Options();
  if (options && !prevOptions) recenterSteering();
  prevOptions = options;

  // --- 運動学 → 駆動 -------------------------------------------------------
  BodyTwist twist =
      rover::mapSticks(leftX, leftY, rightX, rightY, kLimits, speedScale);
  updateWheels(twist, dt);

  delay(CONTROL_PERIOD_MS);
}

// ---------------------------------------------------------------------------
// パターンB 車輪更新
// (rover-simulator の RoverController.UpdateWheels Mode B ブランチの移植)
// ---------------------------------------------------------------------------
void updateWheels(const BodyTwist& command, float dt) {
  // パターンB は 2:1 ワイドレンジギア相当のため、スルーレートを 2 倍にする。
  float maxStep = STEER_RATE_DEG_PER_SEC * dt * 2.0f;

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    WheelCommand target = rover::computeWheelWideRange(
        WHEEL_POS_X[i], WHEEL_POS_Y[i], command,
        currentSteerDeg[i], MAX_STEER_DEG);

    // 逆転フリップ最適化: 目標まで 90° 超回す必要があるとき、
    // モーターを逆転して目標±180° を狙えば移動量が 90° 未満に収まる。
    float directTravel = fabsf(rover::kinDeltaAngle(currentSteerDeg[i], target.steerDeg));
    if (directTravel > 90.0f) {
      float flipSteer = target.steerDeg >= 0.0f
                            ? target.steerDeg - 180.0f
                            : target.steerDeg + 180.0f;
      target.steerDeg   = flipSteer;
      target.driveSpeed = -target.driveSpeed;
    }

    // サーボモデルをスルーレート制限しながら目標へ近づける。
    currentSteerDeg[i] = rover::kinMoveTowards(currentSteerDeg[i], target.steerDeg, maxStep);
    float actual = currentSteerDeg[i];
    applySteer(i, actual);

    // 追従中は cos(誤差) でドライブをゲーティング（横滑り防止）。
    float errorRad  = rover::kinDeltaAngle(actual, target.steerDeg) * rover::KIN_DEG2RAD;
    float alignment = cosf(errorRad);
    if (alignment < 0.0f) alignment = 0.0f;
    float drive = target.driveSpeed * alignment;

    applyDrive(i, drive);
  }
}

void recenterSteering() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    currentSteerDeg[i] = 0.0f;
    applySteer(i, 0.0f);
  }
  Serial.println("全ステアを 0° にリセンター");
}

// ---------------------------------------------------------------------------
// ハードウェア出力: ステアリングサーボ（PCA9685）
// ---------------------------------------------------------------------------
// outputDeg は車輪のステア角（±180°）。
// 2:1 ワイドレンジギア: servo = outputDeg / 2 + 90
//   出力 -180° → サーボ  0°（左端）
//   出力    0° → サーボ 90°（センター）
//   出力 +180° → サーボ180°（右端）
void applySteer(uint8_t wheel, float outputDeg) {
  outputDeg = rover::kinClamp(outputDeg, -MAX_STEER_DEG, MAX_STEER_DEG);

  float servoDeg = outputDeg * 0.5f + 90.0f;          // ギア変換 + センタリング
  if (SERVO_REVERSED[wheel]) servoDeg = 180.0f - servoDeg;
  servoDeg += SERVO_TRIM_DEG[wheel];
  servoDeg = rover::kinClamp(servoDeg, 0.0f, 180.0f);

  uint16_t us = (uint16_t)(SERVO_MIN_US +
      (servoDeg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
  pwm.writeMicroseconds(SERVO_CHANNEL[wheel], us);
}

// ---------------------------------------------------------------------------
// ハードウェア出力: 走行モーター（DRV8833）
// ---------------------------------------------------------------------------
// driveSpeed は符号付き m/s。IN1 = 正転 / IN2 = 逆転。両方 0 でコースト停止。
void applyDrive(uint8_t wheel, float driveSpeed) {
  float mag = fabsf(driveSpeed);
  if (mag < DRIVE_SPEED_DEADBAND) {
    ledcWrite(MOTOR_IN1[wheel], 0);
    ledcWrite(MOTOR_IN2[wheel], 0);
    return;
  }

  int duty = (int)(mag / MAX_WHEEL_SPEED_MPS * MOTOR_MAX_DUTY);
  duty = constrain(duty, 0, MOTOR_MAX_DUTY);
  // 最低デューティ未満でも動かす設定なら底上げ（停動防止）
  if (duty > 0 && duty < MOTOR_MIN_DUTY) duty = MOTOR_MIN_DUTY;

  if (driveSpeed >= 0.0f) {
    ledcWrite(MOTOR_IN1[wheel], duty);
    ledcWrite(MOTOR_IN2[wheel], 0);
  } else {
    ledcWrite(MOTOR_IN1[wheel], 0);
    ledcWrite(MOTOR_IN2[wheel], duty);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    ledcWrite(MOTOR_IN1[i], 0);
    ledcWrite(MOTOR_IN2[i], 0);
  }
}

// ---------------------------------------------------------------------------
// ユーティリティ
// ---------------------------------------------------------------------------
// DS4 スティック値 (-128..127) を正規化 (-1..1) に変換する。
float normStick(int raw) {
  float v = raw / 127.0f;
  return rover::kinClamp(v, -1.0f, 1.0f);
}
