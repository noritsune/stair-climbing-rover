// stair-climbing-rover.ino
// 6輪全輪ステアローバー実機用 ESP32 ファームウェア。
//
// 操縦感は Unity 製 rover-simulator の「パターンB (B_WideRange)」を実機へ移植したもの:
//   * 各輪は広角（2:1 ギア）サーボを GPIO 直結 ledc で駆動し、±180° の範囲で直接目標方向を向く。
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
//
// 操作方法（モード共通）:
//   R1（押しっぱなし）: ブースト（離すと 0.7 倍）
//   Options         : 全ステアを 0° にリセンター
//   Share           : 操作モード切替
//
// [通常モード（デフォルト）]
//   左スティック Y  : 前進 / 後退
//   左スティック X  : 左右並進（カニ歩き）
//   右スティック X  : 旋回
//
// [タンクモード]
//   左スティック Y  : 左タイヤ（FL/ML/BL）正転 / 逆転
//   右スティック Y  : 右タイヤ（FR/MR/BR）正転 / 逆転
//   全サーボ        : 常に 0°

#include <PS4Controller.h>

#include "RoverConfig.h"
#include "RoverKinematics.h"

using rover::BodyTwist;
using rover::WheelCommand;
using rover::KinLimits;

// ---------------------------------------------------------------------------
// 状態変数
// ---------------------------------------------------------------------------

// 各ステアリングサーボの出力角（度、±180）のソフトウェアモデル。
// スルーレート制限はここで行い、実際に向いている角度をドライブゲートにも使う。
static float currentSteerDeg[WHEEL_COUNT] = { 0, 0, 0, 0, 0, 0 };

static const KinLimits kLimits = {
    MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED_DEG, STICK_DEADZONE
};

// ---------------------------------------------------------------------------
// 操作モード
// ---------------------------------------------------------------------------
enum DriveMode : uint8_t {
  MODE_NORMAL = 0,  // デフォルト: パターンB 広角ステア
  MODE_TANK   = 1   // タンク: 左右スティック Y で左右タイヤ独立制御、全サーボ 0°
};

static DriveMode driveMode   = MODE_NORMAL;
static bool prevOptions  = false;  // Options ボタンのエッジ検出用
static bool prevShare    = false;  // Share ボタンのエッジ検出用
static bool wasConnected = false;  // 接続エッジ検出用（接続時に LED 色を設定するため）
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
    // BL/BR は FL/FR と同じピンを共用するため、重複 attach をスキップ
    bool duplicate = false;
    for (uint8_t j = 0; j < i; j++) {
      if (MOTOR_IN1[j] == MOTOR_IN1[i]) { duplicate = true; break; }
    }
    if (duplicate) {
      Serial.printf("[Motor] %s  IN1=GPIO%d(共用)  IN2=GPIO%d(共用)\n",
        WHEEL_LABELS[i], MOTOR_IN1[i], MOTOR_IN2[i]);
      continue;
    }
    bool ok1 = ledcAttach(MOTOR_IN1[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    bool ok2 = ledcAttach(MOTOR_IN2[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    Serial.printf("[Motor] %s  IN1=GPIO%d(%s)  IN2=GPIO%d(%s)\n",
      WHEEL_LABELS[i],
      MOTOR_IN1[i], ok1 ? "OK" : "NG",
      MOTOR_IN2[i], ok2 ? "OK" : "NG");
  }
  stopAllMotors();
}

void setupServos() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    bool ok = ledcAttach(SERVO_PIN[i], SERVO_PWM_FREQ, SERVO_PWM_BITS);
    Serial.printf("[Servo] %s  GPIO%d(%s)\n",
      WHEEL_LABELS[i], SERVO_PIN[i], ok ? "OK" : "NG");
  }

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
  bool connected = PS4.isConnected();
  if (!connected) {
    stopAllMotors();           // フェイルセーフ: 駆動を停止、ステアは保持
    Serial.println("DS4 接続待ち...");
    wasConnected = false;
    delay(100);
    lastUpdateMs = millis();   // 再接続後に dt が大きくなるのを防ぐ
    return;
  }
  // 接続直後（立ち上がりエッジ）に現在モードの LED 色を設定する
  if (!wasConnected) {
    applyModeColor();
    wasConnected = true;
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
  float speedScale = PS4.R1() ? BOOST_MULTIPLIER : 0.7f;

  // Options ボタン: 全ステアを 0° にリセンター（立ち上がりエッジのみ）
  bool options = PS4.Options();
  if (options && !prevOptions) recenterSteering();
  prevOptions = options;

  // Share ボタン: 操作モード切替（立ち上がりエッジのみ）
  bool share = PS4.Share();
  if (share && !prevShare) {
    driveMode = (driveMode == MODE_NORMAL) ? MODE_TANK : MODE_NORMAL;
    Serial.printf("モード切替: %s\n", driveMode == MODE_NORMAL ? "通常（広角ステア）" : "タンク（左右独立）");
    applyModeColor();
  }
  prevShare = share;

  // --- デバッグ出力（問題解析中のみ。確認後は #if 0 で無効化）-----------
#if 1
  static unsigned long dbgPrint = 0;
  if (millis() - dbgPrint > 200) {
    dbgPrint = millis();
    Serial.printf("raw LX=%d LY=%d RX=%d RY=%d\n",
        PS4.LStickX(), PS4.LStickY(), PS4.RStickX(), PS4.RStickY());
  }
#endif

  // --- 配線確認モード（スティック中立時にボタン単押しで個別輪を回転）----------
  // 十字上 : FL+BL 正転   十字下 : FL+BL 逆転
  // 十字左 : ML   正転   十字右 : ML   逆転
  // △     : FR+BR 正転   ✕     : FR+BR 逆転
  // ○     : MR   正転   □     : MR   逆転
  bool sticksNeutral = (fabsf(leftX) < STICK_DEADZONE
                     && fabsf(leftY) < STICK_DEADZONE
                     && fabsf(rightX) < STICK_DEADZONE);
  bool wireTestHandled = false;
  if (sticksNeutral) {
    constexpr float WIRE_TEST_SPEED = 1.0f;
    float flCmd = 0.0f, mlCmd = 0.0f, frCmd = 0.0f, mrCmd = 0.0f;
    const char* wireLabel = nullptr;

    if (PS4.Up()) {
      flCmd = +WIRE_TEST_SPEED; wireLabel = "FL+BL 正転";
    } else if (PS4.Down()) {
      flCmd = -WIRE_TEST_SPEED; wireLabel = "FL+BL 逆転";
    } else if (PS4.Left()) {
      mlCmd = +WIRE_TEST_SPEED; wireLabel = "ML 正転";
    } else if (PS4.Right()) {
      mlCmd = -WIRE_TEST_SPEED; wireLabel = "ML 逆転";
    } else if (PS4.Triangle()) {
      frCmd = +WIRE_TEST_SPEED; wireLabel = "FR+BR 正転";
    } else if (PS4.Cross()) {
      frCmd = -WIRE_TEST_SPEED; wireLabel = "FR+BR 逆転";
    } else if (PS4.Circle()) {
      mrCmd = +WIRE_TEST_SPEED; wireLabel = "MR 正転";
    } else if (PS4.Square()) {
      mrCmd = -WIRE_TEST_SPEED; wireLabel = "MR 逆転";
    }

    if (wireLabel) {
      stopAllMotors();
      if (flCmd != 0.0f) applyDrive(W_FL, flCmd);
      if (mlCmd != 0.0f) applyDrive(W_ML, mlCmd);
      if (frCmd != 0.0f) applyDrive(W_FR, frCmd);
      if (mrCmd != 0.0f) applyDrive(W_MR, mrCmd);
      wireTestHandled = true;
      static unsigned long dbgWire = 0;
      if (millis() - dbgWire > 500) {
        dbgWire = millis();
        Serial.printf("[配線確認] %s\n", wireLabel);
      }
    }
  }

  // --- 運動学 → 駆動 -------------------------------------------------------
  if (!wireTestHandled) {
    if (driveMode == MODE_TANK) {
      updateTankMode(leftY, rightY, speedScale);
    } else {
      BodyTwist twist =
          rover::mapSticks(leftX, leftY, rightX, rightY, kLimits, speedScale);
      updateWheels(twist, dt);
    }
  }

  delay(CONTROL_PERIOD_MS);
}

// ---------------------------------------------------------------------------
// パターンB 車輪更新
// (rover-simulator の RoverController.UpdateWheels Mode B ブランチの移植)
// ---------------------------------------------------------------------------
void updateWheels(const BodyTwist& command, float dt) {
  // パターンB は 2:1 ワイドレンジギア相当のため、スルーレートを 2 倍にする。
  float maxStep = STEER_RATE_DEG_PER_SEC * dt * 2.0f;

  // --- ステア更新 + 各輪のドライブ速度を計算 --------------------------------
  float driveOut[WHEEL_COUNT];
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
    applySteer(i, currentSteerDeg[i]);

    // 追従中は cos(誤差) でドライブをゲーティング（横滑り防止）。
    float errorRad  = rover::kinDeltaAngle(currentSteerDeg[i], target.steerDeg) * rover::KIN_DEG2RAD;
    float alignment = cosf(errorRad);
    if (alignment < 0.0f) alignment = 0.0f;
    driveOut[i] = target.driveSpeed * alignment;
  }

  // --- FL/BL、FR/BR は共用ピンなので平均値で駆動 ---------------------------
  float avgFLBL = (driveOut[W_FL] + driveOut[W_BL]) * 0.5f;
  float avgFRBR = (driveOut[W_FR] + driveOut[W_BR]) * 0.5f;

  // デバッグ: 各輪の指令値（問題解析中のみ）
#if 1
  static unsigned long dbgDrive = 0;
  if (millis() - dbgDrive > 200) {
    dbgDrive = millis();
    float dbgDrives[WHEEL_COUNT] = {
      avgFLBL, avgFRBR, driveOut[W_ML], driveOut[W_MR], avgFLBL, avgFRBR
    };
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
      if (fabsf(dbgDrives[i]) > DRIVE_SPEED_DEADBAND || fabsf(currentSteerDeg[i]) > 1.0f) {
        Serial.printf("  [%s] steer=%.1f drive=%.3f\n",
            WHEEL_LABELS[i], currentSteerDeg[i], dbgDrives[i]);
      }
    }
  }
#endif

  // 共用ピンは FL/FR 側のみ書き込む（BL/BR は物理的に同じ信号を受ける）。
  applyDrive(W_FL, avgFLBL);
  applyDrive(W_FR, avgFRBR);
  applyDrive(W_ML, driveOut[W_ML]);
  applyDrive(W_MR, driveOut[W_MR]);
}

// ---------------------------------------------------------------------------
// タンクモード更新
// 左スティック Y → 左タイヤ全輪（FL/ML/BL）、右スティック Y → 右タイヤ全輪（FR/MR/BR）。
// 全サーボは 0° に固定。BL/BR は FL/FR と共用ピンのため FL/FR への書き込みで兼用。
// ---------------------------------------------------------------------------
static float tankDeadzone1D(float v) {
  float a = fabsf(v);
  if (a < STICK_DEADZONE) return 0.0f;
  float scaled = (a - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
  return (v > 0.0f ? 1.0f : -1.0f) * rover::kinClamp(scaled, 0.0f, 1.0f);
}

void updateTankMode(float leftY, float rightY, float speedScale) {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    currentSteerDeg[i] = 0.0f;
    applySteer(i, 0.0f);
  }

  float leftSpeed  = tankDeadzone1D(leftY)  * MAX_WHEEL_SPEED_MPS * speedScale;
  float rightSpeed = tankDeadzone1D(rightY) * MAX_WHEEL_SPEED_MPS * speedScale;

  applyDrive(W_FL, leftSpeed);   // FL + BL（共用ピン）
  applyDrive(W_ML, leftSpeed);
  applyDrive(W_FR, rightSpeed);  // FR + BR（共用ピン）
  applyDrive(W_MR, rightSpeed);
}

// ---------------------------------------------------------------------------
// コントローラー LED 色: 通常モード = 青 (0,0,255) / タンクモード = 黄 (255,200,0)
// ---------------------------------------------------------------------------
void applyModeColor() {
  if (driveMode == MODE_NORMAL) {
    PS4.setLed(0, 0, 255);
  } else {
    PS4.setLed(255, 200, 0);
  }
  PS4.sendToController();
}

void recenterSteering() {
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    currentSteerDeg[i] = 0.0f;
    applySteer(i, 0.0f);
  }
  Serial.println("全ステアを 0° にリセンター");
}

// ---------------------------------------------------------------------------
// ハードウェア出力: ステアリングサーボ（DS3235 GPIO 直結、ledc）
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
  servoDeg += SERVO_TRIM_MM[wheel] * SERVO_TRIM_MM_TO_DEG;
  servoDeg = rover::kinClamp(servoDeg, 0.0f, 180.0f);

  uint16_t us = (uint16_t)(SERVO_MIN_US +
      (servoDeg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));

  // ledc duty = us / period_us * (2^bits)
  // period_us = 1,000,000 / SERVO_PWM_FREQ = 20,000 us
  constexpr uint32_t PERIOD_US = 1000000UL / SERVO_PWM_FREQ;
  uint32_t duty = (uint32_t)us * (1UL << SERVO_PWM_BITS) / PERIOD_US;
  ledcWrite(SERVO_PIN[wheel], duty);
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
