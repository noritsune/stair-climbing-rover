// RoverConfig.h
// 実機ローバーのピン配置・定数定義。
//
// 座標系（シミュレーターと同一）:
//   +Z 前方 / +X 右 / +Y 上。
//   ステア角 0° = 前方 (+Z)、正方向 = 右 (+X)、範囲 ±180°。
//
// 輪の並び順（コード内固定）: FL, FR, ML, MR, BL, BR

#pragma once

// ---------------------------------------------------------------------------
// コントローラー
// ---------------------------------------------------------------------------

// Sixaxis Pair Tool でDS4に書き込む固定MACアドレス（全マイコン共通）。
#define PS4_MAC_ADDRESS "aa:bb:cc:dd:ee:ff"

// ---------------------------------------------------------------------------
// 輪のインデックス（固定順序）
// ---------------------------------------------------------------------------
enum WheelIndex : uint8_t {
  W_FL = 0,  // front  左
  W_FR = 1,  // front  右
  W_ML = 2,  // middle 左
  W_MR = 3,  // middle 右
  W_BL = 4,  // back   左
  W_BR = 5,  // back   右
  WHEEL_COUNT = 6
};

static const char* const WHEEL_LABELS[WHEEL_COUNT] = {
  "FL", "FR", "ML", "MR", "BL", "BR"
};

// ---------------------------------------------------------------------------
// 寸法（メートル）
// ---------------------------------------------------------------------------
constexpr float HALF_WHEELBASE = 0.75f;  // ボディ中心 → 前後輪軸 (+Z)
constexpr float HALF_TRACK     = 0.60f;  // ボディ中心 → 左右輪 (+X)
constexpr float WHEEL_RADIUS   = 0.22f;  // タイヤ半径

// 各輪の平面位置（x = 右オフセット, y = 前方オフセット）、FL..BR 順。
constexpr float WHEEL_POS_X[WHEEL_COUNT] = {
  -HALF_TRACK,  +HALF_TRACK,   // FL, FR
  -HALF_TRACK,  +HALF_TRACK,   // ML, MR
  -HALF_TRACK,  +HALF_TRACK    // BL, BR
};
constexpr float WHEEL_POS_Y[WHEEL_COUNT] = {
  +HALF_WHEELBASE, +HALF_WHEELBASE,  // FL, FR
  0.0f,            0.0f,             // ML, MR
  -HALF_WHEELBASE, -HALF_WHEELBASE   // BL, BR
};

// ---------------------------------------------------------------------------
// 速度・入力
// ---------------------------------------------------------------------------
constexpr float MAX_LINEAR_SPEED      = 1.8f;   // m/s（並進最大速度）
constexpr float MAX_ANGULAR_SPEED_DEG = 80.0f;  // deg/s（旋回最大角速度）
constexpr float BOOST_MULTIPLIER      = 2.2f;   // R1 押下中の速度倍率
constexpr float STICK_DEADZONE        = 0.12f;  // スティックのデッドゾーン（円形、0..1）

// サーボのスルーレート（deg/s）。60° / 0.13 秒 ≈ 461.5°/s。
// パターンB では 2:1 ギア相当のため .ino 側で 2 倍する。
constexpr float STEER_RATE_DEG_PER_SEC = 60.0f / 0.13f;

// ±180° 広角ステア（パターンB ワイドレンジギア）。
constexpr float MAX_STEER_DEG = 180.0f;

// ---------------------------------------------------------------------------
// 走行モーター（運動学 m/s → DRV8833 PWMデューティ）
// ---------------------------------------------------------------------------
// デューティ正規化の基準速度。フルスティック（ブーストなし）でこの速度 = 最大デューティ。
// ブースト（R1）押下時は運動学出力がこれを超えるが、constrain() で 255 にクランプされる。
// 大きくすると最高速が下がる。小さくすると体感速度が上がる（モーターへの負担に注意）。
constexpr float MAX_WHEEL_SPEED_MPS = MAX_LINEAR_SPEED;  // 1.8 m/s → フルスティック = 100% PWM

constexpr float   DRIVE_SPEED_DEADBAND = 0.02f;   // これ未満はモーター停止（m/s）
constexpr uint8_t MOTOR_MIN_DUTY = 0;             // 停動するなら 40 程度に上げる
constexpr uint8_t MOTOR_MAX_DUTY = 255;           // 8ビットPWM

constexpr uint32_t MOTOR_PWM_FREQ = 1000;  // Hz
constexpr uint8_t  MOTOR_PWM_BITS = 8;     // 8ビット → 0..255

// ---------------------------------------------------------------------------
// DRV8833 ピン配置（ドライバー 3個 × 各2モーター）
// IN1 = 正転 / IN2 = 逆転。
// MOTOR_REVERSED: モーターが逆向きに取り付けられている輪を反転する。
// 左輪は右輪と鏡像になるため、通常は左輪（FL/ML/BL）を true にする。
// ---------------------------------------------------------------------------
constexpr bool MOTOR_REVERSED[WHEEL_COUNT] = {
  true, // FL
  false,  // FR
  true, // ML
  false,  // MR
  true, // BL
  false,  // BR
};

constexpr uint8_t MOTOR_IN1[WHEEL_COUNT] = {
  32, // FL 正転
  16, // FR 正転
  27, // ML 正転
  23, // MR 正転
  18, // BL 正転
  21  // BR 正転
};
constexpr uint8_t MOTOR_IN2[WHEEL_COUNT] = {
  33, // FL 逆転
   4, // FR 逆転
  14, // ML 逆転
  22, // MR 逆転
  17, // BL 逆転
  19  // BR 逆転
};

// ---------------------------------------------------------------------------
// PCA9685 ステアリングサーボ
// ---------------------------------------------------------------------------
constexpr uint8_t  PCA9685_I2C_SDA  = 13;   // I2C SDA
constexpr uint8_t  PCA9685_I2C_SCL  = 26;   // I2C SCL
constexpr uint8_t  PCA9685_I2C_ADDR = 0x40;
constexpr float    PCA9685_OSC_FREQ = 27000000.0f;  // writeMicroseconds() 用
constexpr uint16_t SERVO_PWM_FREQ   = 50;           // Hz（標準アナログサーボ）

// 各輪の PCA9685 チャンネル（FL..BR 順）。
constexpr uint8_t SERVO_CHANNEL[WHEEL_COUNT] = { 5, 4, 3, 2, 1, 0 };

// サーボのパルス幅キャリブレーション（マイクロ秒、0..180° の両端）。
// 実機サーボに合わせて調整する。一般的な値: 500..2500 us。
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;

// 各輪のキャリブレーション。TRIM は機械的ゼロ点の微調整。
// REVERSED はサーボホーンが逆向きに取り付けられている輪を反転する。
constexpr float SERVO_TRIM_DEG[WHEEL_COUNT] = { 0, 0, 0, 0, 0, 0 };
constexpr bool  SERVO_REVERSED[WHEEL_COUNT] = {
  false, false, false, false, false, false
};

// ---------------------------------------------------------------------------
// 制御ループ
// ---------------------------------------------------------------------------
constexpr uint16_t CONTROL_PERIOD_MS = 20;  // ~50 Hz
constexpr float    MAX_DT_SEC = 0.10f;      // 停止後の再開時に dt が跳ね上がるのを防ぐ
