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

// Sixaxis Pair Tool でDS4に書き込む固定MACアドレス(汎用)
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
constexpr float HALF_WHEELBASE = 0.158f;  // ボディ中心 → 前後輪軸 (+Z)
constexpr float HALF_TRACK     = 0.130f;  // ボディ中心 → 左右輪 (+X)
constexpr float WHEEL_RADIUS   = 0.05f;  // タイヤ半径

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
// 速度・入力（スティック感度）
// ---------------------------------------------------------------------------
// フルスティックでのデューティ比率（0.0〜1.0 で指定）。
//   1.0 = フルスティックでモーター最大デューティ（255）
//   0.5 = フルスティックで 50% デューティ
constexpr float LINEAR_SPEED_RATIO   = 1.0f;  // 並進（左スティック Y/X）
constexpr float ROTATION_SPEED_RATIO = 1.0f;  // 旋回（右スティック X）、最外輪基準

constexpr float BOOST_MULTIPLIER = 2.2f;   // R1 押下中の速度倍率
constexpr float STICK_DEADZONE   = 0.20f;  // スティックのデッドゾーン（円形、0..1）

// サーボのスルーレート（deg/s）。60° / 0.13 秒 ≈ 461.5°/s。
// パターンB では 2:1 ギア相当のため .ino 側で 2 倍する。
constexpr float STEER_RATE_DEG_PER_SEC = 60.0f / 0.13f;

// ±189° 広角ステア（パターンB ワイドレンジギア）。
constexpr float MAX_STEER_DEG = 189.0f;

// ---------------------------------------------------------------------------
// 走行モーター（運動学 m/s → DRV8833 PWMデューティ）
// ---------------------------------------------------------------------------
// モーター最大地面速度（フルデューティ時の推定値）。duty = |speed| / this × 255。
constexpr float MAX_WHEEL_SPEED_MPS = 1.8f;
// 最外輪の旋回半径 = sqrt(HALF_TRACK² + HALF_WHEELBASE²) = sqrt(0.130²+0.158²)
constexpr float MAX_SPIN_RADIUS_M   = 0.2046f;
// 運動学への入力値（比率から自動計算、変更不要）
constexpr float MAX_LINEAR_SPEED      = LINEAR_SPEED_RATIO * MAX_WHEEL_SPEED_MPS;
constexpr float MAX_ANGULAR_SPEED_DEG = ROTATION_SPEED_RATIO * MAX_WHEEL_SPEED_MPS
                                        / MAX_SPIN_RADIUS_M * 57.2957795f;

constexpr float   DRIVE_SPEED_DEADBAND = 0.02f;   // これ未満はモーター停止（m/s）
constexpr uint8_t MOTOR_MIN_DUTY = 60;            // 静止摩擦を超える最低デューティ（要調整）
constexpr uint8_t MOTOR_MAX_DUTY = 255;           // 8ビットPWM

constexpr uint32_t MOTOR_PWM_FREQ = 1000;  // Hz
constexpr uint8_t  MOTOR_PWM_BITS = 8;     // 8ビット → 0..255

// ---------------------------------------------------------------------------
// DRV8833 ピン配置（ドライバー 3個）
//
// Driver A (ledc ×4): FL(ch1) + ML(ch2)
// Driver B (ledc ×4): FR(ch1) + MR(ch2)
// Driver C (ledc ×0): BL(ch1) + BR(ch2)
//   └─ 入力ピンは Driver A ch1 (FL) と Driver B ch1 (FR) を直結共用。
//      GPIO への ledcAttach は FL/FR 分のみ行い BL/BR は重複スキップ。
// ---------------------------------------------------------------------------

constexpr uint8_t MOTOR_IN1[WHEEL_COUNT] = {
  16, // FL 正転  (Driver A ch1)
  22, // FR 正転  (Driver B ch1)
  18, // ML 正転  (Driver A ch2)
  19, // MR 正転  (Driver B ch2)
  16, // BL 正転  (Driver C ch1 = Driver A ch1 共用)
  22  // BR 正転  (Driver C ch2 = Driver B ch1 共用)
};
constexpr uint8_t MOTOR_IN2[WHEEL_COUNT] = {
   4, // FL 逆転  (Driver A ch1)
  23, // FR 逆転  (Driver B ch1)
  17, // ML 逆転  (Driver A ch2)
  21, // MR 逆転  (Driver B ch2)
   4, // BL 逆転  (Driver C ch1 = Driver A ch1 共用)
  23  // BR 逆転  (Driver C ch2 = Driver B ch1 共用)
};

// ---------------------------------------------------------------------------
// ステアリングサーボ直結（DS3235 ×6、サーボドライバ基板なし、ledc ×6）
//
// 使用 GPIO: 旧 BL/BR IN ピン (17,18,19,21) と 旧 I2C ピン (25,26) を転用。
// ---------------------------------------------------------------------------
constexpr uint8_t SERVO_PIN[WHEEL_COUNT] = {
  32, // FL サーボ GPIO
  33, // FR サーボ GPIO
  25, // ML サーボ GPIO
  26, // MR サーボ GPIO
  27, // BL サーボ GPIO
  14  // BR サーボ GPIO
};

constexpr uint16_t SERVO_PWM_FREQ = 50;   // Hz（DS3235 標準）
constexpr uint8_t  SERVO_PWM_BITS = 16;   // 16ビット分解能（パルス幅精度向上）

// サーボのパルス幅キャリブレーション（マイクロ秒、0..180° の両端）。
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;

// 各輪のキャリブレーション。TRIM は機械的ゼロ点の微調整。
// 足の軸の外周が何mmズレているかを測る。時計回りにズレている場合は負とする
constexpr float SERVO_TRIM_MM[WHEEL_COUNT] = { -0.7f, -1.5f, -1.7f, -1.0f, -1.6f, -1.3f };
// TRIM を mm → deg に変換するための係数。
// 外形の直径が26mmかつ、ギア比が2倍なのでこの値になる
constexpr float SERVO_TRIM_MM_TO_DEG = 2.2f;
// REVERSED はサーボホーンが逆向きに取り付けられている輪を反転する。
constexpr bool  SERVO_REVERSED[WHEEL_COUNT] = {
  false, false, false, false, false, false
};

// ---------------------------------------------------------------------------
// 目玉サーボ（左右 / 上下 / まぶた、ledc ×3）
//
// ステアリングサーボと違いギアを介さない直結のため、可動角 = サーボ角そのもの。
// センター（90°）を 0° として EYE_MIN/MAX_OUTPUT_DEG[] の範囲で動かす。
// ---------------------------------------------------------------------------
enum EyeAxis : uint8_t {
  EYE_PAN  = 0,  // 左右（+ = 右）
  EYE_TILT = 1,  // 上下（+ = 上）
  EYE_LID  = 2,  // まぶた（+ = 開、- = 閉）
  EYE_COUNT = 3
};

static const char* const EYE_LABELS[EYE_COUNT] = { "PAN", "TILT", "LID" };

// 使用 GPIO: 走行モーター/ステアサーボで未使用のピンを割り当て。
// 13/15 は汎用、5 はストラッピングピン（起動時 HIGH 必須）だが内部プルアップ済みのため使用可。
constexpr uint8_t EYE_SERVO_PIN[EYE_COUNT] = {
  13, // 左右   (PAN)
  15, // 上下   (TILT)
   5  // まぶた (LID)
};

// 目玉サーボは LEDC ではなく RMT で駆動する。
// ESP32 の LEDC は最大 16 チャンネルしかなく、走行モーター 8 本
// （FL/FR/ML/MR の IN1+IN2。BL/BR は FL/FR と共用）＋ ステアサーボ 6 本 = 14 を
// 消費するため、目玉 3 本を足すと 17 本となり最後の 1 本（まぶた）の
// ledcAttach が必ず失敗していた（トリガーを押してもまぶたが動かない原因）。
// RMT は別ペリフェラル（TX 8 チャンネル）なので競合しない。
constexpr uint32_t EYE_RMT_TICK_HZ  = 1000000;  // 1 tick = 1 us（RMT は 312.5kHz〜80MHz）
constexpr uint16_t SERVO_PERIOD_US  = 20000;    // 50 Hz

// 十字キー押下中の左右／上下の移動速度（deg/s）。
constexpr float EYE_RATE_DEG_PER_SEC = 360.0f;

// 十字キーを離してからセンター（0°）へ自動復帰する速度（deg/s）。
constexpr float EYE_RETURN_RATE_DEG_PER_SEC = 180.0f;

// まぶた: R2 未押下 = 全開 / R2 全押し = 全閉。
constexpr float EYELID_OPEN_DEG   = -45.0f;
constexpr float EYELID_CLOSED_DEG = +30.0f;

// 軸ごとの出力角リミット（applyEye のクランプ用）。
// まぶたは PAN/TILT と可動範囲が異なるため、共通の ±MAX ではなく軸ごとに持つ。
constexpr float EYE_MIN_OUTPUT_DEG[EYE_COUNT] = {
  -30.0f, -20.0f, EYELID_OPEN_DEG
};
constexpr float EYE_MAX_OUTPUT_DEG[EYE_COUNT] = {
  +30.0f, +40.0f, EYELID_CLOSED_DEG
};

// DS4 アナログトリガー（L2/R2）の最大値。PS4-esp32 は 0..255 を返す。
constexpr float PS4_TRIGGER_MAX = 255.0f;

// 機械的ゼロ点の微調整（度）。
constexpr float EYE_TRIM_DEG[EYE_COUNT] = { 3.0f, 3.0f, 0.0f };
// サーボホーンが逆向きに付いている軸を反転する。
constexpr bool EYE_SERVO_REVERSED[EYE_COUNT] = { false, false, false };

// ---------------------------------------------------------------------------
// 制御ループ
// ---------------------------------------------------------------------------
constexpr uint16_t CONTROL_PERIOD_MS = 20;  // ~50 Hz
constexpr float    MAX_DT_SEC = 0.10f;      // 停止後の再開時に dt が跳ね上がるのを防ぐ
