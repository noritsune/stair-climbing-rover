// RoverKinematics.h
// ハードウェア非依存のピュア運動学。パターンB（±189° 広角ステア）に対応。
//
// Arduino / PCA / DRV の依存を持たないため、ホスト PC で単体テスト可能。
// .ino からインクルードし、計算結果をサーボ・モーターへ渡す。
//
// 座標系（RoverConfig.h と同一）:
//   ボディ局所座標: +Z 前方 / +X 右。
//   ステア角 0° = 前方 / ±189° が最大範囲。
//   ヨーレート正 = 右旋回（上から見て時計回り）。
#pragma once

#include <math.h>

namespace rover {

constexpr float KIN_EPSILON = 1e-4f;
constexpr float KIN_DEG2RAD = 0.01745329252f;
constexpr float KIN_RAD2DEG = 57.2957795131f;

// ボディの指令速度（ローカル座標系）。
struct BodyTwist {
  float forward;     // +Z 方向速度 m/s
  float right;       // +X 方向速度 m/s（横移動 / カニ歩き）
  float yawRateDeg;  // ヨーレート deg/s（正 = 右旋回）
};

// 1輪の指令値: 向く角度と転動速度。
struct WheelCommand {
  float steerDeg;    // 0° = 前方 (+Z)、±189° 範囲
  float driveSpeed;  // 符号付き地面速度 m/s（負 = 後退方向に転動）
};

// スティック → BodyTwist の変換に必要な上限値。
struct KinLimits {
  float maxLinearSpeed;     // m/s
  float maxAngularSpeedDeg; // deg/s
  float stickDeadzone;      // 円形デッドゾーン、正規化 0..1
};

// --- 小ヘルパー関数 -------------------------------------------------------

inline float kinClamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// current → target の最短角度差（-180, 180]。Mathf.DeltaAngle に相当。
inline float kinDeltaAngle(float current, float target) {
  float d = fmodf(target - current, 360.0f);
  if (d >  180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

// スルーレート制限付き移動（非ラッピング）。
// サーボは端点をまたいで回転できないため、線形補間が物理的に正しい。
// Mathf.MoveTowards に相当。
inline float kinMoveTowards(float current, float target, float maxStep) {
  float diff = target - current;
  if (fabsf(diff) <= maxStep) return target;
  return current + (diff > 0.0f ? maxStep : -maxStep);
}

// 円形デッドゾーン（端から滑らかにランプアップ）。
// Mathf.ApplyDeadzone に相当。
inline void kinApplyDeadzone(float& x, float& y, float deadzone) {
  float magnitude = sqrtf(x * x + y * y);
  if (magnitude < deadzone || magnitude < KIN_EPSILON) {
    x = 0.0f;
    y = 0.0f;
    return;
  }
  float scaled = kinClamp((magnitude - deadzone) / (1.0f - deadzone), 0.0f, 1.0f);
  float inv = scaled / magnitude;
  x *= inv;
  y *= inv;
}

// --- コア運動学 -----------------------------------------------------------

// 生スティック値（-1..1）をボディツイストに変換する。
// speedScale はブースト倍率などの最終乗数。
// RoverKinematics.MapSticks に相当。
inline BodyTwist mapSticks(float leftX, float leftY,
                           float rightX, float rightY,
                           const KinLimits& lim, float speedScale) {
  kinApplyDeadzone(leftX, leftY, lim.stickDeadzone);
  kinApplyDeadzone(rightX, rightY, lim.stickDeadzone);

  // 左スティック: Y = 前後、X = 左右並進。
  float forward = leftY * lim.maxLinearSpeed;
  float strafe  = leftX * lim.maxLinearSpeed;

  // 斜め入力が最大速度を超えないようにクランプ。
  float mag = sqrtf(forward * forward + strafe * strafe);
  if (mag > lim.maxLinearSpeed && mag > KIN_EPSILON) {
    float clamp = lim.maxLinearSpeed / mag;
    forward *= clamp;
    strafe  *= clamp;
  }

  // 右スティック: X = 右旋回。
  float yaw = rightX * lim.maxAngularSpeedDeg;

  BodyTwist t;
  t.forward    = forward * speedScale;
  t.right      = strafe * speedScale;
  t.yawRateDeg = yaw * speedScale;
  return t;
}

// パターンB の 1輪ごとの解（RoverKinematics.ComputeWheelWideRange に相当）。
// サーボは後方半球も含めて目標方向へ直接向く（ドライブは常に正転が基本）。
// .ino 側でさらに逆転フリップ最適化を重ねる。
//
//   posX = 輪の右方向オフセット / posY = 前方向オフセット（メートル）
inline WheelCommand computeWheelWideRange(float posX, float posY,
                                          const BodyTwist& twist,
                                          float previousSteerDeg,
                                          float maxSteerDeg) {
  float omega = twist.yawRateDeg * KIN_DEG2RAD;  // rad/s、+Y 周り
  float vx = twist.right   + omega * posY;        // 右方向速度成分
  float vz = twist.forward - omega * posX;        // 前方向速度成分

  float speed = sqrtf(vx * vx + vz * vz);
  if (speed < KIN_EPSILON) {
    // 指令なし: ゼロに戻さず現在角を保持（ハンチング防止）。
    return WheelCommand{ previousSteerDeg, 0.0f };
  }

  // +Z を基準とした走行方向（+X 方向が正）。[-189, 189]
  float raw   = atan2f(vx, vz) * KIN_RAD2DEG;
  float steer = kinClamp(raw, -maxSteerDeg, maxSteerDeg);
  return WheelCommand{ steer, speed };  // ドライブは常に正
}

}  // namespace rover
