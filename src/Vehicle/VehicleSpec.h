#pragma once

// 車の寸法と足回りの値。描画（CarObject）と物理（Car、WheelCollider）が同じ値を読む。
// 位置は車の原点（車体の真下の地面、前後は前輪と後輪のほぼ中間）からの距離で、x が右、y が上、z が前。
namespace VehicleSpec {

constexpr float kMass = 1300.0f;            // kg
constexpr float kCenterOfMassY = 0.45f;     // 原点から重心までの高さ（m）
// 慣性モーメント（kg·m²）。幅 1.9 m、長さ 4.55 m、高さ 1.0 m の箱から
constexpr float kInertiaYaw = 2630.0f;
constexpr float kInertiaPitch = 2350.0f;
constexpr float kInertiaRoll = 500.0f;
// 空気抵抗。速度の 2 乗に掛けて減速（1/m）。エンジンの最大出力（約 230 kW）で、
// 最高速度が 82 m/s（約 295 km/h）前後になる値
constexpr float kDrag = 0.00029f;

constexpr float kWheelRadius = 0.34f;
constexpr float kWheelX = 0.82f;            // 車輪の左右の位置（トレッド 1.64 m）
constexpr float kFrontWheelZ = 1.42f;
constexpr float kRearWheelZ = -1.38f;       // ホイールベース 2.80 m
constexpr float kMountY = 0.54f;            // サスペンションの取付点の高さ
constexpr float kBodyBoxCenterY = 0.66f;    // 当たり判定の箱（車体）の中心の高さ

// サスペンション。止まっているとき 0.10 m 縮み、車輪の中心が 0.34 m（今の車高）になる。
constexpr float kRestLength = 0.30f;        // 取付点から車輪の中心までの自然長
constexpr float kSpringRate = 32000.0f;     // N/m。1 輪あたり
constexpr float kDamping = 2300.0f;         // N·s/m。減衰比 0.35
constexpr float kBumpDampingScale = 0.8f;   // 縮む側
constexpr float kReboundDampingScale = 1.3f;  // 伸びる側
constexpr float kBumpStop = 0.18f;          // これを超えて縮んだら硬いバネを足す
constexpr float kBumpStopRate = kSpringRate * 10.0f;
constexpr float kBumpStopReturn = 0.2f;     // 伸びるときにバンプストップが返す力の割合
constexpr float kFrontAntiRoll = 12000.0f;  // N/m。左右の縮みの差に掛ける
constexpr float kRearAntiRoll = 8000.0f;

// タイヤと車輪
constexpr float kWheelInertia = 1.2f;       // kg·m²
constexpr float kRollingDecel = 0.15f;      // タイヤ自身の転がり抵抗（車を減速させる加速度、m/s²。μr 約 0.015）
constexpr bool kTractionControl = true;     // 駆動輪が空転しないように回転を抑える（切るとパワースライド）
constexpr bool kAntiLockBrakes = true;      // ブレーキで車輪がロックしないように回転を保つ
constexpr float kSlipControlShare = 0.8f;   // 上の 2 つが許すスリップ率（ピークに対する割合）

// 空中姿勢制御（ゲーム用の補助）
constexpr bool kAirControl = true;
constexpr float kAirControlStrength = 1.0f;  // 角加速度の上限に掛ける倍率
constexpr bool kLandingAssist = true;        // 路面が近づいたら車体を路面の傾きへ寄せる

}  // namespace VehicleSpec
