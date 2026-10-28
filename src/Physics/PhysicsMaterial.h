#pragma once

// 路面ごとの摩擦と反発。タイヤと衝突の両方が読む共有データ。
struct PhysicsMaterial {
    const char* name;
    float friction;           // μ。タイヤの力の上限と、壁の摩擦に使う
    float rollingResistance;  // 路面が足す転がり抵抗（m/s²）。タイヤ自身の分は車が持つ
    float restitution;        // 反発係数。衝突に使う
};

namespace Materials {

inline constexpr PhysicsMaterial kAsphalt{"Asphalt", 1.1f, 0.0f, 0.1f};
// 縁石。路面の外側 curb_width の帯。少し滑りやすい。
inline constexpr PhysicsMaterial kCurb{"Curb", 0.95f, 0.0f, 0.1f};
// 芝。滑りやすく、転がり抵抗が大きいので速度も伸びない。
// 転がり抵抗は、芝の上の後輪で出せる加速（約 3.4 m/s²）より十分小さくする。大きいと芝から出られない。
inline constexpr PhysicsMaterial kRunoff{"Runoff", 0.6f, 1.2f, 0.1f};
// 壁。車体が当たったときの摩擦と反発に使う。
inline constexpr PhysicsMaterial kWall{"Wall", 0.3f, 0.0f, 0.2f};
// ジャンプ台。上面はアスファルトと同じように走れ、側面に当たったときはあまり跳ね返らない。
inline constexpr PhysicsMaterial kRamp{"Ramp", 1.0f, 0.0f, 0.05f};

}  // namespace Materials
