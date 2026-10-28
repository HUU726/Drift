#pragma once

// 路面ごとの摩擦と反発。タイヤと衝突の両方が読む共有データ。
// 材質は使う段階で足す（縁石はタイヤモデル、壁は衝突、ジャンプ台は RampObject の段階）。
struct PhysicsMaterial {
    const char* name;
    float friction;           // μ。タイヤモデル（物理の段階 8）から使う
    float rollingResistance;  // 路面が足す転がり抵抗（m/s²）。タイヤ自身の分は車が持つ
    float restitution;        // 反発係数。衝突（物理の段階 4）から使う
    float gripScale;          // 駆動力の倍率。今の車モデル（Car::update）が使う
};

namespace Materials {

inline constexpr PhysicsMaterial kAsphalt{"Asphalt", 1.1f, 0.0f, 0.1f, 1.0f};
// 芝。加速が鈍く、速度も伸びない。横には滑らない。
inline constexpr PhysicsMaterial kRunoff{"Runoff", 0.6f, 3.2f, 0.1f, 0.55f};

}  // namespace Materials
