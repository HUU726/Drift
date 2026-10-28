#pragma once

#include "geom.hpp"

#include <cmath>

// 剛体の状態。物理の段階 3 では水平面の運動だけを扱う。
// 高さ（position.y）は路面に張り付けたままで、velocity.y は使わない（段階 6 で 3D にする）。
// 力の積み上げ（AddForce / AddForceAt）と衝突のインパルスは、使う段階で足す。
struct Rigidbody {
    float mass = 1300.0f;  // kg
    Vec3 position;
    Vec3 velocity;         // m/s。ワールド座標
    float yaw = 0;         // 向き。+z が 0、右回りが正
    float yawRate = 0;     // rad/s。右回りが正

    Vec3 Forward() const { return {std::sin(yaw), 0, std::cos(yaw)}; }
    Vec3 Right() const { return {std::cos(yaw), 0, -std::sin(yaw)}; }
    // 車体から見た前向きと右向きの速度。ドリフトでは LateralSpeed が 0 でなくなる。
    float ForwardSpeed() const { return dot(velocity, Forward()); }
    float LateralSpeed() const { return dot(velocity, Right()); }
};
