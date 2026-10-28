#pragma once

#include "geom.hpp"

#include <cmath>

// 剛体の状態。物理の段階 3 では水平面の運動だけを扱う。
// 高さ（position.y）は路面に張り付けたままで、velocity.y は使わない（段階 6 で 3D にする）。
// 力の積み上げ（AddForce / AddForceAt）は、使う段階で足す。
struct Rigidbody {
    float mass = 1300.0f;         // kg
    float inertiaYaw = 2630.0f;   // kg·m²。上下の軸まわり。幅 1.9 m、長さ 4.55 m の箱から
    Vec3 position;
    Vec3 velocity;         // m/s。ワールド座標
    float yaw = 0;         // 向き。+z が 0、右回りが正
    float yawRate = 0;     // rad/s。右回りが正

    Vec3 Forward() const { return {std::sin(yaw), 0, std::cos(yaw)}; }
    Vec3 Right() const { return {std::cos(yaw), 0, -std::sin(yaw)}; }
    // 車体から見た前向きと右向きの速度。ドリフトでは LateralSpeed が 0 でなくなる。
    float ForwardSpeed() const { return dot(velocity, Forward()); }
    float LateralSpeed() const { return dot(velocity, Right()); }

    // point（ワールド座標）での速度。回転による分を含む。
    Vec3 VelocityAt(Vec3 point) const {
        Vec3 r = point - position;
        return velocity + Vec3{yawRate * r.z, 0, -yawRate * r.x};
    }
    // point に力積 j を加える。水平面だけなので、回転は上下の軸まわりの成分だけを使う。
    void AddImpulseAt(Vec3 j, Vec3 point) {
        Vec3 r = point - position;
        velocity = velocity + Vec3{j.x, 0, j.z} * (1.0f / mass);
        yawRate += (r.z * j.x - r.x * j.z) / inertiaYaw;
    }
};
