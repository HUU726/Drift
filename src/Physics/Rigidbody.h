#pragma once

#include "geom.hpp"

#include <cmath>

// 剛体の状態。position は重心。サスペンションとタイヤの力、重力、空気抵抗を積み上げて積分する。
struct Rigidbody {
    float mass = 1300.0f;          // kg
    float inertiaYaw = 2630.0f;    // kg·m²。上下の軸まわり
    float inertiaPitch = 2350.0f;  // 左右の軸まわり
    float inertiaRoll = 500.0f;    // 前後の軸まわり
    float dragCoefficient = 0;     // 空気抵抗（N / (m/s)²）。水平の速度の 2 乗に比例する
    Vec3 position;
    Vec3 velocity;          // m/s。ワールド座標
    float yaw = 0;          // 向き。+z が 0、右回りが正
    float pitch = 0;        // 鼻先が上がるのが正
    float roll = 0;         // 右側が上がるのが正
    float yawRate = 0;      // rad/s
    float pitchRate = 0;
    float rollRate = 0;

    // 水平面での向き。壁の判定と、車体から見た速度に使う。
    Vec3 Forward() const { return {std::sin(yaw), 0, std::cos(yaw)}; }
    Vec3 Right() const { return {std::cos(yaw), 0, -std::sin(yaw)}; }
    // 車体から見た前向きと右向きの速度。ドリフトでは LateralSpeed が 0 でなくなる。
    float ForwardSpeed() const { return dot(velocity, Forward()); }
    float LateralSpeed() const { return dot(velocity, Right()); }

    // pitch と roll を含む車体の軸。
    void Axes(Vec3& right, Vec3& up, Vec3& forward) const {
        forward = normalize(Vec3{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)});
        Vec3 right0 = Right();
        Vec3 up0 = normalize(cross(forward, right0));
        right = normalize(right0 * std::cos(roll) + up0 * std::sin(roll));
        up = normalize(cross(forward, right));
    }
    // 車体の座標（重心から、x 右、y 上、z 前）をワールド座標にする。
    Vec3 ToWorld(Vec3 local) const {
        Vec3 right, up, forward;
        Axes(right, up, forward);
        return position + right * local.x + up * local.y + forward * local.z;
    }

    // point（ワールド座標）での速度。回転による分を含む。
    Vec3 VelocityAt(Vec3 point) const {
        Vec3 right, up, forward;
        Axes(right, up, forward);
        // 角速度のベクトル。pitch は鼻先が上がる向きが正なので、右向きの軸まわりでは負になる。
        Vec3 omega = Vec3{0, yawRate, 0} + right * -pitchRate + forward * rollRate;
        return velocity + cross(omega, point - position);
    }

    // point に力 f を加える（次の Integrate まで積み上げる）。
    void AddForceAt(Vec3 f, Vec3 point) {
        Vec3 right, up, forward;
        Axes(right, up, forward);
        Vec3 torque = cross(point - position, f);
        force_ = force_ + f;
        pitchTorque_ -= dot(torque, right);
        rollTorque_ += dot(torque, forward);
        yawTorque_ += torque.y;
    }

    // 積み上げた力と重力、空気抵抗で速度と角速度を進め、位置と向きを進める（半陰的オイラー法）。
    void Integrate(float dt, float gravity) {
        float speed = std::sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
        force_.x -= dragCoefficient * velocity.x * speed;
        force_.z -= dragCoefficient * velocity.z * speed;
        velocity = velocity + force_ * (dt / mass);
        velocity.y -= gravity * dt;
        yawRate += yawTorque_ / inertiaYaw * dt;
        pitchRate += pitchTorque_ / inertiaPitch * dt;
        rollRate += rollTorque_ / inertiaRoll * dt;
        position = position + velocity * dt;
        yaw += yawRate * dt;
        pitch += pitchRate * dt;
        roll += rollRate * dt;
        force_ = Vec3{};
        pitchTorque_ = 0;
        rollTorque_ = 0;
        yawTorque_ = 0;
    }

    // 車体の軸まわりのトルク（N·m。yaw は右回り、pitch は鼻先が上がる向き、roll は右側が上がる向きが正）を、
    // dt のあいだ加える。空中姿勢制御が使う。
    void AddTorque(float yawTorque, float pitchTorque, float rollTorque, float dt) {
        yawRate += yawTorque / inertiaYaw * dt;
        pitchRate += pitchTorque / inertiaPitch * dt;
        rollRate += rollTorque / inertiaRoll * dt;
    }

    // point に上向きの力積 j（N·s）を加える。上下の速度と pitch / roll が変わる（着地の底突き用）。
    void AddVerticalImpulseAt(float j, Vec3 point) {
        float pitchArm, rollArm;
        verticalArms(point, pitchArm, rollArm);
        velocity.y += j / mass;
        pitchRate += pitchArm * j / inertiaPitch;
        rollRate += rollArm * j / inertiaRoll;
    }
    // point を上下に動かすときの、質量の逆数にあたる値（回転のしやすさを含む）。
    float VerticalInverseMassAt(Vec3 point) const {
        float pitchArm, rollArm;
        verticalArms(point, pitchArm, rollArm);
        return 1.0f / mass + pitchArm * pitchArm / inertiaPitch + rollArm * rollArm / inertiaRoll;
    }

    // point に力積 j を加える。水平の成分だけを使い、回転は上下の軸まわりだけ（壁の衝突用）。
    void AddImpulseAt(Vec3 j, Vec3 point) {
        Vec3 r = point - position;
        velocity = velocity + Vec3{j.x, 0, j.z} * (1.0f / mass);
        yawRate += (r.z * j.x - r.x * j.z) / inertiaYaw;
    }

private:
    // point に上向きの単位の力を加えたときの、pitch と roll のトルク（AddForceAt と同じ符号）。
    void verticalArms(Vec3 point, float& pitchArm, float& rollArm) const {
        Vec3 right, up, forward;
        Axes(right, up, forward);
        Vec3 torque = cross(point - position, Vec3{0, 1, 0});
        pitchArm = -dot(torque, right);
        rollArm = dot(torque, forward);
    }

    Vec3 force_;
    float pitchTorque_ = 0;
    float rollTorque_ = 0;
    float yawTorque_ = 0;
};
