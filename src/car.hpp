#pragma once

#include "Physics/PhysicsMaterial.h"
#include "Physics/Rigidbody.h"
#include "meshbuild.hpp"
#include "track.hpp"

struct DriveInput {
    float throttle = 0;  // 0..1
    float brake = 0;     // 0..1  停止中は後退
    float steer = 0;     // -1..1  左が負、右が正
};

struct Car {
    Rigidbody body;         // 位置、向き、速度、ヨーレート
    float steerAngle = 0;   // 前輪の切れ角（ラジアン、右が正）
    float wheelSpin = 0;
    float brakeHold = 0;    // 止まってから後退に入るまでの待ち時間
    float gripYawRate = 0;  // 前のステップでグリップが決めたヨーレート。衝突で加わった回転との差を見る
    const PhysicsMaterial* surface = &Materials::kAsphalt;  // 車の下の路面
    TrackPose ground;

    void reset(const Track& track);
    void update(float dt, const DriveInput& input, const Track& track);
    // 前向きの速度（m/s）。正が前進
    float ForwardSpeed() const { return body.ForwardSpeed(); }

    // グリップしたまま進む。ドリフトの課題ではこの関数を差し替える。
    void stepGrip(float dt);
};

void buildCarBody(CpuMesh& mesh);
void buildCarWheel(CpuMesh& mesh);
void buildCarShadow(CpuMesh& mesh);
Mat4 carModelMatrix(const Car& car, float pitch, float roll);
