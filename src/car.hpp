#pragma once

#include "Physics/PhysicsMaterial.h"
#include "Physics/Rigidbody.h"
#include "Vehicle/WheelCollider.h"
#include "meshbuild.hpp"
#include "track.hpp"

struct DriveInput {
    float throttle = 0;  // 0..1
    float brake = 0;     // 0..1  停止中は後退
    float steer = 0;     // -1..1  左が負、右が正
};

struct Car {
    // 車輪の並び。WheelCollider の配列の添字
    enum Wheel { kFrontLeft, kFrontRight, kRearLeft, kRearRight, kWheelCount };

    Car();
    Car(const Car&) = delete;             // 車輪がスタビライザーの相手を指すので、コピーしない
    Car& operator=(const Car&) = delete;

    Rigidbody body;         // 重心の位置、向き、速度、角速度
    WheelCollider wheels[kWheelCount];
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
    // 4 輪とも路面から離れているか
    bool Airborne() const;
    // 車の原点（車体の真下の地面の高さ）。描画とカメラの基準。重心から VehicleSpec::kCenterOfMassY 下
    Vec3 Origin() const;
    // 車輪の中心（車体の座標、原点から）。サスペンションの縮みを含む
    Vec3 WheelCenter(int wheel) const;

    // グリップしたまま進む。ドリフトの課題ではこの関数を差し替える。
    void stepGrip(float dt);
};

void buildCarBody(CpuMesh& mesh);
void buildCarWheel(CpuMesh& mesh);
void buildCarShadow(CpuMesh& mesh);
// 車の原点に置き、Rigidbody の向き（yaw / pitch / roll）に合わせる行列。
Mat4 carModelMatrix(const Car& car);
