#pragma once

#include "Physics/PhysicsMaterial.h"
#include "Physics/Rigidbody.h"
#include "Vehicle/AirControl.h"
#include "Vehicle/Powertrain.h"
#include "Vehicle/TireModel.h"
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
    float brakeHold = 0;    // 止まってから後退に入るまでの待ち時間
    TrackPose ground;       // 車の下のコース（周回位置、横位置）。ラップ計測と HUD が読む

    void reset(const Track& track);
    // 入力から前輪の切れ角と、各輪の駆動・ブレーキのトルクを決める。車を動かすのは ColliderManager。
    void update(float dt, const DriveInput& input, const Track& track);
    // 後輪をドリフト用のタイヤにする（false でグリップ用に戻す）。前輪はグリップ用のまま。
    void UseDriftTires(bool drift);
    // 駆動をモーターにする（false でエンジンに戻す）。
    void UseMotor(bool motor);
    const IPowertrain& Powertrain() const { return *powertrain; }
    // 前向きの速度（m/s）。正が前進
    float ForwardSpeed() const { return body.ForwardSpeed(); }
    // 4 輪とも路面から離れているか
    bool Airborne() const;
    // 車の原点（車体の真下の地面の高さ）。描画とカメラの基準。重心から VehicleSpec::kCenterOfMassY 下
    Vec3 Origin() const;
    // 車輪の中心（車体の座標、原点から）。サスペンションの縮みを含む
    Vec3 WheelCenter(int wheel) const;

    // タイヤのモデル。ドリフトの課題では、ここを差し替えるか、新しいモデルを足す。
    GripTire gripTire;
    DriftTire driftTire;
    // 駆動系。エンジンとモーターのどちらかを使う。
    EnginePowertrain engine;
    MotorPowertrain motor;
    IPowertrain* powertrain = &engine;
    // 空中姿勢制御。設定（有効・無効、強さ、着地補助）はここで変える。
    AirControl airControl;
};

void buildCarBody(CpuMesh& mesh);
void buildCarWheel(CpuMesh& mesh);
void buildCarShadow(CpuMesh& mesh);
// 車の原点に置き、Rigidbody の向き（yaw / pitch / roll）に合わせる行列。
Mat4 carModelMatrix(const Car& car);
