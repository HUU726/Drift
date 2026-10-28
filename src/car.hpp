#pragma once

#include "Physics/PhysicsMaterial.h"
#include "Physics/Rigidbody.h"
#include "Vehicle/AirControl.h"
#include "Vehicle/Powertrain.h"
#include "Vehicle/TireModel.h"
#include "Vehicle/VehicleSpec.h"
#include "Vehicle/WheelCollider.h"
#include "meshbuild.hpp"
#include "track.hpp"

#include <functional>

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
    TrackPose ground;       // 車の下のコース（周回位置、横位置）。ラップ計測と HUD が読む。コースがなければ found = false

    // 運転と車体の値。試験コースの調整の画面でプレイ中に変えられる。既定の値は VehicleSpec と同じ。
    float brakeDecel = 30.0f;        // ブレーキいっぱいで出したい減速（m/s²）。実際の減速はタイヤで決まる
    float brakeFrontShare = 0.6f;    // ブレーキの前の割合
    float maxSteer = 0.46f;          // 止まっているときの最大の切れ角（rad）
    float steerSpeedFalloff = 0.085f;  // 速度 1 m/s あたりで最大の切れ角を減らす割合
    float steerRate = 2.6f;          // ハンドルを切る速さ（rad/s）
    float mass = VehicleSpec::kMass;
    float dragPerMass = VehicleSpec::kDrag;
    float centerOfMassY = VehicleSpec::kCenterOfMassY;  // 原点から重心までの高さ

    // 真下の路面を調べる関数（x, z, レイの始点の高さ, 直前の周回位置）。空中姿勢制御の着地補助が使う。
    // CarObject と self-test が ColliderManager::GroundAt を渡す。なければ着地補助は効かない。
    std::function<RaycastHit(float, float, float, float)> groundProbe;

    // コースのスポーン地点に置き直す。
    void reset(const Track& track);
    // 原点（車体の真下の地面）を origin に、向きを yaw にして、止まった状態で置き直す。
    void Place(Vec3 origin, float yaw);
    // 入力から前輪の切れ角と、各輪の駆動・ブレーキのトルクを決める。車を動かすのは ColliderManager。
    // track があれば、車の下のコースの位置（ground）も求める。
    void update(float dt, const DriveInput& input, const Track* track);
    // 後輪をドリフト用のタイヤにする（false でグリップ用に戻す）。前輪はグリップ用のまま。
    void UseDriftTires(bool drift);
    bool RearDriftTires() const { return wheels[kRearLeft].tire == &rearDriftTire; }
    // 駆動をモーターにする（false でエンジンに戻す）。
    void UseMotor(bool motor);
    bool UsingMotor() const { return powertrain == &motor; }
    // 質量（慣性モーメントと空気抵抗も合わせる）、空気抵抗、重心の高さを、走っている車のまま変える。
    void SetMass(float newMass);
    void SetDrag(float perMass);
    void SetCenterOfMassY(float height);
    const IPowertrain& Powertrain() const { return *powertrain; }
    // 前向きの速度（m/s）。正が前進
    float ForwardSpeed() const { return body.ForwardSpeed(); }
    // 4 輪とも路面から離れているか
    bool Airborne() const;
    // 車の原点（車体の真下の地面の高さ）。描画とカメラの基準。重心から centerOfMassY 下
    Vec3 Origin() const;
    // 車輪の中心（車体の座標、原点から）。サスペンションの縮みを含む
    Vec3 WheelCenter(int wheel) const;

    // タイヤのモデル。前輪と後輪で別々に持ち、試験コースの調整の画面で前後を別々に変えられる。
    // 後輪はグリップ用とドリフト用を UseDriftTires で切り替える。ドリフトの課題では、ここを差し替えるか、
    // 新しいモデルを足す。
    GripTire frontTire;
    GripTire rearGripTire;
    DriftTire rearDriftTire;
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
