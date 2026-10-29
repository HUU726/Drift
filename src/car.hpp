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
    bool handbrake = false;  // サイドブレーキ。押している間、後輪をロックして後ろを流す
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
    float handbrakeDecel = 25.0f;    // サイドブレーキの強さ（後輪だけで出したい減速、m/s²）。後輪がロックする大きさ
    float maxSteer = 0.46f;          // 止まっているときの最大の切れ角（rad）
    float steerSpeedFalloff = 0.085f;  // 速度 1 m/s あたりで最大の切れ角を減らす割合
    float steerRate = 2.6f;          // ハンドルを切る速さ（rad/s）
    bool steerAssist = true;         // 前輪の滑りの角度がピークを越えないように切れ角を抑える
    float steerAssistShare = 0.9f;   // 補助が許す滑りの角度（ピークに対する割合）
    float mass = VehicleSpec::kMass;
    float dragPerMass = VehicleSpec::kDrag;
    float centerOfMassY = VehicleSpec::kCenterOfMassY;  // 原点から重心までの高さ
    float inertiaScale = 1.0f;       // 慣性モーメント（回りにくさ）の倍率。質量に比例した値にさらに掛ける

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
    bool RearDriftTires() const { return rearDriftTires_; }
    // 駆動をモーターにする（false でエンジンに戻す）。
    void UseMotor(bool motor);
    bool UsingMotor() const { return powertrain == &motor; }
    // 質量（慣性モーメントと空気抵抗も合わせる）、空気抵抗、重心の高さを、走っている車のまま変える。
    void SetMass(float newMass);
    void SetDrag(float perMass);
    void SetCenterOfMassY(float height);
    void SetInertiaScale(float scale);
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
    // ドリフト中の後輪。普段の後輪の特性を写し、μ だけ前輪の driftRearGrip 倍にする（update で毎回作り直す）。
    GripTire driftModeTire;

    // ドリフト（D1 の単走のように、横を向いたまま滑り続ける）。サイドブレーキを押している間だけドリフトになり、
    // 後輪のグリップを下げ、トラクションコントロールを切り、前輪は自動で逆ハンドルを当てる。押し始めはサイドブレーキで
    // 後ろを流し、角度が driftEnterAngle を越えたらブレーキを外す。離すと普段の車へ戻る。
    // ハンドルの入力は行きたい向き、アクセルは角度の深さ（離すと driftMinAngle、全開で driftMaxAngle）になる。
    bool driftAssist = true;         // ドリフトの状態を使うか（切ると普段の車のまま）
    float driftEnterAngle = 0.21f;   // サイドブレーキを外す角度（rad、約 12°）。車体の滑りの角度（進む向きと車体の向きの差）
    float driftRearGrip = 0.8f;      // ドリフト中の後輪の μ / 前輪の μ
    float driftMinAngle = 0.35f;     // アクセルを離したときに保つ角度（rad、約 20°）
    float driftMaxAngle = 0.96f;     // アクセル全開で保つ角度（rad、約 55°）
    float driftAngleAssist = 1.0f;   // 角度を保つために車体を回す補助の強さ
    float driftMaxSteer = 0.96f;     // ドリフト中の逆ハンドルの最大の切れ角（rad、約 55°）
    // ドリフト中の曲がり。スティックを倒しきったときに、進む向きが回る速さ（rad/s）。倒す量に比例し、離すとまっすぐ。
    float driftCurveRate = 0.6f;     // 約 34°/s
    float driftCurveAssist = 1.0f;   // 曲がりをその速さへ寄せる補助の強さ（0 で補助なし、大きいほどその速さに近づく）
    // 状態
    bool drifting = false;
    float driftAngle = 0;            // 車体の滑りの角度（rad、右へ滑るのが正。右のコーナーでは負）
    // 駆動系。エンジンとモーターのどちらかを使う。
    EnginePowertrain engine;
    MotorPowertrain motor;
    IPowertrain* powertrain = &engine;
    // 空中姿勢制御。設定（有効・無効、強さ、着地補助）はここで変える。
    AirControl airControl;

private:
    // ドリフトの状態を進め、ドリフト中の後輪と角度の補助を決める。update から呼ぶ。
    void updateDrift(float dt, const DriveInput& input, float speed);

    bool rearDriftTires_ = false;
    bool driftKicked_ = false;  // ドリフトに入ってから、サイドブレーキで向きを変え終えたか
    float prevDriftAngle_ = 0;
    float pathRate_ = 0;  // 進む向きの回る速さ（rad/s、ならしたもの）
};

void buildCarBody(CpuMesh& mesh);
void buildCarWheel(CpuMesh& mesh);
void buildCarShadow(CpuMesh& mesh);
// 車の原点に置き、Rigidbody の向き（yaw / pitch / roll）に合わせる行列。
Mat4 carModelMatrix(const Car& car);
