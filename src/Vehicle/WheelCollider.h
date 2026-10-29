#pragma once

#include "Physics/Collider.h"

class ITireModel;

// 車輪 1 つ分のサスペンションとタイヤ。
// サスペンション: 取付点から真下へレイを飛ばし、縮みからバネとダンパーの力を求める。
// タイヤ: 車輪の回転と路面の上の速度から滑りを求め、ITireModel で路面から受ける力を求める。
// ColliderManager がサブステップごとに Update、Force、StepTire の順に呼び、力を車体に加える。
class WheelCollider : public Collider {
public:
    // 設定（車体の座標。重心から、x 右、y 上、z 前）
    Vec3 mount;
    float radius = 0.34f;
    float restLength = 0.30f;
    float springRate = 32000.0f;
    float damping = 2300.0f;
    float bumpDampingScale = 0.8f;
    float reboundDampingScale = 1.3f;
    float bumpStop = 0.18f;
    float bumpStopRate = 320000.0f;
    float bumpStopReturn = 0.2f;    // 伸びるときにバンプストップが返す力の割合
    WheelCollider* antiRollPartner = nullptr;  // 同じ車軸の反対側
    float antiRollRate = 0;
    const ITireModel* tire = nullptr;
    float wheelInertia = 1.2f;   // kg·m²
    float rollingDecel = 0.9f;   // タイヤ自身の転がり抵抗（車を減速させる加速度、m/s²）
    // タイヤの力を掛ける高さ（接地点からの上、m）。0 なら接地点。上げるほど、曲がるときに車体を傾ける力が小さくなる
    //（横の力の腕が短くなる）。滑りの計算は接地点で行う。
    float forceAppHeight = 0;

    // 操作（Car が固定ステップごとに決める）
    float steer = 0;        // 切れ角（rad、右が正）
    float driveTorque = 0;  // N·m。前へ回す向きが正
    float brakeTorque = 0;  // N·m。回転を止める向きに掛かる大きさ
    bool handbrake = false; // サイドブレーキが掛かっている。ABS を効かせず、車輪をロックさせる
    bool drifting = false;  // ドリフト中。トラクションコントロールを効かせず、アクセルで空転させる
    bool antiLock = true;           // ABS: ブレーキで滑りすぎないように回転を保つ
    bool tractionControl = true;    // 駆動で滑りすぎないように回転を抑える
    float slipControlShare = 0.8f;  // 上の 2 つが許すスリップ率（ピークのスリップ率に対する割合）

    // 状態
    bool grounded = false;
    float compression = 0;  // 自然長からの縮み（m）
    float length = 0.30f;   // 取付点から車輪の中心まで（m）。描画に使う
    float load = 0;         // 路面に掛けている力（N）
    float lastS = -1;       // 直前に当たった路面の周回位置
    float groundY = 0;      // 直前に当たった路面の高さ
    const PhysicsMaterial* surface = nullptr;  // 直前に当たった路面の材質
    float omega = 0;        // 回転の速さ（rad/s）。前へ転がる向きが正
    float spinAngle = 0;    // 描画用の回転角
    float slipRatio = 0;
    float slipAngle = 0;
    bool sliding = false;   // タイヤがピークを越えて滑っている

    // レイの結果から縮みとバネの力を求める（スタビライザーの分は含まない）。
    void Update(const RaycastHit& hit, Vec3 mountWorld, float dt);
    // スタビライザーを含めた、車体を押し上げる力（N）。
    float Force();
    // タイヤの力（ワールド座標）とその作用点（接地点）を求め、車輪の回転を dt だけ進める。
    void StepTire(float load, float dt, float gravity, Vec3& force, Vec3& point);
    void Reset();

private:
    // ABS とトラクションコントロールで、回転の速さ w を滑りすぎない範囲に収める。
    float limitSlip(float w, float vx, float speed, float slipAngle) const;

    float springForce_ = 0;
    float prevCompression_ = 0;
    bool hasPrev_ = false;
};
