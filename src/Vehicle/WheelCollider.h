#pragma once

#include "Physics/Collider.h"

// 車輪 1 つ分のサスペンション。取付点から真下へレイを飛ばし、縮みからバネとダンパーの力を求める。
// ColliderManager が固定ステップのサブステップごとに Update を呼び、力を車体に加える。
// タイヤの力（駆動、ブレーキ、横のグリップ）は段階 8 で足す。
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

    // 状態
    bool grounded = false;
    float compression = 0;  // 自然長からの縮み（m）
    float length = 0.30f;   // 取付点から車輪の中心まで（m）。描画に使う
    float load = 0;         // 路面に掛けている力（N）
    float lastS = -1;       // 直前に当たった路面の周回位置
    float groundY = 0;      // 直前に当たった路面の高さ

    // レイの結果から縮みとバネの力を求める（スタビライザーの分は含まない）。
    void Update(const RaycastHit& hit, Vec3 mountWorld, float dt);
    // スタビライザーを含めた、車体を押し上げる力（N）。
    float Force();
    void Reset();

private:
    float springForce_ = 0;
    float prevCompression_ = 0;
    bool hasPrev_ = false;
};
