#pragma once

#include "Physics/PhysicsMaterial.h"
#include "geom.hpp"

class GameObject;
struct Rigidbody;
class Collider;

// 2 つの Collider の接触。normal は other から self を押し返す向き。
struct Contact {
    Collider* self = nullptr;
    Collider* other = nullptr;
    Vec3 point;
    Vec3 normal;
    float depth = 0;    // めり込み（m）
    float impulse = 0;  // 法線方向の力積（N·s）。当たりの強さ
};

// 形状と材質を持つ当たり判定。GameObject がメンバとして持ち、ColliderManager に登録する。
class Collider {
public:
    virtual ~Collider() = default;

    GameObject* owner = nullptr;                  // 接触の通知先。なければ通知しない
    Rigidbody* body = nullptr;                    // 動かないものは null
    const PhysicsMaterial* material = nullptr;    // null なら相手の材質だけを使う
};
