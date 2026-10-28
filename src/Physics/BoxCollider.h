#pragma once

#include "Physics/Collider.h"
#include "Physics/Rigidbody.h"

// 車体の向き付きの箱。物理の段階 4 では水平面の 4 隅だけを使い、壁との接触を調べる。
class BoxCollider : public Collider {
public:
    Vec3 halfExtents{0.95f, 0.4f, 2.275f};  // 車体の外形（幅、高さ、長さの半分）
    Vec3 offset{0, 0.21f, 0.075f};         // 重心（body.position）から箱の中心まで（車体の座標）

    // 水平面での 4 隅（ワールド座標）。前右、前左、後右、後左の順。
    void Corners(Vec3 out[4]) const {
        Vec3 forward = body->Forward();
        Vec3 right = body->Right();
        Vec3 center = body->position + forward * offset.z + right * offset.x;
        const float sx[4] = {1, -1, 1, -1};
        const float sz[4] = {1, 1, -1, -1};
        for (int i = 0; i < 4; ++i) {
            out[i] = center + right * (sx[i] * halfExtents.x) + forward * (sz[i] * halfExtents.z);
        }
    }
};
