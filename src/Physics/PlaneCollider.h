#pragma once

#include "Physics/Collider.h"

#include <vector>

// 平らな地面。高さ height のどこまでも続く平面で、場所ごとに材質を変えられる。
// 材質は、長方形の区画と、折れ線に沿った帯（道）で決め、どちらにも入らない所は baseMaterial。
// 帯は区画より優先し、同じ種類では後から足したものほど優先する。試験コースで使う。
class PlaneCollider : public Collider {
public:
    struct Rect {
        float minX, minZ, maxX, maxZ;
        const PhysicsMaterial* material;
    };
    struct Strip {
        std::vector<Vec3> points;  // 折れ線（x, z を使う）
        float halfWidth;
        const PhysicsMaterial* material;
    };

    float height = 0;
    const PhysicsMaterial* baseMaterial = &Materials::kDirt;
    std::vector<Rect> rects;
    std::vector<Strip> strips;

    // (x, z) の真下の地面。yHint が地面より下（地面の下からのレイ）なら当たらない。
    RaycastHit GroundAt(float x, float z, float yHint) const;
    const PhysicsMaterial* MaterialAt(float x, float z) const;
};
