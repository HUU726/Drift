#include "Physics/PlaneCollider.h"

#include <algorithm>

namespace {

// 点 (x, z) から線分 a-b までの水平の距離の 2 乗。
float distanceSquared(float x, float z, Vec3 a, Vec3 b) {
    float abx = b.x - a.x;
    float abz = b.z - a.z;
    float len2 = abx * abx + abz * abz;
    float t = len2 > 1e-6f ? clampf(((x - a.x) * abx + (z - a.z) * abz) / len2, 0.0f, 1.0f) : 0.0f;
    float dx = x - (a.x + abx * t);
    float dz = z - (a.z + abz * t);
    return dx * dx + dz * dz;
}

}  // namespace

const PhysicsMaterial* PlaneCollider::MaterialAt(float x, float z) const {
    const PhysicsMaterial* found = baseMaterial;
    for (const Rect& r : rects) {
        if (x >= r.minX && x <= r.maxX && z >= r.minZ && z <= r.maxZ) {
            found = r.material;
        }
    }
    for (const Strip& s : strips) {
        float limit = s.halfWidth * s.halfWidth;
        for (size_t i = 0; i + 1 < s.points.size(); ++i) {
            if (distanceSquared(x, z, s.points[i], s.points[i + 1]) <= limit) {
                found = s.material;
                break;
            }
        }
    }
    return found;
}

RaycastHit PlaneCollider::GroundAt(float x, float z, float yHint) const {
    RaycastHit hit;
    if (yHint < height) {
        return hit;
    }
    hit.hit = true;
    hit.point = Vec3{x, height, z};
    hit.material = MaterialAt(x, z);
    return hit;
}
