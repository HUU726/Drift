#include "Physics/TrackCollider.h"

#include "Physics/BoxCollider.h"

#include <cmath>

TrackCollider::TrackCollider(const Track& track) : track_(track) {
    material = &Materials::kWall;
}

RaycastHit TrackCollider::GroundAt(float x, float z, float yHint, float preferS) const {
    RaycastHit hit;
    TrackPose pose = track_.queryLocal(x, z, yHint, preferS);
    if (!pose.found) {
        return hit;
    }
    float bankLimit = track_.halfWidth + track_.curbWidth;
    float bankLat = clampf(pose.lateral, -bankLimit, bankLimit);
    hit.hit = true;
    hit.point = Vec3{x, pose.centerY + bankLat * pose.cross, z};
    hit.s = pose.s;
    return hit;
}

bool TrackCollider::CollideBox(BoxCollider& box, Contact& out) {
    const Vec3& center = box.body->position;
    auto hint = lastS_.find(&box);
    float preferS = hint != lastS_.end() ? hint->second : -1.0f;
    TrackPose pose = track_.queryLocal(center.x, center.z, center.y, preferS);
    if (!pose.found) {
        return false;
    }
    lastS_[&box] = pose.s;

    // 箱の大きさ（数 m）ではコースの曲がりは無視できるので、隅の横位置は中心の横位置から直線で求める。
    const Vec3 right{pose.rightX, 0, pose.rightZ};
    const float wall = track_.wallLateral();
    Vec3 corners[4];
    box.Corners(corners);
    // 接触点は、めり込んだ隅をめり込みの深さで重み付けした平均。正面から当たったときに偏った回転が付かない。
    float deepest = 0;
    float weight = 0;
    Vec3 point;
    for (const Vec3& corner : corners) {
        float lateral = pose.lateral + dot(corner - center, right);
        float depth = std::fabs(lateral) - wall;
        if (depth <= 0) {
            continue;
        }
        point = point + corner * depth;
        weight += depth;
        if (depth > deepest) {
            deepest = depth;
            out.normal = right * (lateral > 0 ? -1.0f : 1.0f);
        }
    }
    if (deepest <= 0) {
        return false;
    }
    out.point = point * (1.0f / weight);
    out.depth = deepest;
    out.self = &box;
    out.other = this;
    return true;
}
