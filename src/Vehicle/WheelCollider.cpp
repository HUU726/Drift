#include "Vehicle/WheelCollider.h"

#include <algorithm>

void WheelCollider::Update(const RaycastHit& hit, Vec3 mountWorld, float dt) {
    // 取付点から路面までの距離。車輪の半径の分を引くと、サスペンションの長さになる。
    float reach = hit.hit ? mountWorld.y - hit.point.y - radius : restLength;
    if (hit.hit) {
        lastS = hit.s;
    }
    if (!hit.hit || reach >= restLength) {
        grounded = false;
        compression = 0;
        length = restLength;
        springForce_ = 0;
        load = 0;
        hasPrev_ = false;
        return;
    }
    grounded = true;
    groundY = hit.point.y;
    // 自然長より縮むことはない（車輪が取付点を越える）。越えた分は ColliderManager が底突きとして押し戻す。
    compression = std::min(restLength - reach, restLength);
    length = std::max(reach, 0.0f);

    float rate = hasPrev_ ? (compression - prevCompression_) / dt : 0.0f;
    prevCompression_ = compression;
    hasPrev_ = true;

    float spring = springRate * compression;
    if (compression > bumpStop) {
        // バンプストップのゴムは、縮むときに受け止めた力の一部しか返さない。着地で跳ね上がりすぎないため。
        spring += bumpStopRate * (compression - bumpStop) * (rate > 0 ? 1.0f : bumpStopReturn);
    }
    float damper = damping * rate * (rate > 0 ? bumpDampingScale : reboundDampingScale);
    // 路面を引っ張る向きの力は出さない。
    springForce_ = std::max(0.0f, spring + damper);
}

float WheelCollider::Force() {
    if (!grounded) {
        return 0;
    }
    float force = springForce_;
    if (antiRollPartner) {
        // 自分の側が多く縮んでいれば押し上げ、反対側を下げる（反対側では符号が逆になる）。
        force += antiRollRate * (compression - antiRollPartner->compression);
    }
    load = std::max(0.0f, force);
    return load;
}

void WheelCollider::Reset() {
    grounded = false;
    compression = 0;
    length = restLength;
    load = 0;
    lastS = -1;
    springForce_ = 0;
    prevCompression_ = 0;
    hasPrev_ = false;
}
