#include "Physics/RampCollider.h"

#include "Physics/BoxCollider.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace {

// これより低い段差は乗り上げられる（車体の下と上面の差、m）。高い所では台は壁になる。
constexpr float kStepHeight = 0.25f;

// 周回位置 s のコースの中心（point.y は中心線の高さ）と、コースの向き（水平）。
void centerAt(const Track& track, float s, Vec3& point, Vec3& tangent) {
    const int n = static_cast<int>(track.samples.size());
    s = std::fmod(s, track.length);
    if (s < 0) {
        s += track.length;
    }
    int i = 0;
    while (i + 1 < n && track.samples[i + 1].s <= s) {
        ++i;
    }
    const TrackSample& a = track.samples[i];
    const TrackSample& b = track.samples[(i + 1) % n];
    Vec3 d{b.x - a.x, 0, b.z - a.z};
    float len = length(d);
    float t = len > 1e-4f ? clampf((s - a.s) / len, 0.0f, 1.0f) : 0.0f;
    point = Vec3{a.x + d.x * t, a.y + (b.y - a.y) * t, a.z + d.z * t};
    tangent = len > 1e-4f ? d * (1.0f / len) : Vec3{0, 0, 1};
}

// (x, z) の路面の高さ。yHint は中心線の高さ（立体交差で上下を選ぶため）。
float roadHeight(const Track& track, float x, float z, float yHint, float s) {
    TrackPose pose = track.query(x, z, yHint, s);
    float bankLimit = track.halfWidth + track.curbWidth;
    return pose.centerY + clampf(pose.lateral, -bankLimit, bankLimit) * pose.cross;
}

}  // namespace

RampCollider::RampCollider(const Track& track, const RampSpec& spec) : spec_(spec) {
    material = &Materials::kRamp;
    Vec3 center;
    centerAt(track, spec.s, center, forward_);
    right_ = Vec3{forward_.z, 0, -forward_.x};
    origin_ = Vec3{center.x, 0, center.z} + right_ * spec.lateral;
    Vec3 end = origin_ + forward_ * spec.length;
    y0_ = roadHeight(track, origin_.x, origin_.z, center.y, spec.s);
    y1_ = roadHeight(track, end.x, end.z, center.y, spec.s + spec.length);
}

void RampCollider::toLocal(float x, float z, float& u, float& v) const {
    Vec3 d{x - origin_.x, 0, z - origin_.z};
    u = dot(d, forward_);
    v = dot(d, right_);
}

float RampCollider::surfaceAt(float u) const {
    float t = u / spec_.length;
    return y0_ + (y1_ - y0_) * t + spec_.height * t;
}

Vec3 RampCollider::pointAt(float u, float v, float y) const {
    Vec3 p = origin_ + forward_ * u + right_ * v;
    return Vec3{p.x, y, p.z};
}

RaycastHit RampCollider::GroundAt(float x, float z, float yHint) const {
    RaycastHit hit;
    float u, v;
    toLocal(x, z, u, v);
    if (u < 0 || u > spec_.length || std::fabs(v) > spec_.width * 0.5f) {
        return hit;
    }
    float y = surfaceAt(u);
    if (y > yHint) {
        return hit;
    }
    hit.hit = true;
    hit.point = Vec3{x, y, z};
    hit.s = spec_.s + u;
    hit.material = material;
    return hit;
}

bool RampCollider::CollideBox(BoxCollider& box, Contact& out) const {
    const Rigidbody& body = *box.body;
    float bottom = body.position.y + box.offset.y - box.halfExtents.y;
    Vec3 corners[4];
    box.Corners(corners);
    float deepest = 0;
    for (const Vec3& corner : corners) {
        float u, v;
        toLocal(corner.x, corner.z, u, v);
        float halfWidth = spec_.width * 0.5f;
        if (u < 0 || u > spec_.length || std::fabs(v) > halfWidth) {
            continue;
        }
        if (bottom > surfaceAt(u) - kStepHeight) {
            continue;  // 上面に乗り上げられる高さ
        }
        // 側面と奥の面のうち、近いほうから押し出す。手前の端は高さ 0 なので壁にならない。
        float side = halfWidth - std::fabs(v);
        float back = spec_.length - u;
        float depth = std::min(side, back);
        if (depth <= deepest) {
            continue;
        }
        deepest = depth;
        out.point = corner;
        out.normal = side < back ? right_ * (v > 0 ? 1.0f : -1.0f) : forward_;
        out.depth = depth;
    }
    if (deepest <= 0) {
        return false;
    }
    out.self = &box;
    out.other = const_cast<RampCollider*>(this);
    return true;
}

void RampCollider::BuildMesh(CpuMesh& mesh) const {
    const float w = spec_.width * 0.5f;
    const float lift = 0.02f;  // 路面と重ならないように少し浮かせる
    const float top = surfaceAt(spec_.length);
    Vec3 fl = pointAt(0, -w, y0_ + lift);
    Vec3 fr = pointAt(0, w, y0_ + lift);
    Vec3 bl = pointAt(spec_.length, -w, top + lift);
    Vec3 br = pointAt(spec_.length, w, top + lift);
    Vec3 bl0 = pointAt(spec_.length, -w, y1_);
    Vec3 br0 = pointAt(spec_.length, w, y1_);
    const Vec3 face{0.88f, 0.62f, 0.12f};
    const Vec3 sideColor{0.32f, 0.3f, 0.28f};
    Vec3 up = normalize(cross(fr - fl, bl - fl));
    if (up.y < 0) {
        up = up * -1.0f;
    }
    mesh.addQuad(fl, fr, br, bl, up, face);
    mesh.addTriangle(fl, bl, bl0, right_ * -1.0f, sideColor);
    mesh.addTriangle(fr, br0, br, right_, sideColor);
    mesh.addQuad(bl0, br0, br, bl, forward_, sideColor);
}

std::vector<RampSpec> RampCollider::Load(const std::string& path) {
    std::vector<RampSpec> ramps;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        RampSpec spec;
        ss >> spec.s >> spec.lateral >> spec.length >> spec.width >> spec.height;
        if (!ss.fail() && spec.length > 0 && spec.width > 0) {
            ramps.push_back(spec);
        }
    }
    return ramps;
}
