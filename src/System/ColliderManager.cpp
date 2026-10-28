#include "System/ColliderManager.h"

#include "GameObject/GameObject.h"
#include "Physics/BoxCollider.h"
#include "Physics/TrackCollider.h"

#include <algorithm>
#include <cmath>

namespace System {

namespace {

// 2 つの材質が接したときの摩擦は幾何平均、反発は大きいほう。片方がなければもう片方を使う。
void combine(const PhysicsMaterial* a, const PhysicsMaterial* b, float& friction, float& restitution) {
    if (!a) a = b;
    if (!b) b = a;
    friction = a ? std::sqrt(a->friction * b->friction) : 0.0f;
    restitution = a ? std::max(a->restitution, b->restitution) : 0.0f;
}

// 水平面での (r × v) の上下成分。
float crossY(Vec3 r, Vec3 v) {
    return r.z * v.x - r.x * v.z;
}

}  // namespace

bool ColliderManager::Init() {
    return true;
}

void ColliderManager::UnInit() {
    boxes_.clear();
    tracks_.clear();
    touching_.clear();
}

void ColliderManager::Register(BoxCollider* box) {
    boxes_.push_back(box);
}

void ColliderManager::Register(TrackCollider* track) {
    tracks_.push_back(track);
}

void ColliderManager::Unregister(Collider* collider) {
    boxes_.erase(std::remove(boxes_.begin(), boxes_.end(), collider), boxes_.end());
    tracks_.erase(std::remove(tracks_.begin(), tracks_.end(), collider), tracks_.end());
    touching_.erase(std::remove_if(touching_.begin(), touching_.end(),
                                   [&](const auto& pair) { return pair.first == collider || pair.second == collider; }),
                    touching_.end());
}

void ColliderManager::Step(float dt) {
    (void)dt;
    std::vector<std::pair<Collider*, Collider*>> touchingNow;
    for (BoxCollider* box : boxes_) {
        if (!box->body) {
            continue;
        }
        for (TrackCollider* track : tracks_) {
            Contact contact;
            if (!track->CollideBox(*box, contact)) {
                continue;
            }
            resolve(contact);
            std::pair<Collider*, Collider*> pair(box, track);
            touchingNow.push_back(pair);
            bool began = std::find(touching_.begin(), touching_.end(), pair) == touching_.end();
            if (began && box->owner) {
                box->owner->OnCollision(contact);
            }
        }
    }
    touching_ = std::move(touchingNow);
}

void ColliderManager::resolve(Contact& contact) {
    Rigidbody& body = *contact.self->body;
    const Vec3 n = contact.normal;
    const Vec3 r = contact.point - body.position;
    const Vec3 v = body.VelocityAt(contact.point);

    // めり込んだ分だけ押し戻す。接触点も一緒に動くので、r は変わらない。
    body.position = body.position + n * contact.depth;
    contact.point = contact.point + n * contact.depth;

    float vn = dot(v, n);
    if (vn >= 0) {
        return;  // すでに離れる向きに動いている
    }
    float friction = 0;
    float restitution = 0;
    combine(contact.self->material, contact.other->material, friction, restitution);

    // 法線方向: 反発係数の分だけ跳ね返す。
    float rn = crossY(r, n);
    float jn = -(1.0f + restitution) * vn / (1.0f / body.mass + rn * rn / body.inertiaYaw);
    body.AddImpulseAt(n * jn, contact.point);
    contact.impulse = jn;

    // 接線方向: 滑りを止める向きに、摩擦の上限（μ × 法線の力積）まで。
    Vec3 vt = body.VelocityAt(contact.point);
    vt = vt - n * dot(vt, n);
    float speedT = length(vt);
    if (speedT < 1e-4f) {
        return;
    }
    Vec3 t = vt * (1.0f / speedT);
    float rt = crossY(r, t);
    float jt = -speedT / (1.0f / body.mass + rt * rt / body.inertiaYaw);
    jt = std::max(jt, -friction * jn);
    body.AddImpulseAt(t * jt, contact.point);
}

}  // namespace System
