#include "System/ColliderManager.h"

#include "GameObject/GameObject.h"
#include "Physics/BoxCollider.h"
#include "Physics/RampCollider.h"
#include "Physics/Rigidbody.h"
#include "Physics/TrackCollider.h"
#include "Vehicle/WheelCollider.h"

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

// 壁の衝突は水平面で解く。上下の速度と pitch / roll の回転は含めない。
Vec3 planarVelocityAt(const Rigidbody& body, Vec3 point) {
    Vec3 r = point - body.position;
    return Vec3{body.velocity.x + body.yawRate * r.z, 0, body.velocity.z - body.yawRate * r.x};
}

}  // namespace

bool ColliderManager::Init() {
    return true;
}

void ColliderManager::UnInit() {
    bodies_.clear();
    wheels_.clear();
    boxes_.clear();
    tracks_.clear();
    ramps_.clear();
    touching_.clear();
}

void ColliderManager::Register(Rigidbody* body) {
    bodies_.push_back(body);
}

void ColliderManager::Register(WheelCollider* wheel) {
    wheels_.push_back(wheel);
}

void ColliderManager::Register(BoxCollider* box) {
    boxes_.push_back(box);
}

void ColliderManager::Register(TrackCollider* track) {
    tracks_.push_back(track);
}

void ColliderManager::Register(RampCollider* ramp) {
    ramps_.push_back(ramp);
}

void ColliderManager::Unregister(Rigidbody* body) {
    bodies_.erase(std::remove(bodies_.begin(), bodies_.end(), body), bodies_.end());
}

void ColliderManager::Unregister(Collider* collider) {
    wheels_.erase(std::remove(wheels_.begin(), wheels_.end(), collider), wheels_.end());
    boxes_.erase(std::remove(boxes_.begin(), boxes_.end(), collider), boxes_.end());
    tracks_.erase(std::remove(tracks_.begin(), tracks_.end(), collider), tracks_.end());
    ramps_.erase(std::remove(ramps_.begin(), ramps_.end(), collider), ramps_.end());
    touching_.erase(std::remove_if(touching_.begin(), touching_.end(),
                                   [&](const auto& pair) { return pair.first == collider || pair.second == collider; }),
                    touching_.end());
}

void ColliderManager::Step(float dt) {
    float h = dt / kSubsteps;
    for (int i = 0; i < kSubsteps; ++i) {
        stepWheels(h);
    }

    std::vector<std::pair<Collider*, Collider*>> touchingNow;
    for (BoxCollider* box : boxes_) {
        if (!box->body) {
            continue;
        }
        auto handle = [&](Contact& contact) {
            resolve(contact);
            std::pair<Collider*, Collider*> pair(contact.self, contact.other);
            touchingNow.push_back(pair);
            bool began = std::find(touching_.begin(), touching_.end(), pair) == touching_.end();
            if (began && box->owner) {
                box->owner->OnCollision(contact);
            }
        };
        for (TrackCollider* track : tracks_) {
            Contact contact;
            if (track->CollideBox(*box, contact)) {
                handle(contact);
            }
        }
        for (RampCollider* ramp : ramps_) {
            Contact contact;
            if (ramp->CollideBox(*box, contact)) {
                handle(contact);
            }
        }
    }
    touching_ = std::move(touchingNow);
}

RaycastHit ColliderManager::GroundAt(float x, float z, float yHint, float preferS) const {
    // コースは立体交差の上下を yHint と preferS で選んで 1 つ返す。ジャンプ台はその上に載る面で、
    // レイの始点（yHint）より下にあればコースより高いほうを使う。
    RaycastHit best;
    for (const TrackCollider* track : tracks_) {
        RaycastHit hit = track->GroundAt(x, z, yHint, preferS);
        if (hit.hit && (!best.hit || std::fabs(hit.point.y - yHint) < std::fabs(best.point.y - yHint))) {
            best = hit;
        }
    }
    for (const RampCollider* ramp : ramps_) {
        RaycastHit hit = ramp->GroundAt(x, z, yHint);
        if (hit.hit && (!best.hit || hit.point.y > best.point.y)) {
            best = hit;
        }
    }
    return best;
}

void ColliderManager::stepWheels(float dt) {
    // 先に全輪の縮みを求めてから力を加える。スタビライザーが反対側の縮みを読むため。
    for (WheelCollider* wheel : wheels_) {
        Vec3 mountWorld = wheel->body->ToWorld(wheel->mount);
        RaycastHit hit = GroundAt(mountWorld.x, mountWorld.z, mountWorld.y, wheel->lastS);
        wheel->Update(hit, mountWorld, dt);
    }
    for (WheelCollider* wheel : wheels_) {
        // サスペンションの力は取付点に、タイヤの力は接地点に加える。接地点は重心より低いので、
        // 加速で後ろが沈み、曲がると外へ傾く（荷重移動）。
        float load = wheel->Force();
        if (load > 0) {
            wheel->body->AddForceAt(Vec3{0, load, 0}, wheel->body->ToWorld(wheel->mount));
        }
        Vec3 tireForce, contact;
        wheel->StepTire(load, dt, kGravity, tireForce, contact);
        wheel->body->AddForceAt(tireForce, contact);
    }
    for (Rigidbody* body : bodies_) {
        body->Integrate(dt, kGravity);
    }
    // 底突き: 縮みきって車輪が取付点まで来た輪では、その点の下向きの速度を力積で止め、めり込んだ分を押し戻す。
    for (WheelCollider* wheel : wheels_) {
        if (!wheel->grounded) {
            continue;
        }
        Rigidbody& body = *wheel->body;
        Vec3 mountWorld = body.ToWorld(wheel->mount);
        float penetration = wheel->groundY + wheel->radius - mountWorld.y;
        if (penetration <= 0) {
            continue;
        }
        body.position.y += penetration;
        float down = body.VelocityAt(mountWorld).y;
        if (down < 0) {
            body.AddVerticalImpulseAt(-down / body.VerticalInverseMassAt(mountWorld), mountWorld);
        }
    }
}

void ColliderManager::resolve(Contact& contact) {
    Rigidbody& body = *contact.self->body;
    const Vec3 n = contact.normal;
    const Vec3 r = contact.point - body.position;
    const Vec3 v = planarVelocityAt(body, contact.point);

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
    Vec3 vt = planarVelocityAt(body, contact.point);
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
