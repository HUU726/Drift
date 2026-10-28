#pragma once

#include "Physics/Collider.h"

#include <utility>
#include <vector>

class BoxCollider;
class RampCollider;
class TrackCollider;
class WheelCollider;
struct Rigidbody;

namespace System {

// Rigidbody と Collider の登録と、固定ステップごとの物理。
// サブステップごとにサスペンションとタイヤの力を求めて Rigidbody を積分し、
// その後で車体の箱とコースの壁、ジャンプ台の接触を解決する。
class ColliderManager {
public:
    static constexpr int kSubsteps = 4;
    static constexpr float kGravity = 9.81f;

    bool Init();
    void UnInit();

    void Register(Rigidbody* body);
    void Register(WheelCollider* wheel);
    void Register(BoxCollider* box);
    void Register(TrackCollider* track);
    void Register(RampCollider* ramp);
    void Unregister(Rigidbody* body);
    void Unregister(Collider* collider);

    // FixedUpdate の直後に呼ぶ。接触が始まった組は owner に通知する。
    void Step(float dt);

    // (x, z) の真下の路面。コースとジャンプ台のうち、yHint より下でいちばん高い面を返す。
    RaycastHit GroundAt(float x, float z, float yHint, float preferS) const;

private:
    void stepWheels(float dt);
    void resolve(Contact& contact);

    std::vector<Rigidbody*> bodies_;
    std::vector<WheelCollider*> wheels_;
    std::vector<BoxCollider*> boxes_;
    std::vector<TrackCollider*> tracks_;
    std::vector<RampCollider*> ramps_;
    std::vector<std::pair<Collider*, Collider*>> touching_;  // 前のステップで接触していた組
};

}  // namespace System
