#pragma once

#include "Physics/Collider.h"

#include <utility>
#include <vector>

class BoxCollider;
class TrackCollider;
class WheelCollider;
struct Rigidbody;

namespace System {

// Rigidbody と Collider の登録と、固定ステップごとの物理。
// 段階 6 では、サブステップごとにサスペンションの力を求めて上下の動きと pitch / roll を積分し、
// その後で車体の箱とコースの壁の接触を解決する。水平の動きはまだ Car が進める（段階 8 でここに移す）。
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
    void Unregister(Rigidbody* body);
    void Unregister(Collider* collider);

    // FixedUpdate の直後に呼ぶ。接触が始まった組は owner に通知する。
    void Step(float dt);

    // (x, z) の真下の路面。登録されたコースのうち、yHint に合う路面を返す。
    RaycastHit GroundAt(float x, float z, float yHint, float preferS) const;

private:
    void stepSuspension(float dt);
    void resolve(Contact& contact);

    std::vector<Rigidbody*> bodies_;
    std::vector<WheelCollider*> wheels_;
    std::vector<BoxCollider*> boxes_;
    std::vector<TrackCollider*> tracks_;
    std::vector<std::pair<Collider*, Collider*>> touching_;  // 前のステップで接触していた組
};

}  // namespace System
