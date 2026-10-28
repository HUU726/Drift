#pragma once

#include "Physics/Collider.h"

#include <utility>
#include <vector>

class BoxCollider;
class TrackCollider;

namespace System {

// Collider の登録と、固定ステップごとの接触の検出・解決・通知。
// 物理の段階 4 では、車体の箱とコースの壁の組だけを扱う。積分はまだ Car が行う（段階 6 でここに移す）。
class ColliderManager {
public:
    bool Init();
    void UnInit();

    void Register(BoxCollider* box);
    void Register(TrackCollider* track);
    void Unregister(Collider* collider);

    // FixedUpdate の直後に呼ぶ。接触を押し戻し、力積を加え、接触が始まった組を owner に通知する。
    void Step(float dt);

private:
    void resolve(Contact& contact);

    std::vector<BoxCollider*> boxes_;
    std::vector<TrackCollider*> tracks_;
    std::vector<std::pair<Collider*, Collider*>> touching_;  // 前のステップで接触していた組
};

}  // namespace System
