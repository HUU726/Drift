#pragma once

#include "Physics/Collider.h"
#include "track.hpp"

#include <unordered_map>

class BoxCollider;

// コースの壁。Track::query を包み、箱の 4 隅が壁の線を越えていれば接触を返す。
// 路面（Raycast / GroundAt）と材質の判定は、使う段階（段階 5、6）でここに移す。
class TrackCollider : public Collider {
public:
    explicit TrackCollider(const Track& track);

    // いちばん深くめり込んだ隅の接触を返す。接触していなければ false。
    bool CollideBox(BoxCollider& box, Contact& out);

private:
    const Track& track_;
    // 箱ごとの直前の周回位置。立体交差で上下を取り違えないように query に渡す。
    std::unordered_map<const BoxCollider*, float> lastS_;
};
