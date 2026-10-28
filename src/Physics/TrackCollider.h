#pragma once

#include "Physics/Collider.h"
#include "track.hpp"

#include <unordered_map>

class BoxCollider;

// コースの路面と壁。Track::queryLocal を包む。
// 路面は真下へのレイで高さを返し、壁は箱の 4 隅が壁の線を越えていれば接触を返す。
class TrackCollider : public Collider {
public:
    explicit TrackCollider(const Track& track);

    // (x, z) の路面の高さ。yHint は立体交差で上下を選ぶための今の高さ、preferS は直前の周回位置。
    // 縁石の外（芝）は、縁石の端と同じ高さの平らな面として扱う。
    RaycastHit GroundAt(float x, float z, float yHint, float preferS) const;

    // いちばん深くめり込んだ隅の接触を返す。接触していなければ false。
    bool CollideBox(BoxCollider& box, Contact& out);

private:
    const Track& track_;
    // 箱ごとの直前の周回位置。立体交差で上下を取り違えないように query に渡す。
    std::unordered_map<const BoxCollider*, float> lastS_;
};
