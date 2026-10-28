#pragma once

#include "Physics/Collider.h"
#include "meshbuild.hpp"
#include "track.hpp"

#include <string>
#include <vector>

class BoxCollider;

// ジャンプ台 1 台分の置き方。ramps.txt の 1 行。
struct RampSpec {
    float s = 0;         // 周回位置（m）。台の手前の端
    float lateral = 0;   // コースの中心からの横位置（m、右が正）。台の中心
    float length = 10;   // m
    float width = 4;     // m
    float height = 1;    // 奥の端の高さ（m）
};

// ジャンプ台。コースに沿った向きの傾いた箱で、手前の端の高さ 0 から奥の端の height まで上がる。
// 上面は路面として車輪のレイを受け、側面と奥の面は壁として車体と当たる。
class RampCollider : public Collider {
public:
    RampCollider(const Track& track, const RampSpec& spec);

    // (x, z) の真下の上面。台の外、または上面が yHint より上（台の下からのレイ）なら当たらない。
    RaycastHit GroundAt(float x, float z, float yHint) const;
    // 車体の箱の隅が、乗り上げられない高さで台の中に入っていれば、側面か奥の面から押し出す接触を返す。
    bool CollideBox(BoxCollider& box, Contact& out) const;
    // 台の形をワールド座標でメッシュに足す。
    void BuildMesh(CpuMesh& mesh) const;

    // ramps.txt を読む。読めなければ空。
    static std::vector<RampSpec> Load(const std::string& path);

private:
    // 台の座標（u: 手前の端から奥へ、v: 中心から右へ）にする。
    void toLocal(float x, float z, float& u, float& v) const;
    float surfaceAt(float u) const;
    Vec3 pointAt(float u, float v, float y) const;

    RampSpec spec_;
    Vec3 origin_;   // 手前の端の中心（水平の位置）
    Vec3 forward_;  // コースの向き（水平）
    Vec3 right_;
    float y0_ = 0;  // 手前の端の路面の高さ
    float y1_ = 0;  // 奥の端の路面の高さ
};
