#pragma once

#include "Physics/PlaneCollider.h"
#include "Physics/RampCollider.h"
#include "meshbuild.hpp"

#include <memory>
#include <vector>

// 走りを試すための試験コース。GL を使わないので、TestScene と self-test の両方が同じ配置を使う。
// 地面は高さ 0 の平面で、芝の上に次の区画を置く（z が前、x が右）。
//   アスファルトの広場   x -60〜60、   z -60〜260   加速、ブレーキ、定常円、ドリフト
//   ダートの広場         x 70〜190、   z -60〜260   同じことをダートで
//   ジャンプ台の滑走路   x -150〜-80、 z -60〜400   z = 200 に 5° / 10° / 15° / 20° の台を横に並べる
//   S 字（アスファルト） x = 0 を中心に、z 280〜880 を振幅 30 m・波長 160 m で蛇行する幅 12 m の道
//   S 字（ダート）       同じ形を x = 130 に
struct TestCourse {
    // ジャンプ台 1 台分の置き方
    struct RampPlace {
        float angleDeg;
        float x;  // 中心の x。手前の端は z = kRampZ
    };
    static constexpr float kRampZ = 200.0f;
    static constexpr float kRampLength = 10.0f;
    static constexpr float kRampWidth = 8.0f;
    static constexpr RampPlace kRamps[] = {{5, -140}, {10, -125}, {15, -110}, {20, -95}};

    PlaneCollider ground;
    std::vector<std::unique_ptr<RampCollider>> ramps;
    Vec3 spawn{0, 0, -40};  // 車の原点を置く位置（アスファルトの広場の手前）
    float spawnYaw = 0;     // +z 向き

    // 区画、道、ジャンプ台を作る。
    void Build();
    // 地面、道、ジャンプ台、目印の線をメッシュに足す。
    void BuildMesh(CpuMesh& mesh) const;
};
