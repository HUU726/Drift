#include "TestCourse.h"

#include <cmath>

namespace {

constexpr float kPi = 3.14159265f;

// S 字の道の中心線。center の x を中心に、z0 から z1 まで蛇行する。
std::vector<Vec3> sCurve(float centerX, float z0, float z1) {
    const float amplitude = 30.0f;
    const float wavelength = 160.0f;
    std::vector<Vec3> points;
    for (float z = z0; z <= z1 + 0.01f; z += 4.0f) {
        float x = centerX + amplitude * std::sin(2.0f * kPi * (z - z0) / wavelength);
        points.push_back(Vec3{x, 0, z});
    }
    return points;
}

// 高さ y の長方形の面。
void addRect(CpuMesh& mesh, float minX, float minZ, float maxX, float maxZ, float y, Vec3 color) {
    mesh.addQuad({minX, y, minZ}, {maxX, y, minZ}, {maxX, y, maxZ}, {minX, y, maxZ}, {0, 1, 0}, color);
}

// 折れ線に沿った幅 2 × halfWidth の帯。継ぎ目で隙間ができないよう、各点で前後の向きを平均した横向きを使う。
void addStrip(CpuMesh& mesh, const std::vector<Vec3>& points, float halfWidth, float y, Vec3 color) {
    const size_t n = points.size();
    std::vector<Vec3> left(n), right(n);
    for (size_t i = 0; i < n; ++i) {
        Vec3 prev = points[i == 0 ? 0 : i - 1];
        Vec3 next = points[i + 1 < n ? i + 1 : n - 1];
        Vec3 dir = normalize(Vec3{next.x - prev.x, 0, next.z - prev.z});
        Vec3 side{dir.z, 0, -dir.x};
        left[i] = Vec3{points[i].x - side.x * halfWidth, y, points[i].z - side.z * halfWidth};
        right[i] = Vec3{points[i].x + side.x * halfWidth, y, points[i].z + side.z * halfWidth};
    }
    for (size_t i = 0; i + 1 < n; ++i) {
        mesh.addQuad(left[i], right[i], right[i + 1], left[i + 1], {0, 1, 0}, color);
    }
}

}  // namespace

void TestCourse::Build() {
    ground = PlaneCollider{};
    ground.height = 0;
    ground.baseMaterial = &Materials::kRunoff;
    ground.rects.push_back({-60, -60, 60, 260, &Materials::kAsphalt});
    ground.rects.push_back({70, -60, 190, 260, &Materials::kDirt});
    ground.rects.push_back({-150, -60, -80, 400, &Materials::kAsphalt});
    ground.strips.push_back({sCurve(0, 280, 880), 6.0f, &Materials::kAsphalt});
    ground.strips.push_back({sCurve(130, 280, 880), 6.0f, &Materials::kDirt});
    // 広場から S 字の入口までつなぐ道
    ground.strips.push_back({{Vec3{0, 0, 250}, Vec3{0, 0, 290}}, 6.0f, &Materials::kAsphalt});
    ground.strips.push_back({{Vec3{130, 0, 250}, Vec3{130, 0, 290}}, 6.0f, &Materials::kDirt});

    ramps.clear();
    for (const RampPlace& place : kRamps) {
        RampSpec spec;
        spec.length = kRampLength;
        spec.width = kRampWidth;
        spec.height = kRampLength * std::tan(place.angleDeg * kPi / 180.0f);
        ramps.push_back(std::make_unique<RampCollider>(Vec3{place.x, ground.height, kRampZ}, 0.0f, spec));
    }
}

void TestCourse::BuildMesh(CpuMesh& mesh) const {
    const Vec3 grass{0.17f, 0.38f, 0.16f};
    const Vec3 asphalt{0.2f, 0.2f, 0.22f};
    const Vec3 dirt{0.48f, 0.36f, 0.22f};
    const Vec3 line{0.82f, 0.82f, 0.8f};
    const float y = ground.height;
    // 重なる面は少しずつ浮かせる。遠くでも奥行きの精度で面がちらつかない高さ（地面 → 区画 → 道 → 線）。

    addRect(mesh, -700, -500, 700, 1200, y, grass);
    for (const PlaneCollider::Rect& r : ground.rects) {
        addRect(mesh, r.minX, r.minZ, r.maxX, r.maxZ, y + 0.04f, r.material == &Materials::kDirt ? dirt : asphalt);
    }
    for (const PlaneCollider::Strip& s : ground.strips) {
        addStrip(mesh, s.points, s.halfWidth, y + 0.08f, s.material == &Materials::kDirt ? dirt : asphalt);
    }
    // 広場の目印。20 m ごとの横線（速度と距離の目安）と、中心の縦線。
    for (float z = -60; z <= 260; z += 20) {
        addRect(mesh, -60, z - 0.15f, 60, z + 0.15f, y + 0.12f, line);
        addRect(mesh, 70, z - 0.15f, 190, z + 0.15f, y + 0.12f, line);
    }
    addRect(mesh, -0.15f, -60, 0.15f, 260, y + 0.12f, line);
    addRect(mesh, 129.85f, -60, 130.15f, 260, y + 0.12f, line);
    // 滑走路の各レーンの中心線
    for (const RampPlace& place : kRamps) {
        addRect(mesh, place.x - 0.15f, -60, place.x + 0.15f, kRampZ, y + 0.12f, line);
    }
    for (const auto& ramp : ramps) {
        ramp->BuildMesh(mesh);
    }
}
