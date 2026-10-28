#include "SelfTest.h"

#include "Physics/BoxCollider.h"
#include "Physics/TrackCollider.h"
#include "System/ColliderManager.h"
#include "car.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

constexpr float kStep = 1.0f / 60.0f;

// ゲームと同じ順（Car の更新 → 物理ステップ）で車を動かす。描画と音は使わない。
struct Rig {
    explicit Rig(const Track& track) : wall(track) {
        car.reset(track);
        box.body = &car.body;
        physics.Init();
        physics.Register(&wall);
        physics.Register(&box);
    }
    ~Rig() { physics.UnInit(); }

    void Step(const DriveInput& input, const Track& track) {
        car.update(kStep, input, track);
        physics.Step(kStep);
    }

    Car car;
    BoxCollider box;
    TrackCollider wall;
    System::ColliderManager physics;
};

// 車体の 4 隅のうち、壁の線をいちばん越えている量（m）。越えていなければ 0 以下。
float wallDepth(const Rig& rig, const Track& track) {
    const Vec3& center = rig.car.body.position;
    TrackPose pose = track.query(center.x, center.z, center.y, rig.car.ground.s);
    if (!pose.found) {
        return 0;
    }
    Vec3 corners[4];
    rig.box.Corners(corners);
    float deepest = -1.0e9f;
    for (const Vec3& corner : corners) {
        float lateral = pose.lateral + dot(corner - center, Vec3{pose.rightX, 0, pose.rightZ});
        deepest = std::max(deepest, std::fabs(lateral) - track.wallLateral());
    }
    return deepest;
}

// 自動運転でコースを 50 秒走る。
int lapTest(const Track& track) {
    Rig rig(track);
    Car& car = rig.car;
    float travelled = 0;
    float prevS = -1;
    float maxLat = 0;
    for (int step = 0; step < 60 * 50; ++step) {
        TrackPose pose = track.query(car.body.position.x, car.body.position.z, car.body.position.y, car.ground.found ? car.ground.s : -1.0f);
        float lookS = pose.s + 70.0f;
        if (lookS >= track.length) {
            lookS -= track.length;
        }
        int nearest = 0;
        float nearestGap = 1.0e9f;
        for (int i = 0; i < static_cast<int>(track.samples.size()); i += 2) {
            float gap = std::fabs(track.samples[i].s - lookS);
            if (gap > track.length * 0.5f) {
                gap = track.length - gap;
            }
            if (gap < nearestGap) {
                nearestGap = gap;
                nearest = i;
            }
        }
        int next = (nearest + 1) % static_cast<int>(track.samples.size());
        float tx = track.samples[next].x - track.samples[nearest].x;
        float tz = track.samples[next].z - track.samples[nearest].z;
        float targetYaw = std::atan2(tx, tz);
        float errNow = wrapPi(pose.yaw - car.body.yaw);
        float errAhead = wrapPi(targetYaw - car.body.yaw);
        float bend = std::fabs(wrapPi(targetYaw - pose.yaw));
        DriveInput input;
        input.steer = clampf(errNow * 2.6f + errAhead * 0.5f, -1.0f, 1.0f);
        input.steer = clampf(input.steer - pose.lateral * 0.12f, -1.0f, 1.0f);
        input.throttle = bend < 0.15f ? 1.0f : 0.45f;
        input.brake = 0;
        if (bend > 0.28f && car.ForwardSpeed() > 22.0f) {
            input.throttle = 0;
            input.brake = 1.0f;
        } else if (bend > 0.12f && car.ForwardSpeed() > 34.0f) {
            input.throttle = 0;
            input.brake = 0.55f;
        }
        if (car.ForwardSpeed() < 10.0f) {
            input.brake = 0;
            input.throttle = std::max(input.throttle, 0.5f);
        }
        rig.Step(input, track);
        pose = track.query(car.body.position.x, car.body.position.z, car.body.position.y, car.ground.found ? car.ground.s : -1.0f);
        if (prevS >= 0) {
            float ds = pose.s - prevS;
            if (ds < -track.length * 0.5f) ds += track.length;
            if (ds > track.length * 0.5f) ds -= track.length;
            if (ds > 0) travelled += ds;
        }
        prevS = pose.s;
        maxLat = std::max(maxLat, pose.xzDist);
        if (!std::isfinite(car.body.position.x) || !std::isfinite(car.ForwardSpeed()) || !std::isfinite(car.body.yaw)) {
            std::printf("self-test non-finite\n");
            return 1;
        }
    }
    std::printf("self-test length %.0f travelled %.0f maxLat %.1f speed %.1f km/h\n",
                track.length, travelled, maxLat, car.ForwardSpeed() * 3.6f);
    if (track.length < 5200.0f || track.length > 6500.0f) return 2;
    if (travelled < 1200.0f) return 3;
    if (maxLat > 30.0f) return 4;
    return 0;
}

// ハンドルを切ったまま全開で壁に当て続ける。車体が壁を抜けず、値が発散しないことを確かめる。
int wallTest(const Track& track) {
    Rig rig(track);
    float maxDepth = 0;
    float minSpeed = 0;
    for (int step = 0; step < 60 * 20; ++step) {
        DriveInput input;
        input.throttle = 1;
        input.steer = (step / 300) % 2 == 0 ? 1.0f : -1.0f;  // 5 秒ごとに左右の壁へ
        rig.Step(input, track);
        maxDepth = std::max(maxDepth, wallDepth(rig, track));
        minSpeed = std::min(minSpeed, rig.car.ForwardSpeed());
        const Rigidbody& body = rig.car.body;
        if (!std::isfinite(body.position.x) || !std::isfinite(body.velocity.x) || !std::isfinite(body.yawRate)) {
            std::printf("self-test wall non-finite\n");
            return 1;
        }
    }
    std::printf("self-test wall maxDepth %.3f m minSpeed %.1f km/h\n", maxDepth, minSpeed * 3.6f);
    if (maxDepth > 0.05f) return 5;
    return 0;
}

}  // namespace

int RunSelfTest(const Track& track) {
    int result = lapTest(track);
    if (result != 0) {
        return result;
    }
    return wallTest(track);
}
