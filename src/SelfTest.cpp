#include "SelfTest.h"

#include "Physics/BoxCollider.h"
#include "Physics/TrackCollider.h"
#include "System/ColliderManager.h"
#include "Vehicle/VehicleSpec.h"
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
        physics.Register(&car.body);
        for (WheelCollider& w : car.wheels) {
            physics.Register(&w);
        }
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
    TrackPose pose = track.query(center.x, center.z, rig.car.Origin().y, rig.car.ground.s);
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
    float air = 0;        // 4 輪とも浮いている時間
    float maxAir = 0;     // その最長（跳ねていないか）
    float maxCompression = 0;  // 縮みの最大（底突きしていないか）
    for (int step = 0; step < 60 * 50; ++step) {
        TrackPose pose = track.query(car.body.position.x, car.body.position.z, car.Origin().y, car.ground.found ? car.ground.s : -1.0f);
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
        pose = track.query(car.body.position.x, car.body.position.z, car.Origin().y, car.ground.found ? car.ground.s : -1.0f);
        if (prevS >= 0) {
            float ds = pose.s - prevS;
            if (ds < -track.length * 0.5f) ds += track.length;
            if (ds > track.length * 0.5f) ds -= track.length;
            if (ds > 0) travelled += ds;
        }
        prevS = pose.s;
        maxLat = std::max(maxLat, pose.xzDist);
        bool airborne = true;
        for (const WheelCollider& w : car.wheels) {
            airborne = airborne && !w.grounded;
            maxCompression = std::max(maxCompression, w.compression);
        }
        air = airborne ? air + kStep : 0.0f;
        maxAir = std::max(maxAir, air);
        if (!std::isfinite(car.body.position.x) || !std::isfinite(car.body.position.y) || !std::isfinite(car.ForwardSpeed()) ||
            !std::isfinite(car.body.yaw) || !std::isfinite(car.body.pitch) || !std::isfinite(car.body.roll)) {
            std::printf("self-test non-finite\n");
            return 1;
        }
    }
    std::printf("self-test length %.0f travelled %.0f maxLat %.1f speed %.1f km/h airtime %.2f s compression %.3f m\n",
                track.length, travelled, maxLat, car.ForwardSpeed() * 3.6f, maxAir, maxCompression);
    if (track.length < 5200.0f || track.length > 6500.0f) return 2;
    if (travelled < 1200.0f) return 3;
    if (maxLat > 30.0f) return 4;
    if (maxAir > 0.2f || maxCompression > VehicleSpec::kBumpStop) return 8;
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

// 2 つの周回位置の差（m）。1 周をまたぐ分を考える。
float sGap(const Track& track, float a, float b) {
    float gap = std::fabs(a - b);
    return std::min(gap, track.length - gap);
}

// queryLocal が、全区間を調べる query と同じ区間を返すことを確かめる。
// コース上の全サンプル点で、横位置を左右の壁まで変えて比べる。立体交差では上下両方の路面で比べる。
int queryTest(const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    int checked = 0;
    int mismatchRoad = 0;    // 路面と縁石の上（|横位置| が halfWidth + curbWidth 以内）
    int mismatchRunoff = 0;  // 芝の上
    float maxRoadGap = 0;
    for (int i = 0; i < n; ++i) {
        const TrackSample& a = track.samples[i];
        const TrackSample& b = track.samples[(i + 1) % n];
        float tx = b.x - a.x;
        float tz = b.z - a.z;
        float len = std::sqrt(tx * tx + tz * tz);
        if (len < 1e-4f) {
            continue;
        }
        Vec3 right{tz / len, 0, -tx / len};
        for (float lateral = -15.0f; lateral <= 15.01f; lateral += 2.5f) {
            float x = a.x + right.x * lateral;
            float z = a.z + right.z * lateral;
            TrackPose full = track.query(x, z, a.y, a.s);
            TrackPose local = track.queryLocal(x, z, a.y, a.s);
            ++checked;
            float gap = sGap(track, full.s, local.s);
            if (gap > 0.01f) {
                if (std::fabs(lateral) <= track.halfWidth + track.curbWidth) {
                    ++mismatchRoad;
                    maxRoadGap = std::max(maxRoadGap, gap);
                } else {
                    ++mismatchRunoff;
                }
            }
        }
    }

    // 立体交差: 上の路面の点と、ほぼ同じ平面位置にある下の路面の点の組を探し、それぞれの高さで調べる。
    int crossings = 0;
    int wrongLevel = 0;
    for (int i = 0; i < n; ++i) {
        const TrackSample& upper = track.samples[i];
        if (!upper.bridge) {
            continue;
        }
        for (int j = 0; j < n; ++j) {
            const TrackSample& lower = track.samples[j];
            float dx = upper.x - lower.x;
            float dz = upper.z - lower.z;
            if (dx * dx + dz * dz > 4.0f * 4.0f || sGap(track, upper.s, lower.s) < 200.0f || upper.y - lower.y < 3.0f) {
                continue;
            }
            ++crossings;
            const TrackSample* levels[2] = {&upper, &lower};
            for (const TrackSample* level : levels) {
                TrackPose full = track.query(upper.x, upper.z, level->y, level->s);
                TrackPose local = track.queryLocal(upper.x, upper.z, level->y, level->s);
                if (sGap(track, full.s, level->s) > 10.0f || sGap(track, local.s, level->s) > 10.0f) {
                    ++wrongLevel;
                }
            }
        }
    }
    std::printf("self-test query checked %d mismatch road %d (max %.2f m) runoff %d, crossings %d wrongLevel %d\n",
                checked, mismatchRoad, maxRoadGap, mismatchRunoff, crossings, wrongLevel);
    if (mismatchRoad > 0) return 6;
    if (crossings == 0 || wrongLevel > 0) return 7;
    return 0;
}

// 原点の真下の路面の高さ。
float groundHeight(const Track& track, const Car& car) {
    const TrackPose& g = car.ground;
    float bankLimit = track.halfWidth + track.curbWidth;
    return g.centerY + clampf(g.lateral, -bankLimit, bankLimit) * g.cross;
}

// スポーン地点に置いた車が 1 秒以内に落ち着き、今の車高（原点が路面の高さ）に載ることを確かめる。
int settleTest(const Track& track) {
    Rig rig(track);
    DriveInput none;
    for (int step = 0; step < 60; ++step) {
        rig.Step(none, track);
    }
    const Car& car = rig.car;
    float height = car.Origin().y - groundHeight(track, car);
    float load = 0;
    for (const WheelCollider& w : car.wheels) {
        load += w.load;
    }
    std::printf("self-test settle vy %.4f m/s height %.4f m compression %.3f/%.3f/%.3f/%.3f m load %.0f N\n",
                car.body.velocity.y, height, car.wheels[0].compression, car.wheels[1].compression,
                car.wheels[2].compression, car.wheels[3].compression, load);
    if (std::fabs(car.body.velocity.y) >= 0.01f) return 9;
    if (std::fabs(height) > 0.01f) return 10;
    return 0;
}

// 立体交差の上と下にそれぞれ車を置き、どちらもその高さの路面に載ることを確かめる。
int crossingTest(const Track& track) {
    const int n = static_cast<int>(track.samples.size());
    int placed = 0;
    int wrong = 0;
    for (int i = 0; i < n; ++i) {
        const TrackSample& upper = track.samples[i];
        if (!upper.bridge) {
            continue;
        }
        for (int j = 0; j < n; ++j) {
            const TrackSample& lower = track.samples[j];
            float dx = upper.x - lower.x;
            float dz = upper.z - lower.z;
            if (dx * dx + dz * dz > 4.0f * 4.0f || sGap(track, upper.s, lower.s) < 200.0f || upper.y - lower.y < 3.0f) {
                continue;
            }
            const TrackSample* levels[2] = {&upper, &lower};
            for (const TrackSample* level : levels) {
                Rig rig(track);
                Car& car = rig.car;
                TrackPose pose = track.query(upper.x, upper.z, level->y, level->s);
                car.body.position = Vec3{upper.x, level->y + VehicleSpec::kCenterOfMassY, upper.z};
                car.body.yaw = pose.yaw;
                car.ground = pose;
                DriveInput none;
                for (int step = 0; step < 90; ++step) {
                    rig.Step(none, track);
                }
                ++placed;
                float height = car.Origin().y - level->y;
                if (std::fabs(height) > 0.2f || sGap(track, car.ground.s, level->s) > 10.0f) {
                    std::printf("self-test crossing wrong: level y %.1f height %.2f s %.0f -> %.0f\n",
                                level->y, height, level->s, car.ground.s);
                    ++wrong;
                }
            }
        }
    }
    std::printf("self-test crossing placed %d wrong %d\n", placed, wrong);
    if (placed == 0 || wrong > 0) return 11;
    return 0;
}

}  // namespace

int RunSelfTest(const Track& track) {
    int result = lapTest(track);
    if (result != 0) {
        return result;
    }
    result = wallTest(track);
    if (result != 0) {
        return result;
    }
    result = queryTest(track);
    if (result != 0) {
        return result;
    }
    result = settleTest(track);
    if (result != 0) {
        return result;
    }
    return crossingTest(track);
}
