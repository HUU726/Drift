#include "SelfTest.h"

#include "GameObject/RampObject.h"
#include "Physics/BoxCollider.h"
#include "Physics/PhysicsMaterial.h"
#include "Physics/RampCollider.h"
#include "Physics/TrackCollider.h"
#include "System/ColliderManager.h"
#include "TestCourse.h"
#include "TougeCourse.h"
#include "Vehicle/Powertrain.h"
#include "Vehicle/VehicleSpec.h"
#include "car.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr float kStep = 1.0f / 60.0f;

// ゲームと同じ順（Car の更新 → 物理ステップ）で車を動かす。描画と音は使わない。
struct Rig {
    explicit Rig(const Track& track) : wall(track) {
        car.reset(track);
        car.groundProbe = [this](float x, float z, float yHint, float preferS) {
            return physics.GroundAt(x, z, yHint, preferS);
        };
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
        car.update(kStep, input, &track);
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

// 周回位置 s のコースの向き（サンプルを 1 つおきに見て、いちばん近い点の区間の向き）。
float trackYawAt(const Track& track, float s) {
    if (s >= track.length) {
        s -= track.length;
    }
    const int n = static_cast<int>(track.samples.size());
    int nearest = 0;
    float nearestGap = 1.0e9f;
    for (int i = 0; i < n; i += 2) {
        float gap = std::fabs(track.samples[i].s - s);
        if (gap > track.length * 0.5f) {
            gap = track.length - gap;
        }
        if (gap < nearestGap) {
            nearestGap = gap;
            nearest = i;
        }
    }
    int next = (nearest + 1) % n;
    return std::atan2(track.samples[next].x - track.samples[nearest].x, track.samples[next].z - track.samples[nearest].z);
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
        // ハンドルとアクセルは 70 m 先のコースの向きで決める。ブレーキは、止まりきれる距離
        //（タイヤで出せる減速は約 9 m/s²。余裕を見て 7 m/s²）の先の向きで決める。
        float speedNow = std::max(car.ForwardSpeed(), 0.0f);
        float targetYaw = trackYawAt(track, pose.s + 70.0f);
        float brakeYaw = trackYawAt(track, pose.s + std::max(70.0f, speedNow * speedNow / (2.0f * 7.0f)));
        float errNow = wrapPi(pose.yaw - car.body.yaw);
        float errAhead = wrapPi(targetYaw - car.body.yaw);
        float bend = std::fabs(wrapPi(targetYaw - pose.yaw));
        float brakeBend = std::max(bend, std::fabs(wrapPi(brakeYaw - pose.yaw)));
        DriveInput input;
        input.steer = clampf(errNow * 2.6f + errAhead * 0.5f, -1.0f, 1.0f);
        input.steer = clampf(input.steer - pose.lateral * 0.12f, -1.0f, 1.0f);
        input.throttle = bend < 0.15f ? 1.0f : 0.45f;
        input.brake = 0;
        if (brakeBend > 0.28f && car.ForwardSpeed() > 22.0f) {
            input.throttle = 0;
            input.brake = 1.0f;
        } else if (brakeBend > 0.12f && car.ForwardSpeed() > 34.0f) {
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
    // 全力ブレーキ（グリップ μ 2.0 で約 2G）では前がバンプストップに少し当たる。深く入らなければよい。
    if (maxAir > 0.2f || maxCompression > VehicleSpec::kBumpStop + 0.02f) return 8;
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

// ジャンプ台を 1 回通る。label は表示用、airControl で空中姿勢制御を使うか、throttleInAir で空中でアクセルを
// 踏み続けるかを決める。空中での pitch の最小（鼻先がいちばん下がった値）を minPitch に返す。
int rampRun(const Track& track, const RampSpec& spec, const char* label, bool airControl, bool throttleInAir,
            float& minPitch) {
    Rig rig(track);
    RampCollider ramp(track, spec);
    rig.physics.Register(&ramp);
    Car& car = rig.car;
    car.airControl.enabled = airControl;

    // 台の 40 m 手前、台と同じ横位置に、コースの向きで置く。
    const int n = static_cast<int>(track.samples.size());
    float startS = spec.s - 40.0f;
    int i = 0;
    for (int k = 0; k < n; ++k) {
        if (sGap(track, track.samples[k].s, startS) < sGap(track, track.samples[i].s, startS)) {
            i = k;
        }
    }
    const TrackSample& a = track.samples[i];
    const TrackSample& b = track.samples[(i + 1) % n];
    Vec3 forward = normalize(Vec3{b.x - a.x, 0, b.z - a.z});
    Vec3 right{forward.z, 0, -forward.x};
    car.body.position = Vec3{a.x, a.y + VehicleSpec::kCenterOfMassY, a.z} + right * spec.lateral;
    car.body.yaw = std::atan2(forward.x, forward.z);
    car.body.velocity = forward * 30.0f;
    car.ground = track.query(car.body.position.x, car.body.position.z, a.y, a.s);

    bool allAirborne = false;
    float air = 0;
    float maxAir = 0;
    float takeoffS = -1;
    float landingS = -1;
    minPitch = 0;
    for (int step = 0; step < 60 * 7; ++step) {
        // 3 秒で台を越えて着地する。その後はブレーキで止め、止まった状態で落ち着きを見る（後退に入らない強さ）。
        DriveInput input;
        input.brake = step >= 60 * 3 ? 0.4f : 0.0f;
        if (throttleInAir && car.Airborne()) {
            input.throttle = 1.0f;
        }
        rig.Step(input, track);
        if (car.Airborne()) {
            if (!allAirborne) {
                takeoffS = car.ground.s;
            }
            allAirborne = true;
            air += kStep;
            maxAir = std::max(maxAir, air);
            minPitch = std::min(minPitch, car.body.pitch);
        } else if (air > 0) {
            landingS = car.ground.s;
            air = 0;
        }
        const Rigidbody& body = car.body;
        if (!std::isfinite(body.position.y) || !std::isfinite(body.pitch) || !std::isfinite(body.roll) ||
            !std::isfinite(body.velocity.x)) {
            std::printf("self-test ramp %s non-finite\n", label);
            return 1;
        }
    }
    const Rigidbody& body = car.body;
    float height = car.Origin().y - groundHeight(track, car);
    std::printf("self-test ramp %-14s airtime %.2f s jump %.1f m min pitch %.3f, end speed %.1f km/h pitch %.3f roll %.3f vy %.3f height %.3f\n",
                label, maxAir, landingS - takeoffS, minPitch, car.ForwardSpeed() * 3.6f, body.pitch, body.roll,
                body.velocity.y, height);
    if (!allAirborne) return 13;
    if (car.Airborne() || std::fabs(body.roll) > 0.2f || std::fabs(body.pitch) > 0.2f) return 14;
    if (std::fabs(body.velocity.y) > 0.1f || std::fabs(height) > 0.05f) return 15;
    return 0;
}

// ramps.txt の最初のジャンプ台を 30 m/s で通る。4 輪とも路面から離れ、着地した後に姿勢が戻り、
// 止まったときに落ち着いていて、値が発散しないことを、空中姿勢制御なし・あり・ありでアクセルを踏む、の 3 通りで確かめる。
// アクセルを踏んだときは、なしのときより鼻先が下がる（pitch が 0.1 rad 以上小さくなる）ことも確かめる。
int rampTest(const Track& track) {
    std::vector<RampSpec> specs = RampCollider::Load(RampObject::FindPath());
    if (specs.empty()) {
        std::printf("self-test ramp: ramps.txt がない\n");
        return 12;
    }
    const RampSpec& spec = specs.front();
    float pitchOff = 0;
    float pitchOn = 0;
    float pitchThrottle = 0;
    int result = rampRun(track, spec, "air-control-off", false, false, pitchOff);
    if (result == 0) result = rampRun(track, spec, "air-control-on", true, false, pitchOn);
    if (result == 0) result = rampRun(track, spec, "throttle-in-air", true, true, pitchThrottle);
    if (result != 0) {
        return result;
    }
    if (pitchThrottle > pitchOff - 0.1f) {
        std::printf("self-test ramp: アクセルで鼻先が下がらない（%.3f / %.3f）\n", pitchThrottle, pitchOff);
        return 17;
    }
    return 0;
}

// エンジンと変速だけを、滑らない車輪の 1 次元の車で 2 分間全開にする。
// 6 速まで上がり、最高速度が 82 m/s 前後（75〜90 m/s）になることを確かめる。
int powertrainTest() {
    EnginePowertrain engine;
    engine.Reset();
    const float mass = VehicleSpec::kMass;
    const float radius = VehicleSpec::kWheelRadius;
    float v = 0;
    float to100 = -1;
    int topGear = 0;
    for (int step = 0; step < 60 * 120; ++step) {
        float torque = engine.Update(1.0f, false, v / radius, kStep);
        float force = torque / radius - mass * VehicleSpec::kDrag * v * v - mass * VehicleSpec::kRollingDecel;
        v = std::max(0.0f, v + force / mass * kStep);
        topGear = std::max(topGear, engine.Gear());
        if (to100 < 0 && v >= 100.0f / 3.6f) {
            to100 = step * kStep;
        }
    }
    std::printf("self-test powertrain top %.1f m/s (%.0f km/h) gear %d rpm %.0f, 0-100 km/h %.2f s (no wheelspin)\n",
                v, v * 3.6f, topGear, engine.Rpm(), to100);
    if (topGear != 6 || v < 75.0f || v > 90.0f) return 16;
    return 0;
}

// 試験コース（TestScene と同じ配置）で車を動かす。コースがないので、車の下の路面は ColliderManager だけで調べる。
struct CourseRig {
    CourseRig() {
        course.Build();
        physics.Init();
        physics.Register(&course.ground);
        for (auto& ramp : course.ramps) {
            physics.Register(ramp.get());
        }
        physics.Register(&car.body);
        for (WheelCollider& w : car.wheels) {
            physics.Register(&w);
        }
        box.body = &car.body;
        physics.Register(&box);
        car.groundProbe = [this](float x, float z, float yHint, float preferS) {
            return physics.GroundAt(x, z, yHint, preferS);
        };
        car.Place(course.spawn, course.spawnYaw);
    }
    ~CourseRig() { physics.UnInit(); }

    void Step(const DriveInput& input) {
        car.update(kStep, input, nullptr);
        physics.Step(kStep);
    }
    // 原点を (x, 0, z) に、+z 向きに、前向きの速度 speed で置く。
    void Launch(float x, float z, float speed) {
        car.Place(Vec3{x, course.ground.height, z}, 0.0f);
        car.body.velocity = car.body.Forward() * speed;
        for (WheelCollider& w : car.wheels) {
            w.omega = speed / w.radius;
        }
    }

    TestCourse course;
    Car car;
    BoxCollider box;
    System::ColliderManager physics;
};

// 試験コースの直線で 100 km/h まで加速してから、後退に入らない強さ（0.45）でブレーキを掛け、止まるまでの距離（m）。
float brakingDistance(float x) {
    CourseRig rig;
    rig.car.Place(Vec3{x, 0, -50}, 0.0f);
    DriveInput full;
    full.throttle = 1;
    for (int step = 0; step < 60 * 20 && rig.car.ForwardSpeed() < 100.0f / 3.6f; ++step) {
        rig.Step(full);
    }
    Vec3 start = rig.car.body.position;
    DriveInput brake;
    brake.brake = 0.45f;
    for (int step = 0; step < 60 * 10 && rig.car.ForwardSpeed() > 0.3f; ++step) {
        rig.Step(brake);
    }
    Vec3 d = rig.car.body.position - start;
    return std::sqrt(d.x * d.x + d.z * d.z);
}

// 試験コースで、平面に置いた車の落ち着き、アスファルトとダートのブレーキの距離、4 台のジャンプ台を確かめる。
int testCourseTest() {
    {
        CourseRig rig;
        DriveInput none;
        for (int step = 0; step < 60; ++step) {
            rig.Step(none);
        }
        float height = rig.car.Origin().y - rig.course.ground.height;
        std::printf("self-test course settle vy %.4f m/s height %.4f m\n", rig.car.body.velocity.y, height);
        if (std::fabs(rig.car.body.velocity.y) >= 0.01f || std::fabs(height) > 0.01f) return 18;
    }

    float asphalt = brakingDistance(0.0f);
    float dirt = brakingDistance(130.0f);
    std::printf("self-test course 100-0 km/h asphalt %.1f m dirt %.1f m\n", asphalt, dirt);
    if (asphalt <= 0 || dirt < asphalt * 1.2f) return 19;

    // 各台の 30 m 手前から 25 m/s で入る。浮いた後に着地し、ブレーキで止めたときに姿勢と車高が戻ること。
    for (const TestCourse::RampPlace& place : TestCourse::kRamps) {
        CourseRig rig;
        rig.Launch(place.x, TestCourse::kRampZ - 30.0f, 25.0f);
        float maxAir = 0;
        float maxHeight = 0;
        for (int step = 0; step < 60 * 8; ++step) {
            DriveInput input;
            input.brake = step >= 60 * 4 ? 0.4f : 0.0f;
            rig.Step(input);
            maxAir = std::max(maxAir, rig.car.airControl.AirTime());
            maxHeight = std::max(maxHeight, rig.car.Origin().y);
            const Rigidbody& body = rig.car.body;
            if (!std::isfinite(body.position.y) || !std::isfinite(body.pitch) || !std::isfinite(body.roll)) {
                std::printf("self-test course ramp %.0f deg non-finite\n", place.angleDeg);
                return 1;
            }
        }
        const Rigidbody& body = rig.car.body;
        float height = rig.car.Origin().y - rig.course.ground.height;
        std::printf("self-test course ramp %2.0f deg airtime %.2f s top %.2f m, end speed %.1f km/h pitch %.3f roll %.3f height %.3f\n",
                    place.angleDeg, maxAir, maxHeight, rig.car.ForwardSpeed() * 3.6f, body.pitch, body.roll, height);
        if (maxAir <= 0.1f) return 20;
        if (rig.car.Airborne() || std::fabs(body.pitch) > 0.2f || std::fabs(body.roll) > 0.2f || std::fabs(height) > 0.05f) {
            return 21;
        }
    }
    return 0;
}

struct SteerResult {
    int slidingSteps = 0;  // どれかの車輪がピークを越えて滑っていたステップ数
    float maxFront = 0;    // 前輪の滑りの角度の最大（rad）
    float maxRear = 0;     // 後輪の滑りの角度の最大（rad）
    float maxSideSlip = 0; // 車体の横の速さの最大（m/s）
};

// アスファルトで速度 kmh からハンドルをいっぱいに切り、2 秒間のタイヤの滑りを測る。
// handbrakeSteps の間はサイドブレーキも掛ける。
SteerResult steerRun(float kmh, float throttle, int handbrakeSteps) {
    CourseRig rig;
    rig.Launch(-40.0f, -40.0f, kmh / 3.6f);
    SteerResult result;
    int& slidingSteps = result.slidingSteps;
    float& maxFront = result.maxFront;
    float& maxRear = result.maxRear;
    float& maxSideSlip = result.maxSideSlip;
    for (int step = 0; step < 120; ++step) {
        DriveInput input;
        input.steer = 1;
        input.throttle = throttle;
        input.handbrake = step < handbrakeSteps;
        rig.Step(input);
        const Car& car = rig.car;
        bool sliding = false;
        for (int i = 0; i < Car::kWheelCount; ++i) {
            sliding = sliding || car.wheels[i].sliding;
            float angle = std::fabs(car.wheels[i].slipAngle);
            if (i < Car::kRearLeft) maxFront = std::max(maxFront, angle);
            else maxRear = std::max(maxRear, angle);
        }
        slidingSteps += sliding ? 1 : 0;
        maxSideSlip = std::max(maxSideSlip, std::fabs(car.body.LateralSpeed()));
    }
    std::printf("self-test steer %3.0f km/h throttle %.1f handbrake %3d sliding %3d/120 front %.1f deg rear %.1f deg side %.2f m/s end %.0f km/h\n",
                kmh, throttle, handbrakeSteps, slidingSteps, maxFront * 57.3f, maxRear * 57.3f, maxSideSlip,
                rig.car.ForwardSpeed() * 3.6f);
    return result;
}

// ハンドルを切っただけでは滑らないこと（アクセルを離したまま切る）と、サイドブレーキで後ろが流れること。
int steerTest() {
    for (float kmh : {40.0f, 70.0f, 100.0f, 130.0f}) {
        SteerResult coast = steerRun(kmh, 0.0f, 0);
        steerRun(kmh, 1.0f, 0);
        // 後輪の滑りの角度がピーク（約 6.9°）を大きく越えたり、車体が横へ流れたりしたらスピンしかけている。
        // 横の速さは、速さに対する角度（10°）で見る。グリップが高いほどきつく曲がれ、その分の横の速さは増える。
        float sideLimit = kmh / 3.6f * std::tan(10.0f / 57.3f);
        if (coast.maxRear > 9.0f / 57.3f || coast.maxSideSlip > sideLimit) return 22;
    }
    SteerResult handbrake = steerRun(70.0f, 0.0f, 36);
    if (handbrake.maxRear < 20.0f / 57.3f) return 23;
    return 0;
}

struct DriftResult {
    float minAngle = 1e9f;   // 保っている間の車体の滑りの角度の最小（rad）
    float maxAngle = 0;      // 最大（rad）
    bool heldAll = true;     // 保つ間ずっとドリフトの状態だったか
    float endSpeed = 0;      // 最後の速度（km/h）
    bool endedStraight = false;  // サイドブレーキとアクセルを離した後、ドリフトを抜けてまっすぐに戻ったか
    float curveRate = 0;     // 保っている間に進む向きが回った速さの平均（deg/s、右が正）
};

// 速度 kmh から、左へハンドルを切りながらサイドブレーキを押して向きを変え（0.5 秒）、
// サイドブレーキを押したまま、アクセル throttle、ハンドルはまっすぐで holdSeconds 保つ。その後すべて離して 3 秒待つ。
DriftResult driftRun(float kmh, float throttle, float steer, float holdSeconds) {
    CourseRig rig;
    rig.course.ground.baseMaterial = &Materials::kAsphalt;  // 平面の外へ出てもアスファルトで続ける
    rig.Launch(0.0f, -40.0f, kmh / 3.6f);
    Car& car = rig.car;
    DriftResult result;
    const int entry = 30;
    const int hold = entry + static_cast<int>(holdSeconds * 60);
    std::printf("self-test drift %3.0f km/h throttle %.1f steer %4.1f:", kmh, throttle, steer);
    float turned = 0;  // 保っている間に進む向きが回った角度（rad）
    float prevHeading = 0;
    for (int step = 0; step < hold + 180; ++step) {
        DriveInput input;
        // ドリフトはサイドブレーキを押している間だけなので、保つ間も押し続ける。
        if (step < entry) {
            input.steer = -1;
            input.handbrake = true;
        } else if (step < hold) {
            input.throttle = throttle;
            input.steer = steer;
            input.handbrake = true;
        }
        rig.Step(input);
        float heading = std::atan2(car.body.velocity.x, car.body.velocity.z);
        float delta = heading - prevHeading;
        delta = std::atan2(std::sin(delta), std::cos(delta));
        prevHeading = heading;
        // 入ってから 1 秒後から、保てているかを見る。
        if (step > entry + 60 && step < hold) {
            turned += delta;
        }
        if (step >= entry + 60 && step < hold) {
            float angle = std::fabs(car.driftAngle);
            result.minAngle = std::min(result.minAngle, angle);
            result.maxAngle = std::max(result.maxAngle, angle);
            result.heldAll = result.heldAll && car.drifting;
        }
        if (step % 30 == 29) {
            std::printf(" %3.0f%s/%3.0f", car.driftAngle * 57.3f, car.drifting ? "d" : " ", car.ForwardSpeed() * 3.6f);
        }
        if (step == hold - 1) {
            result.endSpeed = car.ForwardSpeed() * 3.6f;
        }
    }
    result.endedStraight = !car.drifting && std::fabs(car.driftAngle) < 0.1f;
    result.curveRate = turned * 57.3f / (holdSeconds - 1.0f);
    std::printf("\n");
    return result;
}

// ドリフトを保てること。サイドブレーキで向きを変えた後、アクセルを踏んだまま 4 秒、角度と速度を保ち、
// サイドブレーキを離すとまっすぐに戻ること。
int driftTest() {
    for (float kmh : {80.0f, 100.0f, 120.0f}) {
        for (float throttle : {0.5f, 1.0f}) {
            DriftResult r = driftRun(kmh, throttle, 0.0f, 4.0f);
            std::printf("self-test drift   angle %.0f..%.0f deg held %d end %.0f km/h straight %d\n", r.minAngle * 57.3f,
                        r.maxAngle * 57.3f, r.heldAll ? 1 : 0, r.endSpeed, r.endedStraight ? 1 : 0);
            if (!r.heldAll || !r.endedStraight) return 24;
            // アクセル半分なら浅く（12〜35°）、全開なら深く（30〜55°）。全開でも入った速度の 6 割は保つ。
            float low = throttle < 0.75f ? 12.0f : 30.0f;
            float high = throttle < 0.75f ? 35.0f : 55.0f;
            if (r.minAngle < low / 57.3f || r.maxAngle > high / 57.3f) return 25;
            if (r.endSpeed < kmh * 0.6f) return 26;
        }
    }
    // スティックの倒す量で曲がりが決まること。倒すほど大きく曲がり、倒した向きへ曲がる。
    float prevRate = -1e9f;
    for (float steer : {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f}) {
        DriftResult r = driftRun(100.0f, 1.0f, steer, 4.0f);
        std::printf("self-test drift   steer %4.1f curve %.0f deg/s angle %.0f..%.0f deg held %d\n", steer, r.curveRate,
                    r.minAngle * 57.3f, r.maxAngle * 57.3f, r.heldAll ? 1 : 0);
        if (!r.heldAll || r.curveRate <= prevRate + 5.0f) return 27;
        prevRate = r.curveRate;
    }
    return 0;
}

// 中心線の上で、周回位置 s の点。
Vec3 centerAt(const Track& track, float s) {
    s = std::fmod(s, track.length);
    if (s < 0) {
        s += track.length;
    }
    const int n = static_cast<int>(track.samples.size());
    auto it = std::upper_bound(track.samples.begin(), track.samples.end(), s,
                               [](float value, const TrackSample& sample) { return value < sample.s; });
    int i = std::max(0, static_cast<int>(it - track.samples.begin()) - 1);
    const TrackSample& a = track.samples[i];
    const TrackSample& b = track.samples[(i + 1) % n];
    float next = i + 1 < n ? b.s : track.length;
    float t = next > a.s ? (s - a.s) / (next - a.s) : 0.0f;
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

// 峠のコース。形（つながっている、道どうしが近づきすぎない、急すぎない）と、自動運転で 1 周走れることを確かめる。
int tougeTest() {
    Track track;
    if (!BuildTougeCourse(track)) return 40;
    const int n = static_cast<int>(track.samples.size());
    float maxGrade = 0;
    float maxGap = 0;
    float minClearance = 1e9f;
    for (int i = 0; i < n; ++i) {
        const TrackSample& a = track.samples[i];
        const TrackSample& b = track.samples[(i + 1) % n];
        float run = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
        maxGap = std::max(maxGap, run);
        maxGrade = std::max(maxGrade, std::fabs(b.y - a.y) / std::max(run, 0.01f));
        // 周回で 80 m 以上離れた点どうしの、平面の距離。ガードレールの外どうしが重ならないこと。
        for (int j = i + 1; j < n; ++j) {
            float along = std::fabs(track.samples[j].s - a.s);
            along = std::min(along, track.length - along);
            if (along < 80.0f) continue;
            const TrackSample& c = track.samples[j];
            minClearance = std::min(minClearance, std::sqrt((c.x - a.x) * (c.x - a.x) + (c.z - a.z) * (c.z - a.z)));
        }
    }
    std::printf("self-test touge length %.0f samples %d max gap %.1f m max grade %.1f%% min clearance %.1f m\n",
                track.length, n, maxGap, maxGrade * 100.0f, minClearance);
    if (maxGap > 6.0f) return 41;
    if (maxGrade > 0.11f) return 42;
    if (minClearance < 2.0f * track.wallLateral() + 2.0f) return 43;

    // 曲がりの半径から出せる速さ（横の加速度 10 m/s²）を決め、止まれる減速（7 m/s²）で手前から下げる。
    std::vector<float> limit(n);
    for (int i = 0; i < n; ++i) {
        const TrackSample& p = track.samples[(i + n - 3) % n];
        const TrackSample& q = track.samples[i];
        const TrackSample& r = track.samples[(i + 3) % n];
        float a = std::hypot(q.x - p.x, q.z - p.z);
        float b = std::hypot(r.x - q.x, r.z - q.z);
        float c = std::hypot(r.x - p.x, r.z - p.z);
        float area2 = std::fabs((q.x - p.x) * (r.z - p.z) - (q.z - p.z) * (r.x - p.x));
        float radius = area2 > 1e-3f ? a * b * c / (2.0f * area2) : 1e4f;
        limit[i] = std::min(40.0f, std::sqrt(10.0f * radius));
    }
    for (int pass = 0; pass < 2; ++pass) {
        for (int k = 2 * n - 1; k >= 0; --k) {
            int i = k % n;
            int j = (i + 1) % n;
            float ds = std::hypot(track.samples[j].x - track.samples[i].x, track.samples[j].z - track.samples[i].z);
            limit[i] = std::min(limit[i], std::sqrt(limit[j] * limit[j] + 2.0f * 7.0f * ds));
        }
    }

    Rig rig(track);
    Car& car = rig.car;
    float travelled = 0;
    float prevS = -1;
    float maxLat = 0;
    float air = 0;
    float maxAir = 0;
    int steps = 0;
    for (; steps < 60 * 240 && travelled < track.length; ++steps) {
        TrackPose pose = track.queryLocal(car.body.position.x, car.body.position.z, car.Origin().y,
                                          car.ground.found ? car.ground.s : -1.0f);
        float speed = std::max(car.ForwardSpeed(), 0.0f);
        // 少し先の中心線の点へハンドルを向ける。
        Vec3 target = centerAt(track, pose.s + 8.0f + speed * 0.35f);
        float aim = wrapPi(std::atan2(target.x - car.body.position.x, target.z - car.body.position.z) - car.body.yaw);
        auto it = std::upper_bound(track.samples.begin(), track.samples.end(), pose.s,
                                   [](float value, const TrackSample& sample) { return value < sample.s; });
        int index = std::max(0, static_cast<int>(it - track.samples.begin()) - 1);
        float want = limit[(index + 2) % n];
        DriveInput input;
        input.steer = clampf(aim * 3.0f, -1.0f, 1.0f);
        if (speed > want + 0.5f) {
            input.brake = clampf((speed - want) / 3.0f, 0.2f, 1.0f);
        } else {
            input.throttle = clampf((want - speed) / 3.0f + 0.3f, 0.0f, 1.0f);
        }
        rig.Step(input, track);
        if (prevS >= 0) {
            float ds = car.ground.s - prevS;
            if (ds < -track.length * 0.5f) ds += track.length;
            if (ds > track.length * 0.5f) ds -= track.length;
            if (ds > 0) travelled += ds;
        }
        prevS = car.ground.s;
        maxLat = std::max(maxLat, std::fabs(pose.lateral));
        bool airborne = true;
        for (const WheelCollider& w : car.wheels) {
            airborne = airborne && !w.grounded;
        }
        air = airborne ? air + kStep : 0.0f;
        maxAir = std::max(maxAir, air);
        if (!std::isfinite(car.body.position.x) || !std::isfinite(car.ForwardSpeed())) return 44;
    }
    std::printf("self-test touge lap %.1f s travelled %.0f m max lateral %.1f m airtime %.2f s\n", steps * kStep,
                travelled, maxLat, maxAir);
    if (travelled < track.length) return 45;
    if (maxLat > track.halfWidth + track.curbWidth) return 46;
    if (maxAir > 0.2f) return 47;
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
    result = crossingTest(track);
    if (result != 0) {
        return result;
    }
    result = rampTest(track);
    if (result != 0) {
        return result;
    }
    result = powertrainTest();
    if (result != 0) {
        return result;
    }
    result = testCourseTest();
    if (result != 0) {
        return result;
    }
    result = steerTest();
    if (result != 0) {
        return result;
    }
    result = driftTest();
    if (result != 0) {
        return result;
    }
    return tougeTest();
}
