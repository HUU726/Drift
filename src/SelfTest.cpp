#include "SelfTest.h"

#include "GameObject/RampObject.h"
#include "Physics/BoxCollider.h"
#include "Physics/PhysicsMaterial.h"
#include "Physics/RampCollider.h"
#include "Physics/TrackCollider.h"
#include "System/ColliderManager.h"
#include "System/SoundSynth.h"
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
// 平らな試験の地面で、kmh から左へサイドブレーキで向きを変え（0.5 秒）、その後は control(step, input) が
// 決める入力で走る。Space は押したまま。ステップごとに observe(step) を呼ぶ。
template <class Control, class Observe, class Setup>
void driftScenario(float kmh, int steps, Control control, Observe observe, Setup setup) {
    CourseRig rig;
    rig.course.ground.baseMaterial = &Materials::kAsphalt;
    rig.Launch(0.0f, -40.0f, kmh / 3.6f);
    setup(rig.car);
    for (int step = 0; step < steps; ++step) {
        DriveInput input;
        input.handbrake = true;
        if (step < 30) {
            input.steer = -1;
        } else {
            control(step, input);
        }
        rig.Step(input);
        observe(step, rig.car);
    }
}

// setup(car) で車の設定を変えてから走る版。
template <class Control, class Observe>
void driftScenario(float kmh, int steps, Control control, Observe observe) {
    driftScenario(kmh, steps, control, observe, [](Car&) {});
}

// ドリフト中にブレーキを踏んでも、ドリフトが切れずにスピンせず、速さだけが落ちること。
int driftBrakeTest() {
    for (float brake : {0.4f, 0.7f, 1.0f}) {
        const int brakeFrom = 120;
        const int brakeTo = brakeFrom + 36;  // 0.6 秒
        bool held = true;
        float minAngle = 1e9f, maxAngle = 0;
        float before = 0, after = 0;
        driftScenario(80.0f, 300,
                      [&](int step, DriveInput& input) {
                          input.throttle = 0.7f;
                          if (step >= brakeFrom && step < brakeTo) {
                              input.throttle = 0;
                              input.brake = brake;
                          }
                      },
                      [&](int step, const Car& car) {
                          if (step == brakeFrom - 1) before = car.ForwardSpeed() * 3.6f;
                          if (step == brakeTo) after = car.ForwardSpeed() * 3.6f;
                          if (step >= 90) {
                              held = held && car.drifting;
                              minAngle = std::min(minAngle, std::fabs(car.driftAngle));
                              maxAngle = std::max(maxAngle, std::fabs(car.driftAngle));
                          }
                      });
        std::printf("self-test drift brake %.1f: held %d angle %.0f..%.0f deg speed %.0f -> %.0f km/h\n", brake,
                    held ? 1 : 0, minAngle * 57.3f, maxAngle * 57.3f, before, after);
        // 切れず、角度は 10〜65° に収まり（スピンしない）、0.6 秒で 5 km/h 以上落ちる。
        if (!held) return 28;
        if (minAngle < 10.0f / 57.3f || maxAngle > 65.0f / 57.3f) return 29;
        if (before - after < 5.0f) return 30;
    }
    return 0;
}

// Space を押したまま、アクセルを抜いてスティックを反対へ倒すと、ドリフトのまま反対側へ振り返せること。
// アクセルを踏んだまま反対へ倒したとき（ラインを外へ膨らませる）は、振り返さないこと。
// 1 回の振り返し（左のドリフトから右へ）、直線で左右へ 4 回続けるマンジ、外へ膨らませる操作を試す。
int driftSwitchTest() {
    // 1 回: 左のドリフト（角度が正）を 1.5 秒保ち、アクセルを抜いて右へ倒す（0.25 秒抜き、1 秒倒す）。
    // いっぱいに倒したときと、パッドで軽く倒したとき（0.55）の両方で振り返せ、振り返す間に進む向きが
    // 大きく回らない（倒したスティックで内側へ曲がり込まない）こと。
    for (float stick : {1.0f, 0.55f}) {
        const int switchAt = 120;
        int flipped = -1;
        bool held = true;
        float after = 0;
        float maxAngle = 0;
        float prevHeading = 0, turned = 0;
        driftScenario(80.0f, 330,
                      [&](int step, DriveInput& input) {
                          bool flick = step >= switchAt && step < switchAt + 15;
                          input.throttle = flick ? 0.0f : 0.6f;
                          input.steer = step >= switchAt && step < switchAt + 60 ? stick : 0.0f;
                      },
                      [&](int step, const Car& car) {
                          float heading = std::atan2(car.body.velocity.x, car.body.velocity.z);
                          if (step > switchAt && flipped < 0) turned += wrapPi(heading - prevHeading);
                          prevHeading = heading;
                          if (step >= 90) held = held && car.drifting;
                          if (step >= switchAt && flipped < 0 && car.driftAngle < -0.26f) flipped = step;
                          if (step >= switchAt) maxAngle = std::max(maxAngle, std::fabs(car.driftAngle));
                          if (step == 329) after = car.driftAngle;
                      });
        float seconds = flipped >= 0 ? (flipped - switchAt) * kStep : -1.0f;
        std::printf("self-test drift switch stick %.2f: held %d flipped in %.2f s turned %.0f deg max %.0f deg end %.0f deg\n",
                    stick, held ? 1 : 0, seconds, turned * 57.3f, maxAngle * 57.3f, after * 57.3f);
        // 切れずに、1 秒以内に右へ 15° 以上移り、スピンせず、倒すのをやめた後も右のドリフトを保つ。
        if (!held || flipped < 0 || seconds > 1.0f) return 31;
        if (maxAngle > 70.0f / 57.3f || after > -0.17f) return 32;
        // 振り返す間に、進む向きが 12° より大きく回らない（直す前は 18° 回って内側へ切れ込んでいた）。
        if (std::fabs(turned) > 12.0f / 57.3f) return 36;
    }
    // マンジ: 1.2 秒ごとに、アクセルを一瞬抜いて左右へ倒す。毎回、反対側へ移ること。
    {
        int switches = 0;
        float prevSign = 1;
        bool held = true;
        float maxAngle = 0;
        driftScenario(80.0f, 30 + 72 * 5,
                      [&](int step, DriveInput& input) {
                          int phase = (step - 30) / 72;
                          int inPhase = (step - 30) % 72;
                          input.throttle = phase > 0 && inPhase < 15 ? 0.0f : 0.6f;
                          input.steer = phase == 0 ? 0.0f : (phase % 2 ? 1.0f : -1.0f);
                      },
                      [&](int step, const Car& car) {
                          if (step < 90) return;
                          held = held && car.drifting;
                          maxAngle = std::max(maxAngle, std::fabs(car.driftAngle));
                          float sign = car.driftAngle > 0.17f ? 1.0f : (car.driftAngle < -0.17f ? -1.0f : prevSign);
                          if (sign != prevSign) switches++;
                          prevSign = sign;
                      });
        std::printf("self-test drift manji: held %d switches %d max %.0f deg\n", held ? 1 : 0, switches, maxAngle * 57.3f);
        if (!held || switches < 4 || maxAngle > 70.0f / 57.3f) return 33;
    }
    // 外へ膨らませる: 左のドリフトのまま、アクセルを踏んだまま右（外）へ半分といっぱいに 2 秒倒す。
    // 左のドリフトのまま（角度が正のまま）で、倒すほど進む向きが右へ回る（左への曲がりが小さくなる）。
    float prevTurn = -1e9f;
    for (float out : {0.0f, 0.5f, 1.0f}) {
        bool sameSide = true;
        float minAngle = 1e9f;
        float prevHeading = 0, turned = 0;
        driftScenario(80.0f, 210,
                      [&](int step, DriveInput& input) {
                          input.throttle = 0.6f;
                          input.steer = step >= 90 ? out : 0.0f;
                      },
                      [&](int step, const Car& car) {
                          float heading = std::atan2(car.body.velocity.x, car.body.velocity.z);
                          if (step > 90) turned += wrapPi(heading - prevHeading);
                          prevHeading = heading;
                          if (step >= 90) {
                              sameSide = sameSide && car.drifting && car.driftAngle > 0.0f;
                              minAngle = std::min(minAngle, car.driftAngle);
                          }
                      });
        float rate = turned * 57.3f / 2.0f;
        std::printf("self-test drift outward %.1f: same side %d min angle %.0f deg turn %.0f deg/s\n", out,
                    sameSide ? 1 : 0, minAngle * 57.3f, rate);
        if (!sameSide || rate <= prevTurn + 3.0f) return 34;
        prevTurn = rate;
    }
    // キーボード: アクセルを踏まずに右（外）のキーを 1 秒押しても振り返さず、左のドリフトのままラインを外へ膨らませる。
    {
        bool sameSide = true;
        driftScenario(80.0f, 210,
                      [&](int step, DriveInput& input) {
                          input.throttle = step >= 90 ? 0.0f : 0.6f;
                          input.steer = step >= 90 && step < 150 ? 1.0f : 0.0f;
                          input.keyboardSteer = input.steer != 0;
                      },
                      [&](int step, const Car& car) {
                          if (step >= 90 && step < 150 && car.drifting) sameSide = sameSide && car.driftAngle > 0.0f;
                      });
        std::printf("self-test drift keyboard outward: same side %d\n", sameSide ? 1 : 0);
        if (!sameSide) return 37;
    }
    return 0;
}

// ヘアピンの速さでも小さく回れること。kmh で左へ向きを変え、スティックを左（内）へいっぱいに倒したまま
// アクセル半分で 2 秒回る。その間の進む向きの回る速さから、回った半径を求める。
float hairpinRadius(float kmh, float& speedOut) {
    float prevHeading = 0, turned = 0, speedSum = 0;
    int count = 0;
    driftScenario(kmh, 30 + 30 + 120,
                  [&](int step, DriveInput& input) {
                      input.throttle = 0.5f;
                      input.steer = -1.0f;
                      (void)step;
                  },
                  [&](int step, const Car& car) {
                      float heading = std::atan2(car.body.velocity.x, car.body.velocity.z);
                      if (step > 60) {
                          turned += wrapPi(heading - prevHeading);
                          speedSum += std::hypot(car.body.velocity.x, car.body.velocity.z);
                          count++;
                      }
                      prevHeading = heading;
                  });
    float rate = std::fabs(turned) / (count * kStep);
    speedOut = speedSum / count;
    return speedOut / std::max(rate, 1e-3f);
}

struct ArcResult {
    float speed = 0;   // 後の 2 秒の平均の速さ（m/s）
    float radius = 0;  // 回った半径（m）
    float angle = 0;   // 平均の角度（rad）
};

// スティックを stick 倒し、アクセル 0.8 でドリフトのまま 3 秒回る（後の 2 秒を測る）。
ArcResult arcRun(float kmh, float stick, float boost) {
    float prevHeading = 0, turned = 0, speedSum = 0, angleSum = 0;
    int count = 0;
    driftScenario(kmh, 30 + 180,
                  [&](int step, DriveInput& input) {
                      (void)step;
                      input.throttle = 0.8f;
                      input.steer = -stick;
                  },
                  [&](int step, const Car& car) {
                      float heading = std::atan2(car.body.velocity.x, car.body.velocity.z);
                      if (step > 90) {
                          turned += wrapPi(heading - prevHeading);
                          speedSum += std::hypot(car.body.velocity.x, car.body.velocity.z);
                          angleSum += std::fabs(car.driftAngle);
                          count++;
                      }
                      prevHeading = heading;
                  },
                  [&](Car& car) { car.driftBoost = boost; });
    ArcResult r;
    r.speed = speedSum / count;
    r.radius = r.speed / std::max(std::fabs(turned) / (count * kStep), 1e-3f);
    r.angle = angleSum / count;
    return r;
}

// ドリフト中の加速。アクセルを踏んだまま（角度を保ったまま）、同じ弧（半径）で速く回れること。
// 速さはその弧に合う速さ（横の加速度 driftComfortLat で回れる速さ）までで止まり、きつい弧ほど遅く収まること。
int driftBoostTest() {
    const float comfortLat = Car().driftComfortLat;
    for (float stick : {0.4f, 1.0f}) {
        ArcResult slow = arcRun(70.0f, stick, 0.0f);
        ArcResult fast = arcRun(70.0f, stick, Car().driftBoost);
        float comfort = std::sqrt(comfortLat * fast.radius);
        std::printf("self-test drift boost stick %.1f: without %.0f km/h radius %.1f m angle %.0f deg, with %.0f km/h radius %.1f m"
                    " angle %.0f deg (comfort %.0f km/h)\n",
                    stick, slow.speed * 3.6f, slow.radius, slow.angle * 57.3f, fast.speed * 3.6f, fast.radius,
                    fast.angle * 57.3f, comfort * 3.6f);
        // 加速なしより速いか、すでに弧に合う速さにいる。半径の差は 2 割以内。
        if (fast.speed < slow.speed - 0.5f) return 38;
        if (fast.speed < comfort - 2.0f && fast.speed - slow.speed < 3.0f / 3.6f) return 38;
        if (std::fabs(fast.radius - slow.radius) > 0.2f * slow.radius) return 39;
        // 加速で、弧に合う速さを大きく越えない。アクセル 0.8 の角度（約 45°）は 3 割以上浅くならない。
        if (fast.speed > std::max(comfort, slow.speed) + 3.0f) return 73;
        if (fast.angle < slow.angle * 0.7f) return 74;
    }
    return 0;
}

int hairpinTest() {
    // 60 km/h で入って半径 13 m 以下、70 km/h で 13.5 m 以下（峠のヘアピン、半径 15 m に余裕を持って収まる）、
    // 80 km/h でも 17 m 以下（道幅を使えば収まる）。
    const float entry[] = {60.0f, 70.0f, 80.0f};
    const float within[] = {13.0f, 13.5f, 17.0f};
    for (int i = 0; i < 3; ++i) {
        float speed = 0;
        float radius = hairpinRadius(entry[i], speed);
        std::printf("self-test hairpin %2.0f km/h: radius %.1f m at %.0f km/h\n", entry[i], radius, speed * 3.6f);
        if (radius > within[i]) return 35;
    }
    return 0;
}

// ドリフトから普段の走りへ戻るとき（Space を離したとき）に、後輪が浮いて跳ねないこと。
// 戻るグリップの横の力で車体が大きく傾くと、内側の後輪が浮き、逆へ振られて叩きつけられる。
int exitBounceTest() {
    for (float kmh : {80.0f, 120.0f}) {
        for (float throttle : {0.5f, 1.0f}) {
            const int release = 30 + 180;
            int airSteps[2] = {0, 0};
            float minLoad[2] = {1e9f, 1e9f}, maxLoad[2] = {0, 0};
            float maxRoll = 0, maxRollRate = 0, maxVy = 0;
            driftScenario(kmh, release + 120,
                          [&](int step, DriveInput& input) {
                              input.throttle = step < release ? throttle : 0.3f;
                              input.handbrake = step < release;
                          },
                          [&](int step, const Car& car) {
                              if (step < release) return;
                              for (int k = 0; k < 2; ++k) {
                                  const WheelCollider& w = car.wheels[Car::kRearLeft + k];
                                  if (!w.grounded) airSteps[k]++;
                                  minLoad[k] = std::min(minLoad[k], w.load);
                                  maxLoad[k] = std::max(maxLoad[k], w.load);
                              }
                              maxRoll = std::max(maxRoll, std::fabs(car.body.roll));
                              maxRollRate = std::max(maxRollRate, std::fabs(car.body.rollRate));
                              maxVy = std::max(maxVy, std::fabs(car.body.velocity.y));
                          });
            std::printf("self-test exit %3.0f km/h throttle %.1f: rear air %d %d steps load %.0f..%.0f / %.0f..%.0f N roll %.3f rate %.2f vy %.2f\n",
                        kmh, throttle, airSteps[0], airSteps[1], minLoad[0], maxLoad[0], minLoad[1], maxLoad[1], maxRoll,
                        maxRollRate, maxVy);
            // 後輪はほとんど浮かず（2 ステップ以下）、車体の傾きは 6° 以下。
            if (airSteps[0] > 2 || airSteps[1] > 2 || maxRoll > 0.1f) return 37;
        }
    }
    return 0;
}

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
// 作ったコースの形を確かめる（点の間隔、勾配、道どうしが近づきすぎないこと）。0 なら成功、それ以外は失敗の番号。
int courseShape(const Track& track, const char* label) {
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
    std::printf("self-test %s length %.0f samples %d max gap %.1f m max grade %.1f%% min clearance %.1f m\n", label,
                track.length, n, maxGap, maxGrade * 100.0f, minClearance);
    if (maxGap > 6.0f) return 41;
    if (maxGrade > 0.11f) return 42;
    if (minClearance < 2.0f * track.wallLateral() + 2.0f) return 43;
    return 0;
}

int tougeTest() {
    Track track;
    if (!BuildTougeCourse(track)) return 40;
    if (int shape = courseShape(track, "touge")) return shape;
    const int n = static_cast<int>(track.samples.size());

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

// 中心線の点 i での曲がり（1/m、右へ曲がるのが正）。前後 2 点ずつの向きの差から求める。
float curvatureAt(const Track& track, int i) {
    const int n = static_cast<int>(track.samples.size());
    const TrackSample& a = track.samples[(i + n - 2) % n];
    const TrackSample& b = track.samples[i];
    const TrackSample& c = track.samples[(i + 2) % n];
    float h0 = std::atan2(b.x - a.x, b.z - a.z);
    float h1 = std::atan2(c.x - b.x, c.z - b.z);
    float run = std::hypot(b.x - a.x, b.z - a.z) + std::hypot(c.x - b.x, c.z - b.z);
    return run > 1e-3f ? wrapPi(h1 - h0) / (0.5f * run) : 0.0f;
}

int sampleAt(const Track& track, float s) {
    s = std::fmod(s, track.length);
    if (s < 0) s += track.length;
    auto it = std::upper_bound(track.samples.begin(), track.samples.end(), s,
                               [](float value, const TrackSample& sample) { return value < sample.s; });
    return std::max(0, static_cast<int>(it - track.samples.begin()) - 1);
}

// コーナーごとの記録。
struct CornerLog {
    int steps = 0;           // コーナーの中にいたステップ数
    int driftSteps = 0;      // そのうちドリフトしていたステップ数
    float maxAngle = 0;      // 車体の滑りの角度の最大（rad）
    float sumAngle = 0;      // ドリフト中の角度の合計（平均を出す）
    float entrySpeed = -1;   // 入ったときの速さ（m/s）
    float minSpeed = 1e9f;
    float maxLateral = 0;    // 中心からのずれの最大（m）
    int wallSteps = 0;       // 壁に触れていたステップ数
    int nameId = -1;
};

// 峠を、ドリフトで 1 周走る自動運転。コーナーの手前でサイドブレーキ（Space）を押して向きを変え、
// 押したままアクセルで角度を、スティックで曲がりを決めて回り、コーナーを抜けたら離す。
// 直線はグリップで走る。コーナーごとの角度、速さ、壁への接触を表示する。
struct TougeDriveResult {
    float lapTime = 0;
    bool finished = false;
    int spins = 0;
    int wallSteps = 0;
    float driftShare = 0;  // コーナーの中でドリフトしていた割合
    int stops = 0;         // 壁などで止まり、下がり直した回数
};

TougeDriveResult tougeDriveRun(const Track& track, bool verbose, float driftLat = 8.0f) {
    const int n = static_cast<int>(track.samples.size());
    // 曲がりの強いところ（半径 130 m 未満）をドリフトで回る。グリップでの速さの上限（横 10 m/s²）と、
    // ドリフトでの速さの目安（横 kDriftLat）を、止まれる減速で手前から下げる。
    constexpr float kDriftRadius = 130.0f;
    const float kDriftLat = driftLat;
    std::vector<float> curvature(n);
    std::vector<float> limit(n);
    for (int i = 0; i < n; ++i) {
        curvature[i] = curvatureAt(track, i);
        float radius = 1.0f / std::max(std::fabs(curvature[i]), 1e-4f);
        float lat = radius < kDriftRadius ? kDriftLat : 10.0f;
        limit[i] = std::min(40.0f, std::sqrt(lat * radius));
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
    std::vector<CornerLog> logs(32);
    TougeDriveResult result;
    float travelled = 0;
    float prevS = -1;
    bool holding = false;   // Space を押している
    float backing = 0;      // 壁から下がっている残りの時間（秒）
    float releaseWait = 0;  // 切り返しで Space を離している残りの時間（秒）
    float holdDir = 0;      // Space を押したときに向かったコーナーの向き（右が 1）
    bool kickedOnce = false;  // 押してから、一度ドリフトの角度が付いたか
    float stuck = 0;
    int cornerSteps = 0;
    int cornerDrift = 0;
    bool wasDrifting = false;
    int step = 0;
    for (; step < 60 * 300 && travelled < track.length; ++step) {
        TrackPose pose = track.queryLocal(car.body.position.x, car.body.position.z, car.Origin().y,
                                          car.ground.found ? car.ground.s : -1.0f);
        float speed = std::max(car.ForwardSpeed(), 0.0f);
        Vec3 velocity = car.body.velocity;
        float travelYaw = std::hypot(velocity.x, velocity.z) > 1.0f ? std::atan2(velocity.x, velocity.z) : car.body.yaw;
        int index = sampleAt(track, pose.s);

        // 先のコーナー: entry m 先までに、ドリフトで回る曲がりがあるか。
        float entry = 12.0f + speed * 0.7f;
        int cornerAhead = -1;
        for (float d = 0; d <= entry; d += 4.0f) {
            int i = sampleAt(track, pose.s + d);
            if (std::fabs(curvature[i]) > 1.0f / kDriftRadius) {
                cornerAhead = i;
                break;
            }
        }
        // 進む向きを向けたい点（少し先の中心線）。きついコーナーでは近くを見て、外側へ寄せる（内側を切りすぎない）。
        float tight = std::fabs(curvature[sampleAt(track, pose.s + 10.0f)]);
        float look = tight > 1.0f / 30.0f ? 4.0f + speed * 0.3f : 8.0f + speed * 0.45f;
        Vec3 aimPoint = centerAt(track, pose.s + look);
        int aimIndex = sampleAt(track, pose.s + look);
        if (std::fabs(curvature[aimIndex]) > 1.0f / 30.0f) {
            float outward = curvature[aimIndex] > 0 ? -1.5f : 1.5f;  // 右へ曲がるなら左（外）へ
            Vec3 right{std::cos(trackYawAt(track, pose.s + look)), 0, -std::sin(trackYawAt(track, pose.s + look))};
            aimPoint = aimPoint + right * outward;
        }
        float aimYaw = std::atan2(aimPoint.x - car.body.position.x, aimPoint.z - car.body.position.z);
        float want = limit[(index + 2) % n];

        // 先のコーナーの、曲がりが続く長さ。短い（20 m 未満）ものはドリフトで回らない。
        // 向きを変えてよい速さは、そのコーナーのいちばんきついところの速さ。
        float cornerLength = 0;
        float kickSpeed = want;
        if (cornerAhead >= 0) {
            for (int k = 0; k < n && std::fabs(curvature[(cornerAhead + k) % n]) > 1.0f / kDriftRadius; ++k) {
                cornerLength += 4.0f;
                kickSpeed = std::min(kickSpeed, limit[(cornerAhead + k) % n]);
            }
        }

        DriveInput input;
        if (!holding && cornerAhead >= 0 && cornerLength >= 20.0f && speed > 10.0f && speed < kickSpeed + 3.0f &&
            releaseWait <= 0) {
            holding = true;
            holdDir = curvature[cornerAhead] > 0 ? 1.0f : -1.0f;
            kickedOnce = false;
        }
        releaseWait = std::max(0.0f, releaseWait - kStep);
        if (holding) {
            input.handbrake = true;
            float angle = std::fabs(car.driftAngle);
            int ahead = sampleAt(track, pose.s + look);
            float bend = curvature[ahead];
            if (angle > 0.21f) {
                kickedOnce = true;
            }
            // 一度ドリフトした後、角度が戻ったところで逆向きのコーナーへ切り直すときは、いったん離して押し直す。
            // 押したままアクセルを抜いて逆へ倒すと、振り返しの操作になる。
            if (kickedOnce && angle < 0.21f && cornerAhead >= 0 && (curvature[cornerAhead] > 0 ? 1.0f : -1.0f) != holdDir) {
                holding = false;
                releaseWait = 0.15f;
            }
            // 切り返し: 先の曲がりが今の滑りと逆向きなら、いったん離して押し直す（振り返し）。
            // 右へ滑る（角度が正）のは左のコーナー（曲がりが負）。
            float side = car.driftAngle > 0 ? -1.0f : 1.0f;
            if (angle > 0.21f && std::fabs(bend) > 1.0f / kDriftRadius && bend * side < 0) {
                holding = false;
                releaseWait = 0.15f;
            }
            if (angle < 0.21f && !kickedOnce) {
                // 向きを変える: 曲がる向きへいっぱいに切る（サイドブレーキで後ろが流れる）。速すぎればブレーキも足す。
                // 一度角度が付いた後は、角度が浅くなっても切り直さず、曲がりの操作を続ける（いっぱいに切ると内側へ切れ込む）。
                float dir = cornerAhead >= 0 ? (curvature[cornerAhead] > 0 ? 1.0f : -1.0f) : (bend > 0 ? 1.0f : -1.0f);
                input.steer = dir;
                if (speed > want + 2.0f) {
                    input.brake = 0.45f;
                }
            } else {
                // 曲がり: 進む向きを狙う点へ向ける回る速さを、スティックで決める。倒しきったときの速さは、
                // driftCurveRate と、半径 driftMinRadius で回る速さの大きいほう（Car と同じ）。
                float travel = std::hypot(velocity.x, velocity.z);
                float error = wrapPi(aimYaw - travelYaw);
                float rate = bend * travel + 2.0f * error;
                float fullRate = std::max(car.driftCurveRate, travel / car.driftMinRadius);
                input.steer = clampf(rate / fullRate, -1.0f, 1.0f);
                // 角度はアクセルで決めるので、アクセルは保つ（緩めると角度が浅くなる）。速さはブレーキで抑える
                //（ブレーキではドリフトは切れない）。遅ければ踏み増す。
                input.throttle = clampf(0.5f + (want - speed) * 0.05f, 0.5f, 0.85f);
                if (speed > want + 2.0f) {
                    input.brake = clampf((speed - want - 2.0f) / 6.0f, 0.1f, 0.6f);
                }
            }
            // 出口: 先にドリフトの曲がりがなければ、アクセルを緩めて角度を戻し、車体がまっすぐに近づいてから離す。
            // 横を向いたまま離すと、戻ったグリップで振られる。
            if (cornerAhead < 0 && angle > 0.21f) {
                input.throttle = 0.15f;
                input.brake = 0;
            }
            if (cornerAhead < 0 && angle < 0.15f) {
                holding = false;
            }
            if (speed < 6.0f) {
                holding = false;
            }
        }
        if (backing > 0) {
            // 壁に止められたら、ハンドルを逆に切って下がる。
            backing -= kStep;
            holding = false;
            input = DriveInput{};
            input.brake = 1.0f;
            input.steer = -clampf(wrapPi(aimYaw - car.body.yaw) * 3.0f, -1.0f, 1.0f);
        } else if (!holding) {
            float aim = wrapPi(aimYaw - car.body.yaw);
            input.handbrake = false;
            input.steer = clampf(aim * 3.0f, -1.0f, 1.0f);
            if (speed > want + 0.5f) {
                input.brake = clampf((speed - want) / 3.0f, 0.2f, 1.0f);
            } else {
                input.throttle = clampf((want - speed) / 3.0f + 0.3f, 0.0f, 1.0f);
            }
        }
        rig.Step(input, track);

        // 記録
        if (prevS >= 0) {
            float ds = car.ground.s - prevS;
            if (ds < -track.length * 0.5f) ds += track.length;
            if (ds > track.length * 0.5f) ds -= track.length;
            if (ds > 0) travelled += ds;
        }
        prevS = car.ground.s;
        int corner = track.samples[index].corner;
        bool wall = wallDepth(rig, track) > -0.05f;
        if (wall) result.wallSteps++;
        if (wasDrifting && !car.drifting && std::fabs(car.driftAngle) > 1.4f) result.spins++;
        wasDrifting = car.drifting;
        if (corner > 0 && corner < 32 && std::fabs(curvature[index]) > 1.0f / kDriftRadius) {
            CornerLog& log = logs[corner];
            if (log.entrySpeed < 0) log.entrySpeed = speed;
            log.nameId = track.samples[index].nameId;
            log.steps++;
            cornerSteps++;
            if (car.drifting) {
                log.driftSteps++;
                cornerDrift++;
                log.sumAngle += std::fabs(car.driftAngle);
            }
            log.maxAngle = std::max(log.maxAngle, std::fabs(car.driftAngle));
            log.minSpeed = std::min(log.minSpeed, speed);
            log.maxLateral = std::max(log.maxLateral, std::fabs(pose.lateral));
            if (wall) log.wallSteps++;
        }
        stuck = speed < 1.0f && backing <= 0 ? stuck + kStep : 0.0f;
        if (stuck > 1.0f) {
            result.stops++;
            backing = 1.5f;
            stuck = 0;
        }
        if (result.stops > 10 || !std::isfinite(car.body.position.x)) break;
    }
    result.finished = travelled >= track.length;
    result.lapTime = step * kStep;
    result.driftShare = cornerSteps > 0 ? float(cornerDrift) / cornerSteps : 0.0f;
    if (verbose) {
        for (int c = 1; c < 32; ++c) {
            const CornerLog& log = logs[c];
            if (log.steps == 0) continue;
            std::printf("self-test touge drive T%-2d %-16s in %3.0f min %3.0f km/h drift %3.0f%% angle avg %2.0f max %2.0f deg lateral %.1f m wall %d\n",
                        c, track.nameOf(log.nameId), log.entrySpeed * 3.6f, log.minSpeed * 3.6f, 100.0f * log.driftSteps / log.steps,
                        log.driftSteps > 0 ? log.sumAngle / log.driftSteps * 57.3f : 0.0f, log.maxAngle * 57.3f,
                        log.maxLateral, log.wallSteps);
        }
    }
    std::printf("self-test touge drive lap %.1f s finished %d travelled %.0f m drift %.0f%% spins %d wall %d steps stops %d\n",
                result.lapTime, result.finished ? 1 : 0, travelled, result.driftShare * 100.0f, result.spins,
                result.wallSteps, result.stops);
    return result;
}

// 峠をドリフトで 1 周できること。今の車の設定のまま、サイドブレーキで向きを変えてアクセルとスティックで回る
// 走り方で、コーナーのほとんど（8 割以上）をドリフトで、スピンせず、壁にほとんど触れずに回れる。
struct GuardrailResult {
    float afterHit = 0;  // 当たってから 0.5 秒後の、コースに沿った速さ（km/h）
    float end = 0;       // 3 秒後の、コースに沿った速さ（km/h）
    bool stuck = false;  // 4 秒たっても走り出せていない（コースに沿って 10 km/h 未満）
    bool hit = false;    // 壁に当たったか（当たる前に向きが戻ることもある）
};

// 峠の山頂の直線で、kmh で右のガードレールへ angle 度の角度で当てる。アクセル半分で、ハンドルは中心線へ向ける。
GuardrailResult guardrailRun(const Track& track, float kmh, float angleDeg, float bodyDeg) {
    Rig rig(track);
    Car& car = rig.car;
    const float s0 = 40.0f;
    Vec3 start = centerAt(track, s0);
    float travel = trackYawAt(track, s0) + angleDeg / 57.3f;
    car.Place(start, travel + bodyDeg / 57.3f);
    car.body.velocity = Vec3{std::sin(travel), 0, std::cos(travel)} * (kmh / 3.6f);
    for (WheelCollider& w : car.wheels) {
        w.omega = kmh / 3.6f / w.radius;
    }
    GuardrailResult result;
    int hitStep = -1;
    for (int step = 0; step < 60 * 4; ++step) {
        TrackPose pose = track.queryLocal(car.body.position.x, car.body.position.z, car.Origin().y,
                                          car.ground.found ? car.ground.s : -1.0f);
        Vec3 aim = centerAt(track, pose.s + 15.0f);
        DriveInput input;
        input.throttle = 0.5f;
        // 当たるまではまっすぐ、当たった後は中心線へ戻す。
        if (hitStep >= 0) {
            input.steer = clampf(
                wrapPi(std::atan2(aim.x - car.body.position.x, aim.z - car.body.position.z) - car.body.yaw) * 2.0f, -1.0f, 1.0f);
        }
        rig.Step(input, track);
        float along = dot(car.body.velocity, Vec3{std::sin(pose.yaw), 0, std::cos(pose.yaw)}) * 3.6f;
        if (hitStep < 0 && wallDepth(rig, track) > -0.05f) {
            hitStep = step;
        }
        if (hitStep >= 0 && step == hitStep + 30) result.afterHit = along;
        result.end = along;
    }
    result.hit = hitStep >= 0;
    result.stuck = result.end < 10.0f;
    std::printf("self-test guardrail %3.0f km/h %2.0f deg body %+3.0f deg: hit %d after 0.5 s %3.0f km/h end %3.0f km/h stuck %d\n",
                kmh, angleDeg, bodyDeg, hitStep >= 0 ? 1 : 0, result.afterHit, result.end, result.stuck ? 1 : 0);
    return result;
}

// ガードレールに斜めに当たっても止まらず、沿って滑り、そのまま走り続けられること。
// 車体の向きは、進む向きのまま（0）、ドリフトのように壁と逆へ向いて横から当たる（−40°）、
// 頭から壁へ向いて当たる（+40°）の 3 つ。
int guardrailTest() {
    Track track;
    if (!BuildTougeCourse(track)) return 60;
    int failed = 0;
    // 進む向きのまま、またはドリフトのように横を向いて斜めに当たる。止まらず、角度に応じた速さ
    //（浅ければ 6 割、45° でも 3 割）でコースに沿って進み続ける。
    for (float body : {0.0f, -40.0f}) {
        for (float angle : {15.0f, 30.0f, 45.0f}) {
            GuardrailResult r = guardrailRun(track, 60.0f, angle, body);
            if (!r.hit) continue;
            float keep = angle < 20.0f ? 0.6f : (angle < 40.0f ? 0.45f : 0.3f);
            if ((r.stuck || r.afterHit < 60.0f * keep) && !failed) failed = 61;
        }
    }
    // 頭を壁へ向けて斜めに当たる。速さは落ちるが、壁に押し付けられたまま動けなくはならない。
    // （ドリフトで車体を斜めにして当たるのと同じ形なので、車体の向きは変えない）
    for (float angle : {15.0f, 30.0f}) {
        GuardrailResult r = guardrailRun(track, 60.0f, angle, 40.0f);
        if (r.hit && r.stuck && !failed) failed = 62;
    }
    // 遅い速さで頭から深く当たる。壁に沿う向きへ回され、走り出せる。
    for (float angle : {60.0f, 75.0f}) {
        GuardrailResult r = guardrailRun(track, 25.0f, angle, 0.0f);
        if (r.hit && r.stuck && !failed) failed = 63;
    }
    return failed;
}

struct AlignResult {
    float alignedAt = -1;    // 離してから、車体と道の向きの差が 5° 以内になった時間（秒）。届かなければ -1
    bool startedAligning = false;  // 離した直後に補正を始めたか
    float crossedAt = -1;    // 差の向きが逆になった（通り越した）時間（秒）
    bool alignedAfterCross = false;  // 通り越した後に、補正をしていたか
    float peakRate = 0;      // 離した後の、車体の回る速さの最大（rad/s）。大きいほど急に振られる
    float overshoot = 0;     // 通り越した側へ行き過ぎた角度の最大（rad）
    float releaseError = 0;  // 離したときの、車体と道の向きの差（rad）
};

// 峠の山頂の直線で、道の向きに進みながら車体を bodyDeg 傾けてドリフトし（Space を release ステップ押し、アクセル 0.8 で
// 角度を付ける）、離す。
// yawRate は離す前の車体の回る速さ（rad/s、通り越させるときに道の向きへ速く回す）。離した後はハンドルまっすぐ、
// アクセル 0.3 で 2 秒走る。
AlignResult exitAlignRun(const Track& track, float bodyDeg, float yawRate, float alignStrength, int release = 90) {
    Rig rig(track);
    Car& car = rig.car;
    car.driftExitAlign = alignStrength;
    const float s0 = 30.0f;
    const float road = trackYawAt(track, s0);
    car.Place(centerAt(track, s0), road + bodyDeg / 57.3f);
    car.body.velocity = Vec3{std::sin(road), 0, std::cos(road)} * (80.0f / 3.6f);
    for (WheelCollider& w : car.wheels) {
        w.omega = 80.0f / 3.6f / w.radius;
    }
    AlignResult result;
    float side = 0;
    for (int step = 0; step < release + 120; ++step) {
        if (step == release - 1) {
            car.body.yawRate = yawRate;
        }
        DriveInput input;
        input.handbrake = step < release;
        input.throttle = step < release ? 0.8f : 0.3f;
        rig.Step(input, track);
        if (step < release) continue;
        float error = wrapPi(road - car.body.yaw);  // 山頂の直線なので、道の向きは変わらない
        float t = (step - release + 1) * kStep;
        if (step == release) {
            result.startedAligning = car.ExitAligning();
            result.releaseError = std::fabs(error);
            side = error > 0 ? 1.0f : -1.0f;
        }
        result.peakRate = std::max(result.peakRate, std::fabs(car.body.yawRate));
        if (side != 0) result.overshoot = std::max(result.overshoot, -error * side);
        if (result.alignedAt < 0 && std::fabs(error) < 5.0f / 57.3f) result.alignedAt = t;
        if (result.crossedAt < 0 && error * side < 0) result.crossedAt = t;
        // 通り越したステップでは、車はまだ通り越したことを見ていない（次の更新で見る）。その次から確かめる。
        if (result.crossedAt >= 0 && t > result.crossedAt + kStep * 1.5f && car.ExitAligning()) result.alignedAfterCross = true;
    }
    return result;
}

// ドリフト後の向きの補正。Space を離すと補正を始め、急に振られずに（補正なしより穏やかに）車体が道の向きに戻ること。
// 道の向きを通り越したら（わざと通り越したときも）、補正をやめること。
int exitAlignTest() {
    Track track;
    if (!BuildTougeCourse(track)) return 65;
    // 浅い角度（車体 20°）と深い角度（車体 45°）で、0.15 秒ドリフトしてから離す。どちらも道の向きへ戻り、
    // 行き過ぎない。角度が深いほど、補正で速く回す。
    for (float sign : {-1.0f, 1.0f}) {
        const float body = 45.0f * sign;
        AlignResult shallow = exitAlignRun(track, 20.0f * sign, 0.0f, 1.0f, 9);
        AlignResult deep = exitAlignRun(track, body, 0.0f, 1.0f, 9);
        AlignResult without = exitAlignRun(track, body, 0.0f, 0.0f, 9);
        for (const AlignResult* r : {&shallow, &deep}) {
            std::printf("self-test exit align body %+3.0f deg: released at %.0f deg started %d aligned in %.2f s peak %.2f rad/s"
                        " overshoot %.1f deg\n",
                        body, r->releaseError * 57.3f, r->startedAligning ? 1 : 0, r->alignedAt, r->peakRate,
                        r->overshoot * 57.3f);
            if (!r->startedAligning || r->alignedAt < 0 || r->alignedAt > 0.8f) return 66;
            if (r->overshoot > 2.0f / 57.3f) return 67;
        }
        std::printf("self-test exit align body %+3.0f deg: without align aligned in %.2f s\n", body, without.alignedAt);
        if (deep.releaseError <= shallow.releaseError || deep.peakRate <= shallow.peakRate) return 69;
        if (without.alignedAt >= 0 && deep.alignedAt > without.alignedAt) return 70;
    }
    // 通り越し: 離す直前に、道の向きへ速く回しておく（わざと通り越すような動き）。通り越した後は補正しない。
    for (float body : {-35.0f, 35.0f}) {
        float rate = body > 0 ? -3.0f : 3.0f;  // 車体を道の向きへ回す向き
        AlignResult over = exitAlignRun(track, body, rate, 1.0f);
        std::printf("self-test exit align overshoot body %+3.0f deg: crossed at %.2f s aligning after cross %d\n", body,
                    over.crossedAt, over.alignedAfterCross ? 1 : 0);
        if (over.crossedAt < 0 || over.alignedAfterCross) return 68;
    }
    return 0;
}

// 峠の山頂の直線で、車体を 30° 傾けてドリフトし（0.15 秒）、Space を離してアクセル全開で 0.6 秒走る。
// その間にコースの進む向きの速さがどれだけ増えたか（m/s）。
float exitBoostRun(const Track& track, float boost) {
    Rig rig(track);
    Car& car = rig.car;
    car.driftExitBoost = boost;
    const float s0 = 30.0f;
    const float road = trackYawAt(track, s0);
    const Vec3 roadForward{std::sin(road), 0, std::cos(road)};
    car.Place(centerAt(track, s0), road + 30.0f / 57.3f);
    car.body.velocity = roadForward * (80.0f / 3.6f);
    for (WheelCollider& w : car.wheels) {
        w.omega = 80.0f / 3.6f / w.radius;
    }
    const int release = 9;
    float atRelease = 0;
    for (int step = 0; step < release + 36; ++step) {
        DriveInput input;
        input.handbrake = step < release;
        input.throttle = 1.0f;
        rig.Step(input, track);
        if (step == release - 1) atRelease = dot(car.body.velocity, roadForward);
    }
    return dot(car.body.velocity, roadForward) - atRelease;
}

// ドリフトを抜けたときの加速。Space を離してアクセルを踏むと、加速なしより速くコースの進む向きへ伸びること。
int exitBoostTest() {
    Track track;
    if (!BuildTougeCourse(track)) return 71;
    float with = exitBoostRun(track, 6.0f);
    float without = exitBoostRun(track, 0.0f);
    std::printf("self-test exit boost: speed gain in 0.6 s %.1f km/h (without %.1f km/h)\n", with * 3.6f, without * 3.6f);
    // 6 m/s² を 0.6 秒で、加速なしより 2.5 m/s（9 km/h）以上多く伸びる。
    if (with - without < 2.5f) return 72;
    return 0;
}

int tougeDriveTest() {
    Track track;
    if (!BuildTougeCourse(track)) return 50;
    TougeDriveResult r = tougeDriveRun(track, true);
    if (!r.finished || r.stops > 0) return 51;
    if (r.spins > 0) return 52;
    if (r.driftShare < 0.8f) return 53;
    if (r.wallSteps > 60) return 54;
    return 0;
}

// 走り比べのコース。形が正しく、ドリフトの自動運転でスピンせずに 1 周できること（止まって下がり直すのは 2 回まで）。
// 区間ごとのドリフトの割合と壁への接触は、どの形が走りやすいかの目安として表示する（区間によって差が出るのが目的なので、
// 割合は判定しない。自動運転が苦手な S 字では止まることもある）。
int labTest() {
    Track track;
    if (!BuildLabCourse(track)) return 55;
    if (int shape = courseShape(track, "lab")) return shape;
    TougeDriveResult r = tougeDriveRun(track, true);
    if (!r.finished || r.stops > 2) return 56;
    if (r.spins > 0) return 57;
    return 0;
}

// 音の合成。音の出る装置は使わず、SoundSynth が作る値だけを調べる。どの場面でも値が -1..1 の有限の数に収まり、
// 何も鳴らさなければ無音、BGM、エンジン音（アクセルで大きくなる）、スキール音、ラップの音がそれぞれ鳴ること。
int soundTest() {
    System::SoundSynth synth;
    synth.SetSampleRate(48000);
    struct Level {
        float rms = 0;
        float peak = 0;
        bool finite = true;
    };
    // sec 秒ぶんを 1024 フレームずつ作り、大きさを測る。
    auto render = [&synth](float sec) {
        constexpr int kFrames = 1024;
        float buffer[kFrames * 2];
        Level level;
        double sum = 0;
        long count = 0;
        for (int done = 0; done < int(sec * 48000.0f); done += kFrames) {
            synth.Render(buffer, kFrames, 2);
            for (float v : buffer) {
                level.finite = level.finite && std::isfinite(v);
                level.peak = std::max(level.peak, std::fabs(v));
                sum += double(v) * v;
                ++count;
            }
        }
        level.rms = float(std::sqrt(sum / double(std::max(1L, count))));
        return level;
    };
    bool ok = true;
    auto check = [&ok](const Level& level) {
        ok = ok && level.finite && level.peak <= 1.0f;
        return level;
    };

    Level silent = check(render(0.5f));
    synth.SetBgm(System::BgmTrack::Title);
    Level title = check(render(3.0f));
    synth.SetBgm(System::BgmTrack::Race);
    check(render(1.0f));  // 曲の切り替え
    Level race = check(render(3.0f));

    // 車の音だけを測る（BGM を 0 にする）。
    synth.musicVolume.store(0);
    System::CarSoundParams car;
    car.active = true;
    car.rpm = 1000;
    car.speed = 10;
    synth.SetCar(car);
    check(render(0.5f));
    Level idle = check(render(1.0f));
    car.rpm = 7000;
    car.throttle = 1;
    car.speed = 30;
    synth.SetCar(car);
    check(render(0.5f));
    Level full = check(render(1.0f));
    car.skid = 1;
    synth.SetCar(car);
    check(render(0.3f));
    Level skid = check(render(1.0f));

    // ラップの音だけを測る（車の音を止める）。
    synth.SetCar(System::CarSoundParams{});
    check(render(1.5f));
    synth.Play(System::SoundEffect::Lap);
    Level lap = check(render(0.5f));
    synth.Play(System::SoundEffect::BestLap);
    check(render(3.0f));
    Level after = check(render(0.5f));

    std::printf("self-test sound rms: silent %.4f title %.3f race %.3f idle %.3f full %.3f skid %.3f lap %.3f after %.4f\n",
                silent.rms, title.rms, race.rms, idle.rms, full.rms, skid.rms, lap.rms, after.rms);
    if (!ok) return 75;
    if (silent.peak > 1e-6f || after.rms > 1e-3f) return 76;
    if (title.rms < 0.01f || race.rms < 0.01f) return 77;
    if (idle.rms < 0.01f || full.rms < idle.rms * 1.3f) return 78;
    if (skid.rms < full.rms * 1.05f || lap.rms < 0.02f) return 79;
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
    result = driftBrakeTest();
    if (result != 0) {
        return result;
    }
    result = exitBounceTest();
    if (result != 0) {
        return result;
    }
    result = exitAlignTest();
    if (result != 0) {
        return result;
    }
    result = exitBoostTest();
    if (result != 0) {
        return result;
    }
    result = driftBoostTest();
    if (result != 0) {
        return result;
    }
    result = hairpinTest();
    if (result != 0) {
        return result;
    }
    result = driftSwitchTest();
    if (result != 0) {
        return result;
    }
    result = tougeTest();
    if (result != 0) {
        return result;
    }
    result = tougeDriveTest();
    if (result != 0) {
        return result;
    }
    result = guardrailTest();
    if (result != 0) {
        return result;
    }
    result = labTest();
    if (result != 0) {
        return result;
    }
    return soundTest();
}
