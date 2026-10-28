#include "SelfTest.h"

#include "car.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

int RunSelfTest(const Track& track) {
    Car car;
    car.reset(track);
    float travelled = 0;
    float prevS = -1;
    float maxLat = 0;
    for (int step = 0; step < 60 * 50; ++step) {
        TrackPose pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
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
        float errNow = wrapPi(pose.yaw - car.yaw);
        float errAhead = wrapPi(targetYaw - car.yaw);
        float bend = std::fabs(wrapPi(targetYaw - pose.yaw));
        DriveInput input;
        input.steer = clampf(errNow * 2.6f + errAhead * 0.5f, -1.0f, 1.0f);
        input.steer = clampf(input.steer - pose.lateral * 0.12f, -1.0f, 1.0f);
        input.throttle = bend < 0.15f ? 1.0f : 0.45f;
        input.brake = 0;
        if (bend > 0.28f && car.speed > 22.0f) {
            input.throttle = 0;
            input.brake = 1.0f;
        } else if (bend > 0.12f && car.speed > 34.0f) {
            input.throttle = 0;
            input.brake = 0.55f;
        }
        if (car.speed < 10.0f) {
            input.brake = 0;
            input.throttle = std::max(input.throttle, 0.5f);
        }
        car.update(1.0f / 60.0f, input, track);
        pose = track.query(car.x, car.z, car.y, car.ground.found ? car.ground.s : -1.0f);
        if (prevS >= 0) {
            float ds = pose.s - prevS;
            if (ds < -track.length * 0.5f) ds += track.length;
            if (ds > track.length * 0.5f) ds -= track.length;
            if (ds > 0) travelled += ds;
        }
        prevS = pose.s;
        maxLat = std::max(maxLat, pose.xzDist);
        if (!std::isfinite(car.x) || !std::isfinite(car.speed) || !std::isfinite(car.yaw)) {
            std::printf("self-test non-finite\n");
            return 1;
        }
    }
    std::printf("self-test length %.0f travelled %.0f maxLat %.1f speed %.1f km/h\n",
                track.length, travelled, maxLat, car.speed * 3.6f);
    if (track.length < 5200.0f || track.length > 6500.0f) return 2;
    if (travelled < 1200.0f) return 3;
    if (maxLat > 30.0f) return 4;
    return 0;
}
