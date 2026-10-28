#include "GameObject/HudObject.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "GameObject/RaceTimerObject.h"
#include "GameObject/TrackObject.h"
#include "Scene/Scene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer2D.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

void formatTime(char* buf, int size, float seconds) {
    if (seconds < 0) {
        std::snprintf(buf, size, "--:--.---");
        return;
    }
    int minutes = static_cast<int>(seconds / 60.0f);
    float rest = seconds - float(minutes) * 60.0f;
    std::snprintf(buf, size, "%d:%06.3f", minutes, rest);
}

}  // namespace

bool HudObject::Init() {
    car_ = scene_.Find<CarObject>();
    timer_ = scene_.Find<RaceTimerObject>();
    track_ = scene_.Find<TrackObject>();
    camera_ = scene_.Find<CameraObject>();
    return car_ && timer_ && track_ && camera_;
}

void HudObject::StartUpdate(float dt) {
    (void)dt;
    if (scene_.Ctx().input.Pressed(System::Action::Help)) {
        showHelp_ = !showHelp_;
    }
}

void HudObject::drawMinimap(System::Renderer2D& hud) {
    const Car& car = car_->GetCar();
    const Track& track = track_->GetTrack();
    float size = 190.0f;
    float x0 = 24.0f;
    float y0 = float(hud.Height()) - size - 28.0f;
    hud.AddQuad(x0, y0, x0 + size, y0 + size, 0.02f, 0.03f, 0.05f, 0.78f);
    float minX = track.samples[0].x;
    float maxX = minX;
    float minZ = track.samples[0].z;
    float maxZ = minZ;
    for (const TrackSample& sample : track.samples) {
        minX = std::min(minX, sample.x);
        maxX = std::max(maxX, sample.x);
        minZ = std::min(minZ, sample.z);
        maxZ = std::max(maxZ, sample.z);
    }
    float midX = (minX + maxX) * 0.5f;
    float midZ = (minZ + maxZ) * 0.5f;
    float span = std::max(maxX - minX, maxZ - minZ) * 1.12f;
    auto mapX = [&](float x) { return x0 + size * 0.5f + (x - midX) / span * (size - 18.0f); };
    auto mapY = [&](float z) { return y0 + size * 0.5f - (z - midZ) / span * (size - 18.0f); };
    const int n = static_cast<int>(track.samples.size());
    for (int i = 0; i < n; ++i) {
        const TrackSample& a = track.samples[i];
        const TrackSample& b = track.samples[(i + 1) % n];
        float r = 0.75f, g = 0.78f, bl = 0.82f;
        if (a.bridge || b.bridge) {
            r = 0.95f;
            g = 0.72f;
            bl = 0.28f;
        }
        hud.DrawLine(mapX(a.x), mapY(a.z), mapX(b.x), mapY(b.z), 2.0f, r, g, bl, 1);
    }
    float mx = mapX(car.body.position.x);
    float my = mapY(car.body.position.z);
    Vec3 nose{std::sin(car.body.yaw), 0, std::cos(car.body.yaw)};
    Vec3 side{std::cos(car.body.yaw), 0, -std::sin(car.body.yaw)};
    float nx = mx + nose.x * 9.0f;
    float ny = my - nose.z * 9.0f;
    float lx = mx - nose.x * 5.0f + side.x * 5.0f;
    float ly = my + nose.z * 5.0f - side.z * 5.0f;
    float rx = mx - nose.x * 5.0f - side.x * 5.0f;
    float ry = my + nose.z * 5.0f + side.z * 5.0f;
    hud.DrawLine(lx, ly, nx, ny, 2.0f, 0.9f, 0.2f, 0.18f, 1);
    hud.DrawLine(rx, ry, nx, ny, 2.0f, 0.9f, 0.2f, 0.18f, 1);
    hud.DrawLine(lx, ly, rx, ry, 2.0f, 0.9f, 0.2f, 0.18f, 1);
}

void HudObject::Draw2D(System::Renderer2D& hud) {
    const Car& car = car_->GetCar();
    const Race& race = timer_->GetRace();
    const Track& track = track_->GetTrack();
    char line[96];
    int kmh = static_cast<int>(std::lround(car.ForwardSpeed() * 3.6f));
    std::snprintf(line, sizeof(line), "%d KM/H", kmh);
    hud.DrawText(24, 22, 4.0f, line, 0.95f, 0.96f, 0.98f, 1);

    if (!race.timing) {
        std::snprintf(line, sizeof(line), "OUT LAP");
    } else {
        std::snprintf(line, sizeof(line), "LAP %d", race.completed + 1);
    }
    hud.DrawText(24, 64, 2.2f, line, 0.85f, 0.88f, 0.92f, 1);
    formatTime(line, sizeof(line), race.lap);
    hud.DrawText(150, 64, 2.2f, line, 0.95f, 0.96f, 0.98f, 1);

    char best[32];
    formatTime(best, sizeof(best), race.best);
    std::snprintf(line, sizeof(line), "BEST %s", best);
    hud.DrawText(24, 90, 2.0f, line, 0.75f, 0.8f, 0.7f, 1);

    int percent = static_cast<int>(car.ground.s / track.length * 100.0f);
    const char* corner = track.nameOf(car.ground.nameId);
    if (car.ground.corner > 0 && corner[0]) {
        std::snprintf(line, sizeof(line), "T%d %s  %d%%", car.ground.corner, corner, percent);
    } else if (corner[0]) {
        std::snprintf(line, sizeof(line), "%s  %d%%", corner, percent);
    } else {
        std::snprintf(line, sizeof(line), "%d%%", percent);
    }
    hud.DrawText(24, 116, 2.0f, line, 0.9f, 0.9f, 0.92f, 1);

    const char* camName = "CHASE";
    if (camera_->Mode() == 1) camName = "CLOSE";
    if (camera_->Mode() == 2) camName = "FAR";
    hud.DrawText(float(hud.Width()) - 140, 22, 2.0f, camName, 0.8f, 0.84f, 0.88f, 1);

    if (showHelp_) {
        hud.DrawText(24, float(hud.Height()) - 78, 1.7f,
                     "W/UP ACCEL  S/DOWN BRAKE  A/D STEER  R RESET  C CAMERA  P PAUSE",
                     0.9f, 0.92f, 0.94f, 1);
        hud.DrawText(24, float(hud.Height()) - 54, 1.7f,
                     "PAD: RT ACCEL  LT BRAKE  STICK STEER  Y CAMERA  BACK RESET  START PAUSE",
                     0.75f, 0.8f, 0.84f, 1);
    }
    hud.DrawText(float(hud.Width()) - 250, float(hud.Height()) - 28, 1.6f, "(C) OSM / GSI", 0.85f, 0.88f, 0.9f, 1);
    if (scene_.Paused()) {
        hud.DrawText(float(hud.Width()) * 0.5f - 70, float(hud.Height()) * 0.45f, 5.0f, "PAUSE", 1, 0.95f, 0.9f, 1);
    }
    drawMinimap(hud);
}
