#include "GameObject/TelemetryHudObject.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "Scene/Scene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer2D.h"

#include <cmath>
#include <cstdio>

bool TelemetryHudObject::Init() {
    car_ = scene_.Find<CarObject>();
    camera_ = scene_.Find<CameraObject>();
    return car_ && camera_;
}

void TelemetryHudObject::StartUpdate(float dt) {
    (void)dt;
    if (scene_.Ctx().input.Pressed(System::Action::Help)) {
        showHelp_ = !showHelp_;
    }
}

void TelemetryHudObject::Draw2D(System::Renderer2D& hud) {
    const Car& car = car_->GetCar();
    const Rigidbody& body = car.body;
    char line[128];

    std::snprintf(line, sizeof(line), "%d KM/H", static_cast<int>(std::lround(car.ForwardSpeed() * 3.6f)));
    hud.DrawText(24, 22, 4.0f, line, 0.95f, 0.96f, 0.98f, 1);
    const IPowertrain& powertrain = car.Powertrain();
    if (powertrain.Gear() < 0) {
        std::snprintf(line, sizeof(line), "R  %d RPM", static_cast<int>(powertrain.Rpm()));
    } else {
        std::snprintf(line, sizeof(line), "%d  %d RPM", powertrain.Gear(), static_cast<int>(powertrain.Rpm()));
    }
    hud.DrawText(250, 34, 2.2f, line, 0.85f, 0.88f, 0.92f, 1);
    hud.DrawText(24, 64, 2.2f, "TEST COURSE", 0.95f, 0.8f, 0.35f, 1);

    // 車体: 横滑りの速さ、ヨーレート、横の加速度（ヨーレート × 速度）、pitch と roll、空中の時間
    float lateralG = body.yawRate * car.ForwardSpeed() / 9.81f;
    std::snprintf(line, sizeof(line), "SLIDE %.1f M/S  YAW %.2f RAD/S  LAT %.2f G", body.LateralSpeed(), body.yawRate, lateralG);
    hud.DrawText(24, 92, 1.8f, line, 0.9f, 0.9f, 0.92f, 1);
    std::snprintf(line, sizeof(line), "PITCH %.3f  ROLL %.3f  AIR %.2f S", body.pitch, body.roll, car.airControl.AirTime());
    hud.DrawText(24, 114, 1.8f, line, 0.9f, 0.9f, 0.92f, 1);

    // 各輪: 荷重、縮み、スリップ率、スリップ角（度）、路面。滑っている輪は赤く出す。
    const char* names[Car::kWheelCount] = {"FL", "FR", "RL", "RR"};
    for (int i = 0; i < Car::kWheelCount; ++i) {
        const WheelCollider& w = car.wheels[i];
        const char* surface = !w.grounded ? "AIR" : (w.surface ? w.surface->name : "-");
        std::snprintf(line, sizeof(line), "%s %5.0f N  %.3f M  SR %6.3f  SA %5.1f  %s", names[i], w.load, w.compression,
                      w.slipRatio, w.slipAngle * 57.2958f, surface);
        float red = w.sliding ? 1.0f : 0.85f;
        float other = w.sliding ? 0.35f : 0.88f;
        hud.DrawText(24, 146 + 22.0f * i, 1.8f, line, red, other, other, 1);
    }

    const char* camName = camera_->Mode() == 1 ? "CLOSE" : (camera_->Mode() == 2 ? "FAR" : "CHASE");
    hud.DrawText(float(hud.Width()) - 140, 22, 2.0f, camName, 0.8f, 0.84f, 0.88f, 1);
    if (showHelp_) {
        hud.DrawText(24, float(hud.Height()) - 78, 1.7f,
                     "AHEAD ASPHALT  RIGHT DIRT  LEFT RAMPS 5/10/15/20  FAR AHEAD S-CURVES",
                     0.9f, 0.92f, 0.94f, 1);
        hud.DrawText(24, float(hud.Height()) - 54, 1.7f,
                     "T RACE  R RESET  C CAMERA  P PAUSE  H HELP",
                     0.75f, 0.8f, 0.84f, 1);
    }
    if (scene_.Paused()) {
        hud.DrawText(float(hud.Width()) * 0.5f - 70, float(hud.Height()) * 0.45f, 5.0f, "PAUSE", 1, 0.95f, 0.9f, 1);
    }
}
