#include "GameObject/TuningObject.h"

#include "GameObject/CarObject.h"
#include "Scene/Scene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer2D.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace {

constexpr float kRadToDeg = 57.2958f;
constexpr const char* kExportPath = "tuning.txt";
constexpr float kMessageSeconds = 2.5f;

}  // namespace

bool TuningObject::Init() {
    car_ = scene_.Find<CarObject>();
    if (!car_) {
        return false;
    }
    buildItems();
    return true;
}

void TuningObject::buildItems() {
    Car& car = car_->MutableCar();
    // 後輪は、今使っているほう（グリップ用かドリフト用）の値を変える。
    auto rear = [&car]() -> TireParams& {
        return car.RearDriftTires() ? car.rearDriftTire.params : car.rearGripTire.params;
    };
    auto forEachWheel = [&car](const std::function<void(WheelCollider&)>& apply) {
        for (WheelCollider& w : car.wheels) {
            apply(w);
        }
    };

    items_ = {
        // タイヤ
        {"FRONT GRIP X", "front_mu_scale", 0.5f, 3.0f, 0.05f,
         [&car] { return car.frontTire.params.muScale; },
         [&car](float v) { car.frontTire.params.muScale = v; }},
        {"FRONT PEAK ANGLE DEG", "front_peak_angle_deg", 2.0f, 20.0f, 0.5f,
         [&car] { return car.frontTire.params.peakSlipAngle * kRadToDeg; },
         [&car](float v) { car.frontTire.params.peakSlipAngle = v / kRadToDeg; }},
        {"REAR TIRE", "rear_drift_tire", 0.0f, 1.0f, 1.0f,
         [&car] { return car.RearDriftTires() ? 1.0f : 0.0f; },
         [&car](float v) { car.UseDriftTires(v > 0.5f); }, "GRIP", "DRIFT"},
        {"REAR GRIP X", "rear_mu_scale", 0.5f, 3.0f, 0.05f,
         [rear] { return rear().muScale; },
         [rear](float v) { rear().muScale = v; }},
        {"REAR PEAK ANGLE DEG", "rear_peak_angle_deg", 2.0f, 20.0f, 0.5f,
         [rear] { return rear().peakSlipAngle * kRadToDeg; },
         [rear](float v) { rear().peakSlipAngle = v / kRadToDeg; }},
        {"REAR SLIDE RATIO", "rear_slide_ratio", 0.3f, 1.0f, 0.05f,
         [rear] { return rear().slideRatio; },
         [rear](float v) { rear().slideRatio = v; }},
        // 運転の補助
        {"ABS", "anti_lock", 0.0f, 1.0f, 1.0f,
         [&car] { return car.wheels[0].antiLock ? 1.0f : 0.0f; },
         [forEachWheel](float v) { forEachWheel([v](WheelCollider& w) { w.antiLock = v > 0.5f; }); }, "OFF", "ON"},
        {"TRACTION CONTROL", "traction_control", 0.0f, 1.0f, 1.0f,
         [&car] { return car.wheels[0].tractionControl ? 1.0f : 0.0f; },
         [forEachWheel](float v) { forEachWheel([v](WheelCollider& w) { w.tractionControl = v > 0.5f; }); }, "OFF", "ON"},
        {"SLIP LIMIT", "slip_control_share", 0.3f, 1.0f, 0.05f,
         [&car] { return car.wheels[0].slipControlShare; },
         [forEachWheel](float v) { forEachWheel([v](WheelCollider& w) { w.slipControlShare = v; }); }},
        {"AIR CONTROL", "air_control", 0.0f, 1.0f, 1.0f,
         [&car] { return car.airControl.enabled ? 1.0f : 0.0f; },
         [&car](float v) { car.airControl.enabled = v > 0.5f; }, "OFF", "ON"},
        {"AIR STRENGTH", "air_control_strength", 0.0f, 3.0f, 0.1f,
         [&car] { return car.airControl.strength; },
         [&car](float v) { car.airControl.strength = v; }},
        {"LANDING ASSIST", "landing_assist", 0.0f, 1.0f, 1.0f,
         [&car] { return car.airControl.landingAssist ? 1.0f : 0.0f; },
         [&car](float v) { car.airControl.landingAssist = v > 0.5f; }, "OFF", "ON"},
    };
}

std::string TuningObject::valueText(const Item& item) const {
    float value = item.get();
    if (item.offText) {
        return value > 0.5f ? item.onText : item.offText;
    }
    char text[32];
    std::snprintf(text, sizeof(text), item.step < 0.1f ? "%.2f" : "%.1f", value);
    return text;
}

void TuningObject::StartUpdate(float dt) {
    messageTime_ = std::max(0.0f, messageTime_ - dt);
    System::Input& input = scene_.Ctx().input;
    if (input.Pressed(System::Action::MenuToggle)) {
        open_ = !open_;
        input.SetMenuMode(open_);
    }
    if (!open_ || items_.empty()) {
        return;
    }
    const int count = static_cast<int>(items_.size());
    // 押した回数だけ動かす。フレームの間に何度も押しても取りこぼさない。
    int move = input.PressCount(System::Action::MenuDown) - input.PressCount(System::Action::MenuUp);
    selected_ = ((selected_ + move) % count + count) % count;
    int direction = input.PressCount(System::Action::MenuRight) - input.PressCount(System::Action::MenuLeft);
    if (direction != 0) {
        Item& item = items_[selected_];
        float value = item.get() + item.step * float(direction);
        // 刻みの倍数に丸めて、足し算の誤差がたまらないようにする。
        value = std::round(value / item.step) * item.step;
        item.set(clampf(value, item.min, item.max));
    }
    if (input.Pressed(System::Action::Export)) {
        exportValues();
    }
}

void TuningObject::exportValues() {
    // ブラウザ版ではファイルがページの中に閉じるので、同じ内容をログ（ブラウザのコンソール）にも出す。
    std::ofstream out(kExportPath);
    for (const Item& item : items_) {
        char line[96];
        std::snprintf(line, sizeof(line), "%s %.4f", item.key, item.get());
        out << line << '\n';
        SDL_Log("tuning %s", line);
    }
    message_ = out ? "SAVED TUNING.TXT" : "SAVE FAILED";
    messageTime_ = kMessageSeconds;
}

void TuningObject::Draw2D(System::Renderer2D& hud) {
    // 計器の下、画面の左に出す。ウィンドウが狭くても値の列が切れない。
    const float rowHeight = 22.0f;
    const float width = 520.0f;
    const float x0 = 24.0f;
    const float y0 = 246.0f;
    const float height = rowHeight * (float(items_.size()) + 2.0f) + 12.0f;
    if (messageTime_ > 0) {
        hud.DrawText(x0, y0 + (open_ ? height + 8 : 0), 2.0f, message_.c_str(), 0.95f, 0.85f, 0.4f, 1);
    }
    if (!open_) {
        return;
    }
    hud.AddQuad(x0, y0, x0 + width, y0 + height, 0.02f, 0.03f, 0.05f, 0.82f);
    hud.DrawText(x0 + 12, y0 + 8, 1.8f, "TUNING  UP/DOWN SELECT  LEFT/RIGHT CHANGE  E SAVE", 0.95f, 0.8f, 0.35f, 1);
    for (size_t i = 0; i < items_.size(); ++i) {
        float y = y0 + 8 + rowHeight * float(i + 1) + 6;
        bool selected = static_cast<int>(i) == selected_;
        if (selected) {
            hud.AddQuad(x0 + 6, y - 4, x0 + width - 6, y + rowHeight - 6, 0.25f, 0.32f, 0.45f, 0.9f);
        }
        float shade = selected ? 1.0f : 0.82f;
        hud.DrawText(x0 + 14, y, 1.8f, items_[i].label, shade, shade, shade, 1);
        hud.DrawText(x0 + 380, y, 1.8f, valueText(items_[i]).c_str(), shade, selected ? 0.9f : 0.82f, selected ? 0.5f : 0.82f, 1);
    }
}

void TuningObject::UnInit() {
    // シーンを離れるときは、矢印キーを運転に戻す。
    scene_.Ctx().input.SetMenuMode(false);
}
