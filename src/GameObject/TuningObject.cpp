#include "GameObject/TuningObject.h"

#include "GameObject/CarObject.h"
#include "Physics/PhysicsMaterial.h"
#include "Scene/Scene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer2D.h"
#include "Vehicle/VehicleSpec.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace {

constexpr float kRadToDeg = 57.2958f;
constexpr const char* kExportPath = "tuning.txt";
constexpr float kMessageSeconds = 2.5f;
constexpr int kVisibleRows = 14;  // 一覧に一度に出す項目の数

}  // namespace

bool TuningObject::Init() {
    car_ = scene_.Find<CarObject>();
    if (!car_) {
        return false;
    }
    buildBasicItems();
    buildDetailItems();
    return true;
}

void TuningObject::buildBasicItems() {
    Car& car = car_->MutableCar();
    // 摩擦係数は舗装路での値（路面の μ × タイヤの倍率）で出す。ダートや芝では路面の μ の比で下がる。
    const float roadMu = Materials::kAsphalt.friction;
    // 後輪は、今使っているほう（グリップ用かドリフト用）の値を変える。
    auto rear = [&car]() -> TireParams& {
        return car.RearDriftTires() ? car.rearDriftTire.params : car.rearGripTire.params;
    };
    auto forEachWheel = [&car](const std::function<void(WheelCollider&)>& apply) {
        for (WheelCollider& w : car.wheels) {
            apply(w);
        }
    };
    auto add = [this](const char* category, Item item) {
        item.category = category;
        basicItems_.push_back(item);
    };

    // 車体
    add("BODY", {"MASS KG", "mass", 800, 2000, 20,
                 [&car] { return car.mass; },
                 [&car](float v) { car.SetMass(v); }});
    add("BODY", {"ROTATION INERTIA X", "inertia_scale", 0.5f, 2.0f, 0.05f,
                 [&car] { return car.inertiaScale; },
                 [&car](float v) { car.SetInertiaScale(v); }});
    add("BODY", {"CENTER OF MASS M", "center_of_mass_y", 0.25f, 0.80f, 0.01f,
                 [&car] { return car.centerOfMassY; },
                 [&car](float v) { car.SetCenterOfMassY(v); }});
    // 足の硬さ。減衰は減衰比が変わらないように √(ばね) に、スタビライザーはばねに比例させる。
    add("BODY", {"SPRING N/M", "spring", 10000, 80000, 1000,
                 [&car] { return car.wheels[Car::kFrontLeft].springRate; },
                 [&car](float v) {
                     float ratio = v / VehicleSpec::kSpringRate;
                     for (int i = 0; i < Car::kWheelCount; ++i) {
                         WheelCollider& w = car.wheels[i];
                         w.springRate = v;
                         w.damping = VehicleSpec::kDamping * std::sqrt(ratio);
                         w.antiRollRate = (i < Car::kRearLeft ? VehicleSpec::kFrontAntiRoll : VehicleSpec::kRearAntiRoll) * ratio;
                     }
                 }});
    add("BODY", {"ENGINE POWER KW", "engine_power_kw", 100, 600, 10,
                 [&car] { return car.engine.torqueScale * EnginePowertrain::PeakPowerKw(); },
                 [&car](float v) { car.engine.torqueScale = v / EnginePowertrain::PeakPowerKw(); }});
    add("BODY", {"BRAKE M/S2", "brake_decel", 5, 40, 1,
                 [&car] { return car.brakeDecel; },
                 [&car](float v) { car.brakeDecel = v; }});
    // タイヤ。前輪を基準に、後輪は前後の配分を掛けた値にする。
    add("TIRES", {"STATIC FRICTION", "static_friction", 0.5f, 3.0f, 0.05f,
                  [&car, roadMu] { return car.frontTire.params.muScale * roadMu; },
                  [&car, rear, roadMu](float v) {
                      // 前後の配分と動摩擦の値はそのまま保つ（動摩擦が静止摩擦を超えるときは同じ値まで下げる）。
                      TireParams& front = car.frontTire.params;
                      float balance = rear().muScale / front.muScale;
                      float sliding = front.slideRatio * front.muScale * roadMu;
                      front.muScale = v / roadMu;
                      rear().muScale = front.muScale * balance;
                      front.slideRatio = std::min(1.0f, sliding / v);
                      rear().slideRatio = front.slideRatio;
                  }});
    // 動摩擦は静止摩擦との比（滑りきったときの力 / ピークの力）として持つので、静止摩擦を超えられない。
    add("TIRES", {"SLIDING FRICTION", "sliding_friction", 0.2f, 3.0f, 0.05f,
                  [&car, roadMu] { return car.frontTire.params.slideRatio * car.frontTire.params.muScale * roadMu; },
                  [&car, rear, roadMu](float v) {
                      float ratio = clampf(v / (car.frontTire.params.muScale * roadMu), 0.1f, 1.0f);
                      car.frontTire.params.slideRatio = ratio;
                      rear().slideRatio = ratio;
                  }});
    add("TIRES", {"REAR/FRONT GRIP", "grip_balance", 0.6f, 1.4f, 0.02f,
                  [&car, rear] { return rear().muScale / car.frontTire.params.muScale; },
                  [&car, rear](float v) { rear().muScale = car.frontTire.params.muScale * v; }});
    // 運転の補助（ABS、トラクションコントロール、空中の姿勢の補助、着地の補助）をまとめて切り替える。
    add("ASSISTS", {"DRIVING ASSISTS", "assists", 0, 1, 1,
                    [&car] { return car.wheels[0].antiLock ? 1.0f : 0.0f; },
                    [&car, forEachWheel](float v) {
                        bool on = v > 0.5f;
                        forEachWheel([on](WheelCollider& w) {
                            w.antiLock = on;
                            w.tractionControl = on;
                        });
                        car.airControl.enabled = on;
                        car.airControl.landingAssist = on;
                    }, "OFF", "ON"});
}

void TuningObject::buildDetailItems() {
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

    detailItems_ = {
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
    for (size_t i = 0; i < detailItems_.size(); ++i) {
        detailItems_[i].category = i < 6 ? "TIRES" : "ASSISTS";
    }

    auto add = [this](const char* category, Item item) {
        item.category = category;
        detailItems_.push_back(item);
    };
    // 前輪と後輪のどちらかの 2 本に同じ値を入れる。
    auto axle = [&car](bool front, const std::function<void(WheelCollider&)>& apply) {
        int first = front ? Car::kFrontLeft : Car::kRearLeft;
        apply(car.wheels[first]);
        apply(car.wheels[first + 1]);
    };

    // 足回り
    add("SUSPENSION", {"FRONT SPRING N/M", "front_spring", 10000, 80000, 1000,
                       [&car] { return car.wheels[Car::kFrontLeft].springRate; },
                       [axle](float v) { axle(true, [v](WheelCollider& w) { w.springRate = v; }); }});
    add("SUSPENSION", {"REAR SPRING N/M", "rear_spring", 10000, 80000, 1000,
                       [&car] { return car.wheels[Car::kRearLeft].springRate; },
                       [axle](float v) { axle(false, [v](WheelCollider& w) { w.springRate = v; }); }});
    add("SUSPENSION", {"DAMPING N S/M", "damping", 500, 6000, 100,
                       [&car] { return car.wheels[0].damping; },
                       [forEachWheel](float v) { forEachWheel([v](WheelCollider& w) { w.damping = v; }); }});
    add("SUSPENSION", {"FRONT ANTI ROLL N/M", "front_anti_roll", 0, 40000, 1000,
                       [&car] { return car.wheels[Car::kFrontLeft].antiRollRate; },
                       [axle](float v) { axle(true, [v](WheelCollider& w) { w.antiRollRate = v; }); }});
    add("SUSPENSION", {"REAR ANTI ROLL N/M", "rear_anti_roll", 0, 40000, 1000,
                       [&car] { return car.wheels[Car::kRearLeft].antiRollRate; },
                       [axle](float v) { axle(false, [v](WheelCollider& w) { w.antiRollRate = v; }); }});
    add("SUSPENSION", {"CENTER OF MASS M", "center_of_mass_y", 0.25f, 0.80f, 0.01f,
                       [&car] { return car.centerOfMassY; },
                       [&car](float v) { car.SetCenterOfMassY(v); }});
    // 駆動
    add("DRIVETRAIN", {"POWER UNIT", "motor", 0, 1, 1,
                       [&car] { return car.UsingMotor() ? 1.0f : 0.0f; },
                       [&car](float v) { car.UseMotor(v > 0.5f); }, "ENGINE", "MOTOR"});
    add("DRIVETRAIN", {"ENGINE TORQUE X", "engine_torque_scale", 0.5f, 2.5f, 0.05f,
                       [&car] { return car.engine.torqueScale; },
                       [&car](float v) { car.engine.torqueScale = v; }});
    add("DRIVETRAIN", {"FINAL DRIVE", "final_drive", 2.5f, 6.0f, 0.1f,
                       [&car] { return car.engine.finalDrive; },
                       [&car](float v) { car.engine.finalDrive = v; }});
    add("DRIVETRAIN", {"UPSHIFT RPM", "upshift_rpm", 4000, 8000, 100,
                       [&car] { return car.engine.upshiftRpm; },
                       [&car](float v) { car.engine.upshiftRpm = v; }});
    add("DRIVETRAIN", {"DOWNSHIFT RPM", "downshift_rpm", 1500, 5000, 100,
                       [&car] { return car.engine.downshiftRpm; },
                       [&car](float v) { car.engine.downshiftRpm = v; }});
    // ブレーキとハンドル
    add("BRAKE AND STEERING", {"BRAKE M/S2", "brake_decel", 5, 40, 1,
                               [&car] { return car.brakeDecel; },
                               [&car](float v) { car.brakeDecel = v; }});
    add("BRAKE AND STEERING", {"BRAKE FRONT SHARE", "brake_front_share", 0.3f, 0.8f, 0.05f,
                               [&car] { return car.brakeFrontShare; },
                               [&car](float v) { car.brakeFrontShare = v; }});
    add("BRAKE AND STEERING", {"MAX STEER DEG", "max_steer_deg", 10, 45, 1,
                               [&car] { return car.maxSteer * kRadToDeg; },
                               [&car](float v) { car.maxSteer = v / kRadToDeg; }});
    add("BRAKE AND STEERING", {"STEER SPEED FALLOFF", "steer_speed_falloff", 0, 0.2f, 0.005f,
                               [&car] { return car.steerSpeedFalloff; },
                               [&car](float v) { car.steerSpeedFalloff = v; }});
    add("BRAKE AND STEERING", {"STEER RATE RAD/S", "steer_rate", 0.5f, 6.0f, 0.1f,
                               [&car] { return car.steerRate; },
                               [&car](float v) { car.steerRate = v; }});
    // 車体
    add("BODY", {"MASS KG", "mass", 800, 2000, 20,
                 [&car] { return car.mass; },
                 [&car](float v) { car.SetMass(v); }});
    add("BODY", {"DRAG X10000", "drag_x10000", 0, 10, 0.1f,
                 [&car] { return car.dragPerMass * 10000.0f; },
                 [&car](float v) { car.SetDrag(v / 10000.0f); }});
}

std::string TuningObject::valueText(const Item& item) const {
    float value = item.get();
    if (item.offText) {
        return value > 0.5f ? item.onText : item.offText;
    }
    char text[32];
    // 刻みに合わせた桁で出す（バネや回転数は整数、刻み 0.005 は 3 桁）。
    const char* format = item.step < 0.01f ? "%.3f" : (item.step < 0.1f ? "%.2f" : (item.step < 1.0f ? "%.1f" : "%.0f"));
    std::snprintf(text, sizeof(text), format, value);
    return text;
}

void TuningObject::StartUpdate(float dt) {
    messageTime_ = std::max(0.0f, messageTime_ - dt);
    System::Input& input = scene_.Ctx().input;
    if (input.Pressed(System::Action::MenuToggle)) {
        open_ = !open_;
        input.SetMenuMode(open_);
    }
    if (!open_ || items().empty()) {
        return;
    }
    if (input.Pressed(System::Action::MenuDetail)) {
        detail_ = !detail_;
        selected_ = 0;
        firstVisible_ = 0;
    }
    const int count = static_cast<int>(items().size());
    // 押した回数だけ動かす。フレームの間に何度も押しても取りこぼさない。
    int move = input.PressCount(System::Action::MenuDown) - input.PressCount(System::Action::MenuUp);
    selected_ = ((selected_ + move) % count + count) % count;
    // 選んでいる項目が見えるように、一覧をスクロールする。
    if (selected_ < firstVisible_) {
        firstVisible_ = selected_;
    } else if (selected_ >= firstVisible_ + kVisibleRows) {
        firstVisible_ = selected_ - kVisibleRows + 1;
    }
    int direction = input.PressCount(System::Action::MenuRight) - input.PressCount(System::Action::MenuLeft);
    if (direction != 0) {
        Item& item = items()[selected_];
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
    // 基本の項目と細かい項目の両方を書く（基本の項目が細かい項目を動かすので、どちらも今の車の値）。
    std::ofstream out(kExportPath);
    for (const std::vector<Item>* list : {&basicItems_, &detailItems_}) {
        for (const Item& item : *list) {
            char line[96];
            std::snprintf(line, sizeof(line), "%s %.4f", item.key, item.get());
            out << line << '\n';
            SDL_Log("tuning %s", line);
        }
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
    const int visibleRows = std::min<int>(kVisibleRows, static_cast<int>(items().size()));
    const float height = rowHeight * (float(visibleRows) + 2.0f) + 12.0f;
    if (messageTime_ > 0) {
        hud.DrawText(x0, y0 + (open_ ? height + 8 : 0), 2.0f, message_.c_str(), 0.95f, 0.85f, 0.4f, 1);
    }
    if (!open_) {
        return;
    }
    hud.AddQuad(x0, y0, x0 + width, y0 + height, 0.02f, 0.03f, 0.05f, 0.82f);
    // 見出し: 選んでいる項目の分類と、何番目か。下に操作の説明。
    char title[96];
    std::snprintf(title, sizeof(title), "TUNING %s  %s  %d/%d", detail_ ? "DETAIL" : "BASIC", items()[selected_].category, selected_ + 1,
                  static_cast<int>(items().size()));
    hud.DrawText(x0 + 12, y0 + 8, 1.8f, title, 0.95f, 0.8f, 0.35f, 1);
    hud.DrawText(x0 + 12, y0 + 8 + rowHeight * float(visibleRows + 1) + 6, 1.6f,
                 detail_ ? "UP/DOWN  LEFT/RIGHT  F BASIC  E SAVE  TAB CLOSE" : "UP/DOWN  LEFT/RIGHT  F DETAIL  E SAVE  TAB CLOSE",
                 0.75f, 0.8f, 0.84f, 1);
    for (int row = 0; row < visibleRows; ++row) {
        size_t i = static_cast<size_t>(firstVisible_ + row);
        float y = y0 + 8 + rowHeight * float(row + 1) + 6;
        bool selected = static_cast<int>(i) == selected_;
        if (selected) {
            hud.AddQuad(x0 + 6, y - 4, x0 + width - 6, y + rowHeight - 6, 0.25f, 0.32f, 0.45f, 0.9f);
        }
        float shade = selected ? 1.0f : 0.82f;
        hud.DrawText(x0 + 14, y, 1.8f, items()[i].label, shade, shade, shade, 1);
        hud.DrawText(x0 + 380, y, 1.8f, valueText(items()[i]).c_str(), shade, selected ? 0.9f : 0.82f, selected ? 0.5f : 0.82f, 1);
    }
}

void TuningObject::UnInit() {
    // シーンを離れるときは、矢印キーを運転に戻す。
    scene_.Ctx().input.SetMenuMode(false);
}
