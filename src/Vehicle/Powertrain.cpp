#include "Vehicle/Powertrain.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kRadPerSecToRpm = 60.0f / (2.0f * 3.14159265f);

// エンジン
constexpr float kIdleRpm = 1000.0f;
constexpr float kRedlineRpm = 8000.0f;
constexpr float kShiftTime = 0.15f;      // 変速中に駆動を切る時間（秒）
constexpr float kEngineBrake = 0.006f;    // アクセル 0 のときの負のトルク（N·m / rpm、エンジン側）
constexpr float kGearRatios[] = {3.20f, 2.20f, 1.60f, 1.25f, 1.00f, 0.82f};
constexpr int kGearCount = 6;
constexpr float kReverseRatio = 3.0f;
constexpr float kEfficiency = 0.85f;

struct TorquePoint {
    float rpm;
    float torque;
};
constexpr TorquePoint kTorqueCurve[] = {
    {1000.0f, 200.0f}, {3000.0f, 330.0f}, {5500.0f, 400.0f}, {7000.0f, 370.0f}, {8000.0f, 300.0f},
};

// モーター
constexpr float kMotorTorque = 450.0f;     // N·m（モーター側）
constexpr float kMotorPower = 230000.0f;   // W。エンジンの最大出力と同じくらい
constexpr float kMotorRatio = 9.0f;        // 減速比（デフ込み）
constexpr float kMotorMaxRpm = 16000.0f;

}  // namespace

float EnginePowertrain::EngineTorque(float rpm) {
    if (rpm > kRedlineRpm) {
        return 0.0f;  // レッドゾーンより上ではトルクを出さない（レブリミッター）
    }
    const int n = static_cast<int>(sizeof(kTorqueCurve) / sizeof(kTorqueCurve[0]));
    if (rpm <= kTorqueCurve[0].rpm) {
        return kTorqueCurve[0].torque;
    }
    for (int i = 1; i < n; ++i) {
        if (rpm <= kTorqueCurve[i].rpm) {
            const TorquePoint& a = kTorqueCurve[i - 1];
            const TorquePoint& b = kTorqueCurve[i];
            float t = (rpm - a.rpm) / (b.rpm - a.rpm);
            return a.torque + (b.torque - a.torque) * t;
        }
    }
    return kTorqueCurve[n - 1].torque;
}

float EnginePowertrain::ratio() const {
    float gear = gear_ < 0 ? -kReverseRatio : kGearRatios[gear_ - 1];
    return gear * finalDrive;
}

void EnginePowertrain::Reset() {
    gear_ = 1;
    rpm_ = kIdleRpm;
    shiftTimer_ = 0;
}

float EnginePowertrain::Update(float throttle, bool reverse, float wheelOmega, float dt) {
    // 後退ギアと前進ギアの切り替えは、止まっているときに Car が決める。
    if (reverse && gear_ > 0) {
        gear_ = -1;
    } else if (!reverse && gear_ < 0) {
        gear_ = 1;
    }
    // 駆動輪の回転から逆算した回転数。クラッチがつながっていればエンジンはこの回転で回る。
    float wheelRpm = wheelOmega * ratio() * kRadPerSecToRpm;

    // 自動変速。変速した直後は回転数がギア比の分だけ変わるので、上げと下げが往復しない。
    shiftTimer_ = std::max(0.0f, shiftTimer_ - dt);
    if (gear_ > 0 && shiftTimer_ <= 0) {
        if (wheelRpm > upshiftRpm && gear_ < kGearCount) {
            ++gear_;
            shiftTimer_ = kShiftTime;
        } else if (wheelRpm < downshiftRpm && gear_ > 1) {
            --gear_;
            shiftTimer_ = kShiftTime;
        }
        wheelRpm = wheelOmega * ratio() * kRadPerSecToRpm;
    }
    rpm_ = std::max(kIdleRpm, wheelRpm);
    if (shiftTimer_ > 0) {
        return 0.0f;
    }
    if (throttle > 0.0f) {
        return EngineTorque(rpm_) * torqueScale * throttle * ratio() * kEfficiency;
    }
    // アクセルを離すとエンジンブレーキ。クラッチがつながる回転（アイドル以上）のときだけ効き、
    // 止まっている車を後ろへ押すことはない。
    if (wheelRpm <= kIdleRpm) {
        return 0.0f;
    }
    return -kEngineBrake * wheelRpm * ratio();
}

float MotorPowertrain::Update(float throttle, bool reverse, float wheelOmega, float dt) {
    (void)dt;
    reverse_ = reverse;
    float motorOmega = std::fabs(wheelOmega) * kMotorRatio;
    rpm_ = motorOmega * kRadPerSecToRpm;
    if (rpm_ >= kMotorMaxRpm) {
        return 0.0f;
    }
    // 低い回転では一定のトルク、出力が上限に達してからは出力一定（トルクは回転に反比例）。
    float torque = kMotorTorque;
    if (motorOmega > 1.0f) {
        torque = std::min(torque, kMotorPower / motorOmega);
    }
    float wheelTorque = torque * throttle * kMotorRatio * kEfficiency;
    return reverse ? -wheelTorque : wheelTorque;
}
