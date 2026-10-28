#include "Vehicle/AirControl.h"

#include "Physics/Rigidbody.h"
#include "car.hpp"

#include <cmath>

namespace {

constexpr float kYawAccel = 3.0f;     // ハンドルで出せる角加速度（rad/s²）
constexpr float kPitchAccel = 2.5f;   // アクセル・ブレーキで出せる角加速度（rad/s²）
constexpr float kAirDampingPerSec = 0.5f;  // 操作していない軸の角速度が 1 秒で残る割合
// 着地補助。路面の傾きとの差に比例して寄せ、回転の速さで止める。弱い補助なので上限も小さい。
constexpr float kAssistStiffness = 4.0f;   // 1/s²
constexpr float kAssistDamping = 2.0f;     // 1/s
constexpr float kAssistMaxAccel = 2.0f;    // rad/s²

float clampAccel(float value, float limit) {
    return value > limit ? limit : (value < -limit ? -limit : value);
}

}  // namespace

void AirControl::Update(Rigidbody& body, const DriveInput& input, bool airborne,
                        float groundDistance, float targetPitch, float targetRoll, float dt) {
    airTime_ = airborne ? airTime_ + dt : 0.0f;
    if (!enabled || airTime_ < kMinAirTime) {
        return;
    }
    // ハンドルはヨー、アクセルは鼻先を下げ、ブレーキは鼻先を上げる。
    float yawAccel = input.steer * kYawAccel * strength;
    float pitchAccel = (input.brake - input.throttle) * kPitchAccel * strength;
    float rollAccel = 0;

    if (landingAssist && groundDistance < kAssistHeight) {
        pitchAccel += clampAccel(kAssistStiffness * (targetPitch - body.pitch) - kAssistDamping * body.pitchRate,
                                 kAssistMaxAccel);
        rollAccel += clampAccel(kAssistStiffness * (targetRoll - body.roll) - kAssistDamping * body.rollRate,
                                kAssistMaxAccel);
    }
    body.AddTorque(yawAccel * body.inertiaYaw, pitchAccel * body.inertiaPitch, rollAccel * body.inertiaRoll, dt);

    // 操作していない軸は、空気の減衰だけで回転が弱まる。
    float keep = std::pow(kAirDampingPerSec, dt);
    if (input.steer == 0.0f) {
        body.yawRate *= keep;
    }
    if (input.throttle == 0.0f && input.brake == 0.0f) {
        body.pitchRate *= keep;
    }
    body.rollRate *= keep;
}
