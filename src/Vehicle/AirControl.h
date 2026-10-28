#pragma once

struct Rigidbody;
struct DriveInput;

// 空中姿勢制御。4 輪とも kMinAirTime 以上浮いている間だけ、操作で車体の向きを直せる。
// 物理ではなくゲーム用の補助なので、有効・無効と強さを設定で変えられる。
// トルクの上限は「角加速度 × 慣性モーメント」で決めるので、車の重さを変えても操作の感触は変わらない。
class AirControl {
public:
    bool enabled = true;
    float strength = 1.0f;       // 角加速度の上限に掛ける倍率
    bool landingAssist = true;   // 路面が近づいたら、車体を路面の傾きへ弱く寄せる

    // 浮いている時間と操作から、車体にトルクを加える。
    // groundDistance: 原点から真下の路面までの高さ（m）。targetPitch / targetRoll: 真下の路面の傾き（rad）。
    void Update(Rigidbody& body, const DriveInput& input, bool airborne,
                float groundDistance, float targetPitch, float targetRoll, float dt);
    void Reset() { airTime_ = 0; }
    float AirTime() const { return airTime_; }

    static constexpr float kMinAirTime = 0.1f;           // これより短い浮き（段差）では効かない
    static constexpr float kAssistHeight = 3.0f;         // 着地補助が効き始める高さ（m）

private:
    float airTime_ = 0;
};
