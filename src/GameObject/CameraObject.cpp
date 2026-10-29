#include "GameObject/CameraObject.h"

#include "GameObject/CarObject.h"
#include "Scene/Scene.h"
#include "System/ColliderManager.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer3D.h"

#include <cmath>

namespace {

// ドリフト中に、車の向きから進む向きへどれだけ寄せるか（0 で車の向き、1 で進む向き）。
constexpr float kTravelShare = 0.7f;
// その割合を変える速さ（1/s）。小さいほど、ドリフトの後にゆっくり車の後ろへ戻る。
constexpr float kTravelShareRate = 1.5f;

}  // namespace

bool CameraObject::Init() {
    car_ = scene_.Find<CarObject>();
    return car_ != nullptr;
}

void CameraObject::StartUpdate(float dt) {
    (void)dt;
    if (scene_.Ctx().input.Pressed(System::Action::Camera)) {
        mode_ = (mode_ + 1) % 3;
    }
}

void CameraObject::LateUpdate(float dt) {
    const Car& car = car_->GetCar();
    const float back[3] = {8.6f, 4.1f, 16.0f};
    const float up[3] = {3.3f, 1.55f, 7.2f};
    const float ahead[3] = {9.0f, 6.0f, 14.0f};
    // ドリフト中は、車の向きと進む向きの間から見る。車の向きだけで追うと、横を向いた車体が画面の正面に来て
    // 進む先が見えず、進む向きだけで追うと、車体の角度が分からない。
    // 寄せる割合は、ドリフトに入るときと抜けるときにゆっくり変える。すぐに 0 へ戻すと、横を向いていた車体の向きへ
    // カメラが一気に回る。割合を残しておけば、車体がまっすぐに戻るにつれて角度も 0 へ近づき、なめらかに車の後ろへ戻る。
    float shareTarget = car.drifting ? kTravelShare : 0.0f;
    travelShare_ += (shareTarget - travelShare_) * (1.0f - std::exp(-dt * kTravelShareRate));
    float yaw = car.body.yaw + car.driftAngle * travelShare_;
    Vec3 forward{std::sin(yaw), 0, std::cos(yaw)};
    Vec3 origin = car.Origin();
    Vec3 eye = origin - forward * back[mode_] + Vec3{0, up[mode_], 0};
    Vec3 look = origin + forward * ahead[mode_] + Vec3{0, 1.15f, 0};
    // カメラが路面に潜らないように、真下の路面より 0.8 m 上に保つ。立体交差では車のいる高さの路面を選ぶ。
    RaycastHit under = scene_.Ctx().physics.GroundAt(eye.x, eye.z, origin.y + 2.0f, car.ground.found ? car.ground.s : -1.0f);
    if (under.hit && eye.y < under.point.y + 0.8f) {
        eye.y = under.point.y + 0.8f;
    }
    if (!ready_) {
        travelShare_ = shareTarget;
        eye_ = eye;
        look_ = look;
        ready_ = true;
        return;
    }
    float blend = 1.0f - std::exp(-dt * 7.0f);
    eye_ = eye_ + (eye - eye_) * blend;
    look_ = look_ + (look - look_) * blend;
}

Mat4 CameraObject::ViewProj() const {
    const System::Renderer3D& renderer = scene_.Ctx().renderer3D;
    float aspect = float(renderer.Width()) / float(renderer.Height());
    Mat4 proj = perspective(60.0f * 3.14159265f / 180.0f, aspect, 0.15f, 4000.0f);
    Mat4 view = lookAt(eye_, look_, {0, 1, 0});
    return mul(proj, view);
}
