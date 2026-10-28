#include "GameObject/CameraObject.h"

#include "GameObject/CarObject.h"
#include "GameObject/TrackObject.h"
#include "Scene/Scene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer3D.h"

#include <cmath>

bool CameraObject::Init() {
    car_ = scene_.Find<CarObject>();
    track_ = scene_.Find<TrackObject>();
    return car_ && track_;
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
    Vec3 forward{std::sin(car.body.yaw), 0, std::cos(car.body.yaw)};
    Vec3 eye = Vec3{car.body.position.x, car.body.position.y, car.body.position.z} - forward * back[mode_] + Vec3{0, up[mode_], 0};
    Vec3 look = Vec3{car.body.position.x, car.body.position.y, car.body.position.z} + forward * ahead[mode_] + Vec3{0, 1.15f, 0};
    TrackPose under = track_->GetTrack().queryLocal(eye.x, eye.z, car.body.position.y, car.ground.s);
    if (under.found && under.xzDist < 30.0f && eye.y < under.centerY + 0.8f) {
        eye.y = under.centerY + 0.8f;
    }
    if (!ready_) {
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
