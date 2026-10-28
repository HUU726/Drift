#include "GameObject/CarObject.h"

#include "GameObject/TrackObject.h"
#include "Scene/Scene.h"
#include "System/Context.h"
#include "System/Input.h"

#include <cmath>

bool CarObject::Init() {
    TrackObject* track = scene_.Find<TrackObject>();
    if (!track) {
        return false;
    }
    track_ = &track->GetTrack();
    System::Renderer3D& renderer = scene_.Ctx().renderer3D;
    CpuMesh body;
    CpuMesh wheel;
    CpuMesh shadow;
    buildCarBody(body);
    buildCarWheel(wheel);
    buildCarShadow(shadow);
    body_ = renderer.Upload(body);
    wheel_ = renderer.Upload(wheel);
    shadow_ = renderer.Upload(shadow);
    car_.reset(*track_);
    return true;
}

void CarObject::Reset() {
    car_.reset(*track_);
}

void CarObject::FixedUpdate(float dt) {
    car_.update(dt, scene_.Ctx().input.Drive(), *track_);
}

void CarObject::LateUpdate(float dt) {
    (void)dt;
    pitch_ = computePitch();
    roll_ = std::atan(car_.ground.cross);
}

float CarObject::computePitch() const {
    Vec3 forward{std::sin(car_.body.yaw), 0, std::cos(car_.body.yaw)};
    float preferS = car_.ground.found ? car_.ground.s : -1.0f;
    TrackPose ahead = track_->query(car_.body.position.x + forward.x * 2.4f, car_.body.position.z + forward.z * 2.4f, car_.body.position.y, preferS);
    TrackPose behind = track_->query(car_.body.position.x - forward.x * 2.4f, car_.body.position.z - forward.z * 2.4f, car_.body.position.y, preferS);
    if (!ahead.found || !behind.found) {
        return 0;
    }
    return std::atan2(ahead.centerY - behind.centerY, 4.8f);
}

void CarObject::Draw3D(System::Renderer3D& renderer) {
    Mat4 shadow = mul(translation(car_.body.position.x, car_.body.position.y + 0.08f, car_.body.position.z), rotationY(car_.body.yaw));
    renderer.Draw(shadow_, shadow);

    Mat4 model = carModelMatrix(car_, pitch_, roll_);
    renderer.Draw(body_, model);
    const float wheelX = 0.82f;
    const float wheelZ[2] = {1.42f, -1.38f};
    const float steerForWheel[2] = {car_.steerAngle, 0};
    for (int i = 0; i < 2; ++i) {
        for (int side = -1; side <= 1; side += 2) {
            Mat4 place = mul(translation(float(side) * wheelX, 0.34f, wheelZ[i]),
                             mul(rotationY(steerForWheel[i]), rotationX(car_.wheelSpin)));
            renderer.Draw(wheel_, mul(model, place));
        }
    }
}

void CarObject::UnInit() {
    System::Renderer3D& renderer = scene_.Ctx().renderer3D;
    renderer.Release(body_);
    renderer.Release(wheel_);
    renderer.Release(shadow_);
}
