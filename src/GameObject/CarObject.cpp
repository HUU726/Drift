#include "GameObject/CarObject.h"

#include "GameObject/TrackObject.h"
#include "Scene/Scene.h"
#include "System/ColliderManager.h"
#include "System/Context.h"
#include "System/Input.h"
#include "Vehicle/VehicleSpec.h"

#include <cmath>

bool CarObject::Init() {
    // コースがあればそのスポーン地点に、なければ SetSpawn で決めた位置に置く。
    TrackObject* track = scene_.Find<TrackObject>();
    track_ = track ? &track->GetTrack() : nullptr;
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
    System::ColliderManager& physics = scene_.Ctx().physics;
    car_.groundProbe = [&physics](float x, float z, float yHint, float preferS) {
        return physics.GroundAt(x, z, yHint, preferS);
    };
    Reset();

    physics.Register(&car_.body);
    for (WheelCollider& w : car_.wheels) {
        w.owner = this;
        physics.Register(&w);
    }
    box_.owner = this;
    box_.body = &car_.body;
    physics.Register(&box_);
    return true;
}

void CarObject::Reset() {
    if (track_) {
        car_.reset(*track_);
    } else {
        car_.Place(spawn_, spawnYaw_);
    }
}

void CarObject::FixedUpdate(float dt) {
    // 重心の高さを変えたときに、当たり判定の箱の中心（重心から見た高さ）を合わせる。
    box_.offset.y = VehicleSpec::kBodyBoxCenterY - car_.centerOfMassY;
    car_.update(dt, scene_.Ctx().input.Drive(), track_);
}

void CarObject::Draw3D(System::Renderer3D& renderer) {
    // 影は車の真下の路面に置く。車体が浮いても路面に残る。
    Vec3 origin = car_.Origin();
    RaycastHit under = scene_.Ctx().physics.GroundAt(origin.x, origin.z, origin.y + 0.5f, car_.ground.found ? car_.ground.s : -1.0f);
    float groundY = under.hit ? under.point.y : origin.y;
    Mat4 shadow = mul(translation(origin.x, groundY + 0.08f, origin.z), rotationY(car_.body.yaw));
    renderer.Draw(shadow_, shadow);

    Mat4 model = carModelMatrix(car_);
    renderer.Draw(body_, model);
    for (int i = 0; i < Car::kWheelCount; ++i) {
        Vec3 center = car_.WheelCenter(i);
        float steer = i < Car::kRearLeft ? car_.steerAngle : 0.0f;
        Mat4 place = mul(translation(center.x, center.y, center.z), mul(rotationY(steer), rotationX(car_.wheels[i].spinAngle)));
        renderer.Draw(wheel_, mul(model, place));
    }
}

void CarObject::UnInit() {
    System::ColliderManager& physics = scene_.Ctx().physics;
    physics.Unregister(&box_);
    for (WheelCollider& w : car_.wheels) {
        physics.Unregister(&w);
    }
    physics.Unregister(&car_.body);
    System::Renderer3D& renderer = scene_.Ctx().renderer3D;
    renderer.Release(body_);
    renderer.Release(wheel_);
    renderer.Release(shadow_);
}
