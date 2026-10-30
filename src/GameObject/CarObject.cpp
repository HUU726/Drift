#include "GameObject/CarObject.h"

#include "GameObject/TrackObject.h"
#include "Scene/Scene.h"
#include "System/ColliderManager.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/SoundManager.h"
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

void CarObject::OnCollision(const Contact& contact) {
    // 力積を質量で割ると、当たりで変わった速さ（m/s）になる。こすった程度（0.8 m/s 未満）では鳴らさない。
    float change = contact.impulse / car_.body.mass;
    if (change < 0.8f) {
        return;
    }
    scene_.Ctx().sound.PlaySe(System::SoundEffect::Crash, clampf((change - 0.8f) / 7.0f, 0.0f, 1.0f));
}

void CarObject::PostPhysicsUpdate(float dt) {
    (void)dt;
    // 車体が 75° ほど傾いたら（上向きの y が 0.25 未満）横転とする。ほぼ起き上がるまで（リセットを含む）次は鳴らさない。
    Vec3 right, up, forward;
    car_.body.Axes(right, up, forward);
    if (!rolled_ && up.y < 0.25f) {
        rolled_ = true;
        float speed = length(car_.body.velocity);
        scene_.Ctx().sound.PlaySe(System::SoundEffect::Rollover, clampf(speed / 20.0f, 0.0f, 1.0f));
    } else if (rolled_ && up.y > 0.8f) {
        rolled_ = false;
    }
}

void CarObject::LateUpdate(float dt) {
    (void)dt;
    // ポーズ中は音を止める（車が止まって見えるのに、エンジンとタイヤが鳴り続けないように）。
    System::CarSoundParams sound;
    sound.active = !scene_.Paused();
    sound.rpm = car_.Powertrain().Rpm();
    sound.throttle = scene_.Ctx().input.Drive().throttle;
    sound.speed = length(car_.body.velocity);
    // スキール音は、ドリフト中に地面に着いているときだけ。滑る角度が深いほど、速いほど大きくする。
    if (car_.drifting && !car_.Airborne()) {
        float angle = clampf(std::fabs(car_.driftAngle) / 0.6f, 0.35f, 1.0f);
        float fast = clampf((sound.speed - 3.0f) / 12.0f, 0.0f, 1.0f);
        sound.skid = angle * fast;
    }
    scene_.Ctx().sound.SetCar(sound);
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
    scene_.Ctx().sound.StopCar();
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
