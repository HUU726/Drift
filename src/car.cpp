#include "car.hpp"

#include "Vehicle/VehicleSpec.h"

#include <algorithm>
#include <cmath>

namespace {

// 駆動は IPowertrain が決める。ブレーキは運転操作から車全体の減速（brakeDecel、m/s²）として求める。
// 実際に出る加速と減速は、タイヤが路面に伝えられる力（μ × 荷重）で決まる。
constexpr float kMaxReverse = 9.0f;     // 後退の最高速度（m/s）。ここで駆動を切る

}  // namespace

Car::Car() {
    using namespace VehicleSpec;
    const float x[kWheelCount] = {-kWheelX, kWheelX, -kWheelX, kWheelX};
    const float z[kWheelCount] = {kFrontWheelZ, kFrontWheelZ, kRearWheelZ, kRearWheelZ};
    for (int i = 0; i < kWheelCount; ++i) {
        WheelCollider& wheel = wheels[i];
        wheel.body = &body;
        wheel.mount = Vec3{x[i], kMountY - centerOfMassY, z[i]};
        wheel.radius = kWheelRadius;
        wheel.restLength = kRestLength;
        wheel.springRate = kSpringRate;
        wheel.damping = kDamping;
        wheel.bumpDampingScale = kBumpDampingScale;
        wheel.reboundDampingScale = kReboundDampingScale;
        wheel.bumpStop = kBumpStop;
        wheel.bumpStopRate = kBumpStopRate;
        wheel.bumpStopReturn = kBumpStopReturn;
        wheel.antiRollRate = i < kRearLeft ? kFrontAntiRoll : kRearAntiRoll;
        wheel.tire = i < kRearLeft ? static_cast<const ITireModel*>(&frontTire) : &rearGripTire;
        wheel.wheelInertia = kWheelInertia;
        wheel.rollingDecel = kRollingDecel;
        wheel.antiLock = kAntiLockBrakes;
        wheel.tractionControl = kTractionControl;
        wheel.slipControlShare = kSlipControlShare;
    }
    airControl.enabled = VehicleSpec::kAirControl;
    airControl.strength = VehicleSpec::kAirControlStrength;
    airControl.landingAssist = VehicleSpec::kLandingAssist;
    wheels[kFrontLeft].antiRollPartner = &wheels[kFrontRight];
    wheels[kFrontRight].antiRollPartner = &wheels[kFrontLeft];
    wheels[kRearLeft].antiRollPartner = &wheels[kRearRight];
    wheels[kRearRight].antiRollPartner = &wheels[kRearLeft];
}

void Car::UseDriftTires(bool drift) {
    const ITireModel* rear = drift ? static_cast<const ITireModel*>(&rearDriftTire) : &rearGripTire;
    wheels[kRearLeft].tire = rear;
    wheels[kRearRight].tire = rear;
}

void Car::SetMass(float newMass) {
    float ratio = newMass / mass;
    mass = newMass;
    body.mass = newMass;
    body.inertiaYaw *= ratio;
    body.inertiaPitch *= ratio;
    body.inertiaRoll *= ratio;
    body.dragCoefficient = mass * dragPerMass;
}

void Car::SetDrag(float perMass) {
    dragPerMass = perMass;
    body.dragCoefficient = mass * dragPerMass;
}

void Car::SetCenterOfMassY(float height) {
    // 原点（地面）の位置は変えずに、重心と、重心から見た取付点を動かす。
    float delta = height - centerOfMassY;
    centerOfMassY = height;
    body.position.y += delta;
    for (WheelCollider& wheel : wheels) {
        wheel.mount.y = VehicleSpec::kMountY - height;
    }
}

void Car::UseMotor(bool useMotor) {
    powertrain = useMotor ? static_cast<IPowertrain*>(&motor) : &engine;
    powertrain->Reset();
}

void Car::reset(const Track& track) {
    Place(Vec3{track.spawnX, track.spawnY, track.spawnZ}, track.spawnYaw);
    ground = track.query(track.spawnX, track.spawnZ, track.spawnY);
}

void Car::Place(Vec3 origin, float yaw) {
    body = Rigidbody{};
    body.position = origin;
    body.yaw = yaw;
    // 慣性モーメントは、質量を変えたときに同じ形の箱として比例させる。
    float massRatio = mass / VehicleSpec::kMass;
    body.mass = mass;
    body.inertiaYaw = VehicleSpec::kInertiaYaw * massRatio;
    body.inertiaPitch = VehicleSpec::kInertiaPitch * massRatio;
    body.inertiaRoll = VehicleSpec::kInertiaRoll * massRatio;
    body.dragCoefficient = mass * dragPerMass;
    // 原点を地面に置く。サスペンションが止まっているときの縮みになる高さで、すぐに落ち着く。
    body.position.y += centerOfMassY;
    for (WheelCollider& wheel : wheels) {
        wheel.Reset();
    }
    steerAngle = 0;
    brakeHold = 0;
    powertrain->Reset();
    airControl.Reset();
    ground = TrackPose{};
}

bool Car::Airborne() const {
    for (const WheelCollider& wheel : wheels) {
        if (wheel.grounded) {
            return false;
        }
    }
    return true;
}

Vec3 Car::Origin() const {
    return body.ToWorld(Vec3{0, -centerOfMassY, 0});
}

Vec3 Car::WheelCenter(int wheel) const {
    const WheelCollider& w = wheels[wheel];
    return Vec3{w.mount.x, w.mount.y + centerOfMassY - w.length, w.mount.z};
}

void Car::update(float dt, const DriveInput& input, const Track* track) {
    float speed = body.ForwardSpeed();
    // 最大の切れ角は、速度が上がるほど小さくする（高速で切りすぎない）。
    float maxSteerNow = maxSteer / (1.0f + std::fabs(speed) * steerSpeedFalloff);
    float target = clampf(input.steer, -1.0f, 1.0f) * maxSteerNow;
    float step = steerRate * dt;
    steerAngle += clampf(target - steerAngle, -step, step);
    steerAngle = clampf(steerAngle, -maxSteer, maxSteer);

    // 止まってブレーキを押し続けると後退に入る。後退に入った後は、速度が出てもブレーキを離すまで後退を続ける。
    bool holdingBrake = input.brake > 0.5f && input.throttle < 0.1f;
    bool stopped = std::fabs(speed) < 0.45f;
    bool backingUp = brakeHold > 0.28f && speed < 0;
    if (holdingBrake && (stopped || backingUp)) {
        brakeHold += dt;
    } else {
        brakeHold = 0;
    }
    bool reversing = brakeHold > 0.28f;

    // 駆動系に渡すアクセル開度と、ブレーキの減速（m/s²）。後退中はブレーキの踏み込みをアクセルとして使う。
    float throttle = 0;
    float brake = 0;
    if (input.brake > 0.05f && !reversing) {
        brake = input.brake * brakeDecel;
    } else if (reversing) {
        throttle = speed > -kMaxReverse ? input.brake : 0.0f;
    } else if (input.throttle > 0.0f && speed > -0.4f) {
        throttle = input.throttle;
    } else if (input.throttle > 0.0f) {
        brake = input.throttle * brakeDecel;  // 後ろへ動いている間のアクセルはブレーキとして効く
    }
    float rearOmega = 0.5f * (wheels[kRearLeft].omega + wheels[kRearRight].omega);
    float axleTorque = powertrain->Update(throttle, reversing, rearOmega, dt);

    // 駆動は後輪の左右に均等（オープンデフ）、ブレーキは前 6 : 後 4。
    const float radius = VehicleSpec::kWheelRadius;
    for (int i = 0; i < kWheelCount; ++i) {
        WheelCollider& wheel = wheels[i];
        bool front = i < kRearLeft;
        wheel.steer = front ? steerAngle : 0.0f;
        float driveTorque = front ? 0.0f : axleTorque * 0.5f;
        float brakeTorque = mass * brake * radius * 0.5f * (front ? brakeFrontShare : 1.0f - brakeFrontShare);
        wheel.driveTorque = driveTorque;
        wheel.brakeTorque = brakeTorque;
    }

    // 車の下のコース。周回位置と横位置を、ラップ計測と HUD のために持っておく。
    float preferS = ground.found ? ground.s : -1.0f;
    Vec3 origin = Origin();
    if (track) {
        TrackPose pose = track->queryLocal(origin.x, origin.z, origin.y, preferS);
        if (pose.found) {
            ground = pose;
        }
    }

    // 空中姿勢制御。着地補助のために、真下の路面までの高さと路面の傾き（前後と左右）を渡す。
    // 傾きは、原点の前後 2.4 m と左右 1 m の路面の高さの差から求める。
    bool airborne = Airborne();
    float groundDistance = 1.0e9f;
    float targetPitch = 0;
    float targetRoll = 0;
    if (airborne && groundProbe) {
        RaycastHit under = groundProbe(origin.x, origin.z, origin.y, preferS);
        if (under.hit) {
            groundDistance = origin.y - under.point.y;
            Vec3 forward = body.Forward();
            Vec3 right = body.Right();
            float probeY = origin.y;
            RaycastHit ahead = groundProbe(origin.x + forward.x * 2.4f, origin.z + forward.z * 2.4f, probeY, preferS);
            RaycastHit behind = groundProbe(origin.x - forward.x * 2.4f, origin.z - forward.z * 2.4f, probeY, preferS);
            RaycastHit toRight = groundProbe(origin.x + right.x, origin.z + right.z, probeY, preferS);
            RaycastHit toLeft = groundProbe(origin.x - right.x, origin.z - right.z, probeY, preferS);
            if (ahead.hit && behind.hit) {
                targetPitch = std::atan2(ahead.point.y - behind.point.y, 4.8f);
            }
            if (toRight.hit && toLeft.hit) {
                targetRoll = std::atan2(toRight.point.y - toLeft.point.y, 2.0f);
            }
        }
    }
    airControl.Update(body, input, airborne, groundDistance, targetPitch, targetRoll, dt);
}

void buildCarBody(CpuMesh& mesh) {
    const Vec3 red{0.72f, 0.08f, 0.09f};
    const Vec3 dark{0.06f, 0.06f, 0.07f};
    const Vec3 carbon{0.12f, 0.12f, 0.13f};
    mesh.addBox({-0.88f, 0.32f, -2.05f}, {0.88f, 0.62f, 2.05f}, red);
    mesh.addBox({-0.72f, 0.62f, -0.55f}, {0.72f, 1.05f, 1.05f}, red);
    mesh.addBox({-0.62f, 0.78f, -0.35f}, {0.62f, 1.02f, 0.9f}, dark);
    mesh.addBox({-0.7f, 0.32f, 1.7f}, {0.7f, 0.5f, 2.25f}, red);
    mesh.addBox({-0.95f, 0.26f, 1.95f}, {0.95f, 0.36f, 2.35f}, carbon);
    mesh.addBox({-0.86f, 0.85f, -2.2f}, {0.86f, 1.0f, -1.85f}, carbon);
    mesh.addBox({-0.15f, 0.95f, -2.05f}, {0.15f, 1.15f, -1.9f}, carbon);
}

void buildCarWheel(CpuMesh& mesh) {
    const int seg = 14;
    const float radius = 0.34f;
    const float halfW = 0.15f;
    const Vec3 tire{0.07f, 0.07f, 0.08f};
    const Vec3 rim{0.72f, 0.74f, 0.76f};
    for (int i = 0; i < seg; ++i) {
        float a0 = float(i) / float(seg) * 6.2831853f;
        float a1 = float(i + 1) / float(seg) * 6.2831853f;
        float c0 = std::cos(a0);
        float s0 = std::sin(a0);
        float c1 = std::cos(a1);
        float s1 = std::sin(a1);
        Vec3 normal = normalize(Vec3{0, c0 + c1, s0 + s1});
        Vec3 a{-halfW, c0 * radius, s0 * radius};
        Vec3 b{halfW, c0 * radius, s0 * radius};
        Vec3 c{halfW, c1 * radius, s1 * radius};
        Vec3 d{-halfW, c1 * radius, s1 * radius};
        mesh.addQuad(a, b, c, d, normal, tire);
        for (int side = -1; side <= 1; side += 2) {
            float x = float(side) * halfW;
            Vec3 nrm{float(side), 0, 0};
            Vec3 center{x, 0, 0};
            Vec3 p0{x, c0 * radius * 0.62f, s0 * radius * 0.62f};
            Vec3 p1{x, c1 * radius * 0.62f, s1 * radius * 0.62f};
            Vec3 o0{x, c0 * radius, s0 * radius};
            Vec3 o1{x, c1 * radius, s1 * radius};
            mesh.addQuad(o0, o1, p1, p0, nrm, tire);
            mesh.addTriangle(center, p0, p1, nrm, rim);
        }
    }
}

void buildCarShadow(CpuMesh& mesh) {
    const Vec3 ink{0.02f, 0.02f, 0.025f};
    mesh.addQuad({-1.05f, 0, -2.1f}, {1.05f, 0, -2.1f}, {1.05f, 0, 2.15f}, {-1.05f, 0, 2.15f}, {0, 1, 0}, ink);
}

Mat4 carModelMatrix(const Car& car) {
    Vec3 right, up, forward;
    car.body.Axes(right, up, forward);
    return basis(car.Origin(), right, up, forward);
}
