#include "car.hpp"

#include "Vehicle/VehicleSpec.h"

#include <cmath>

namespace {

constexpr float kMaxSpeed = 82.0f;      // 約 295 km/h
constexpr float kMaxReverse = 9.0f;
constexpr float kAccel = 13.0f;
constexpr float kBrake = 30.0f;
constexpr float kReverseAccel = 8.0f;
constexpr float kDrag = 0.00045f;
constexpr float kRolling = 0.9f;
constexpr float kWheelbase = 3.05f;
constexpr float kSteerRate = 2.6f;
// 壁の衝突で加わった横の速度と回転を、タイヤのグリップが打ち消すまでの時定数（秒）。
constexpr float kGripRecoverSec = 0.15f;

float maxSteerAt(float speed) {
    return 0.46f / (1.0f + std::fabs(speed) * 0.085f);
}

// 横位置から路面の材質を決める。縁石の内側 0.15 m までは路面、その外は芝として扱う。
// 物理の段階 4 で TrackCollider に移す。
const PhysicsMaterial& surfaceAt(const Track& track, float lateral) {
    if (std::fabs(lateral) <= track.halfWidth + 0.15f) {
        return Materials::kAsphalt;
    }
    return Materials::kRunoff;
}

}  // namespace

Car::Car() {
    using namespace VehicleSpec;
    const float x[kWheelCount] = {-kWheelX, kWheelX, -kWheelX, kWheelX};
    const float z[kWheelCount] = {kFrontWheelZ, kFrontWheelZ, kRearWheelZ, kRearWheelZ};
    for (int i = 0; i < kWheelCount; ++i) {
        WheelCollider& wheel = wheels[i];
        wheel.body = &body;
        wheel.mount = Vec3{x[i], kMountY - kCenterOfMassY, z[i]};
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
    }
    wheels[kFrontLeft].antiRollPartner = &wheels[kFrontRight];
    wheels[kFrontRight].antiRollPartner = &wheels[kFrontLeft];
    wheels[kRearLeft].antiRollPartner = &wheels[kRearRight];
    wheels[kRearRight].antiRollPartner = &wheels[kRearLeft];
}

void Car::reset(const Track& track) {
    body = Rigidbody{};
    body.position = {track.spawnX, track.spawnY, track.spawnZ};
    body.yaw = track.spawnYaw;
    body.mass = VehicleSpec::kMass;
    body.inertiaYaw = VehicleSpec::kInertiaYaw;
    body.inertiaPitch = VehicleSpec::kInertiaPitch;
    body.inertiaRoll = VehicleSpec::kInertiaRoll;
    // 原点を地面に置く。サスペンションが止まっているときの縮みになる高さで、すぐに落ち着く。
    body.position.y += VehicleSpec::kCenterOfMassY;
    for (WheelCollider& wheel : wheels) {
        wheel.Reset();
    }
    steerAngle = 0;
    wheelSpin = 0;
    brakeHold = 0;
    gripYawRate = 0;
    surface = &Materials::kAsphalt;
    ground = track.query(body.position.x, body.position.z, track.spawnY);
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
    return body.ToWorld(Vec3{0, -VehicleSpec::kCenterOfMassY, 0});
}

Vec3 Car::WheelCenter(int wheel) const {
    const WheelCollider& w = wheels[wheel];
    return Vec3{w.mount.x, w.mount.y + VehicleSpec::kCenterOfMassY - w.length, w.mount.z};
}

void Car::stepGrip(float dt) {
    // ------------------------------------------------------------------------
    // グリップ走行。ドリフトはここには書いていない。
    //
    // 速度は車体の前方だけを向く。横滑りはしない。
    // 走っているときの曲がりは、滑らない自転車モデル（ホイールベースと前輪の切れ角）。
    // ほぼ止まっているときは、据え切りで少しだけ向きが変わる。壁に向かって止まっても戻れる。
    //
    // ドリフトの課題では、この関数を差し替える。
    // たとえば横方向の速度、スリップ角、前後で違うグリップをここに足す。
    // アクセルとブレーキは update() 側、壁との接触は ColliderManager にある。
    // ------------------------------------------------------------------------
    if (Airborne()) {
        // 空中ではタイヤが路面に触れないので、向きの回り方と速度はそのまま残る。
        // グリップが決めた回転は 0 として覚え、着地した後は空中で付いた回転を下の spin として消していく。
        gripYawRate = 0;
        body.yaw += body.yawRate * dt;
        body.position.x += body.velocity.x * dt;
        body.position.z += body.velocity.z * dt;
        return;
    }
    float speed = body.ForwardSpeed();
    float lateral = body.LateralSpeed();
    float yawRate = 0;
    if (std::fabs(speed) > 0.05f) {
        yawRate = (speed / kWheelbase) * std::tan(steerAngle);
    }
    float slow = clampf(1.0f - std::fabs(speed) / 8.0f, 0.0f, 1.0f);
    yawRate += steerAngle * 0.9f * slow;

    // 壁の衝突で加わった横の速度と回転は、グリップですぐに消える。衝突がなければどちらも 0。
    // 段階 8 のタイヤモデルで、タイヤの力として置き換える。
    float recover = std::exp(-dt / kGripRecoverSec);
    float spin = (body.yawRate - gripYawRate) * recover;
    gripYawRate = yawRate;
    body.yawRate = yawRate + spin;
    body.yaw += body.yawRate * dt;

    // 水平の速度だけを決める。上下の速度と高さは ColliderManager がサスペンションで進める。
    float vy = body.velocity.y;
    body.velocity = body.Forward() * speed + body.Right() * (lateral * recover);
    body.velocity.y = vy;
    body.position.x += body.velocity.x * dt;
    body.position.z += body.velocity.z * dt;
}

void Car::update(float dt, const DriveInput& input, const Track& track) {
    float speed = body.ForwardSpeed();
    float lateral = body.LateralSpeed();
    float target = clampf(input.steer, -1.0f, 1.0f) * maxSteerAt(speed);
    float step = kSteerRate * dt;
    steerAngle += clampf(target - steerAngle, -step, step);
    steerAngle = clampf(steerAngle, -0.6f, 0.6f);

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

    float force = 0;
    float speedBefore = speed;
    if (input.brake > 0.05f && !reversing) {
        force -= input.brake * kBrake * (speed >= 0 ? 1.0f : -1.0f);
    } else if (reversing) {
        force -= input.brake * kReverseAccel;
    } else if (input.throttle > 0.0f && speed > -0.4f) {
        float ratio = clampf(speed / kMaxSpeed, 0.0f, 1.0f);
        force += input.throttle * kAccel * (1.0f - ratio * ratio);
    } else if (input.throttle > 0.0f) {
        force += input.throttle * kBrake;  // 後ろへ動いている間のアクセルはブレーキとして効く
    }
    force -= kDrag * speed * std::fabs(speed);
    if (std::fabs(speed) > 0.05f) {
        force -= kRolling * (speed > 0 ? 1.0f : -1.0f);
    }
    // 路面の材質で駆動力を絞り、路面の転がり抵抗を足す。
    if (force > 0) {
        force *= surface->gripScale;
    }
    if (std::fabs(speed) > 0.2f) {
        force -= surface->rollingResistance * (speed > 0 ? 1.0f : -1.0f);
    }
    // 4 輪とも浮いている間は、タイヤの力（駆動、ブレーキ、転がり抵抗）がなく、空気抵抗だけが掛かる。
    if (Airborne()) {
        force = -kDrag * speed * std::fabs(speed);
    }

    speed += force * dt;
    if (!reversing && input.brake > 0.05f) {
        if (speedBefore >= 0 && speed < 0) speed = 0;
        if (speedBefore <= 0 && speed > 0) speed = 0;
    }
    speed = clampf(speed, -kMaxReverse, kMaxSpeed);
    wheelSpin += speed / VehicleSpec::kWheelRadius * dt;
    float vy = body.velocity.y;
    body.velocity = body.Forward() * speed + body.Right() * lateral;
    body.velocity.y = vy;

    stepGrip(dt);

    float preferS = ground.found ? ground.s : -1.0f;
    Vec3 origin = Origin();
    ground = track.queryLocal(origin.x, origin.z, origin.y, preferS);
    if (!ground.found) {
        return;
    }
    // 高さは 4 輪のサスペンション、壁との接触は車体の BoxCollider で、どちらも ColliderManager が解く。
    surface = &surfaceAt(track, ground.lateral);
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
