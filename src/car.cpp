#include "car.hpp"

#include "Vehicle/VehicleSpec.h"

#include <algorithm>
#include <cmath>

namespace {

// 駆動は IPowertrain が決める。ブレーキは運転操作から車全体の減速（brakeDecel、m/s²）として求める。
// 実際に出る加速と減速は、タイヤが路面に伝えられる力（μ × 荷重）で決まる。
constexpr float kMaxReverse = 9.0f;     // 後退の最高速度（m/s）。ここで駆動を切る

// ドリフト
constexpr float kDriftSteerRate = 6.0f;   // 逆ハンドルを切る速さ（rad/s）。遅いと滑りの向きの変化に追いつかない
constexpr float kDriftMinSpeed = 8.0f;    // これより遅いとドリフトに入らない（m/s）。抜けるのはこの 6 割
constexpr float kDriftSpinAngle = 1.45f;  // これを越えたらスピンとしてドリフトをやめる（rad、約 83°）
constexpr float kAngleGain = 10.0f;       // 角度の補助: 目標の角度との差（rad）に掛ける角加速度（1/s²）
constexpr float kAngleDamping = 4.0f;     // 角度の補助: 角度の変わる速さ（rad/s）に掛ける角加速度（1/s）
constexpr float kPathRateSmoothing = 0.1f;  // 曲がりの補助: 進む向きの回る速さをならす時間（秒）
constexpr float kMaxCurveAccel = 12.0f;     // 曲がりの補助が出す横の加速度の上限（m/s²）

}  // namespace

void Car::updateDrift(float dt, const DriveInput& input, float speed) {
    // 車体の滑りの角度。進む向きと車体の向きの差で、右へ滑るのが正（右のコーナーのドリフトでは負）。
    driftAngle = speed > 1.0f ? std::atan2(body.LateralSpeed(), speed) : 0.0f;
    float angle = std::fabs(driftAngle);
    float angleRate = (driftAngle - prevDriftAngle_) / dt;
    prevDriftAngle_ = driftAngle;
    bool airborne = Airborne();

    // ドリフトはサイドブレーキ（Space）を押している間だけ。離すと普段の車へ戻る。
    // 押し始めはサイドブレーキで後ろを流し、角度が driftEnterAngle を越えたらブレーキを外してアクセルで保つ。
    bool held = driftAssist && input.handbrake && !airborne && input.brake < 0.5f && angle < kDriftSpinAngle;
    if (!drifting) {
        drifting = held && speed > kDriftMinSpeed;
        driftKicked_ = false;
    } else {
        drifting = held && speed > kDriftMinSpeed * 0.6f;
    }
    if (drifting && angle > driftEnterAngle) {
        driftKicked_ = true;
    }
    if (!drifting) {
        driftKicked_ = false;
    }

    // ドリフト中の後輪。普段の後輪の特性のまま、μ だけ前輪の driftRearGrip 倍にする。
    driftModeTire.params = rearDriftTires_ ? rearDriftTire.params : rearGripTire.params;
    driftModeTire.params.muScale = frontTire.params.muScale * driftRearGrip;

    // 角度の補助。アクセルを踏んでいる間は、踏み込みに応じた角度へ車体を回す（深すぎれば戻す）。
    // アクセルを離すと、角度の変わる速さだけを抑え、車が自然にまっすぐへ戻るのを待つ。
    // 向きを変えている間（ブレーキで流している間）は、どちらへ滑るかが決まっていないので補助しない。
    if (drifting && driftKicked_ && driftAngleAssist > 0) {
        float side = driftAngle < 0 ? -1.0f : 1.0f;
        float accel = -kAngleDamping * side * angleRate;
        if (input.throttle > 0.05f) {
            float target = driftMinAngle + (driftMaxAngle - driftMinAngle) * clampf(input.throttle, 0.0f, 1.0f);
            accel += kAngleGain * (target - angle);
        }
        // 角度を深くするには、滑る向きと逆へ車体を回す（右のコーナーでは右回り = ヨーが正）。
        body.AddTorque(-side * accel * driftAngleAssist * body.inertiaYaw, 0, 0, dt);
    }

    // 曲がりの補助。進む向きの回る速さ（車体のヨーレート + 角度の変わる速さ）を、スティックの倒す量 × driftCurveRate へ
    // 寄せる。進む向きに直角な力を重心に掛けるので、速さは変えずに進む向きだけを曲げる。
    float pathRate = body.yawRate + angleRate;
    pathRate_ += (pathRate - pathRate_) * (1.0f - std::exp(-dt / kPathRateSmoothing));
    if (drifting && driftKicked_ && driftCurveAssist > 0 && speed > 1.0f) {
        float target = clampf(input.steer, -1.0f, 1.0f) * driftCurveRate;
        float lateralAccel = driftCurveAssist * (target - pathRate_) * speed;
        lateralAccel = clampf(lateralAccel, -kMaxCurveAccel, kMaxCurveAccel);
        Vec3 velocity{body.velocity.x, 0, body.velocity.z};
        float length = std::sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
        // 進む向きの右（ヨーが正の向きへ曲げる向き）。
        Vec3 right{velocity.z / length, 0, -velocity.x / length};
        body.AddForceAt(right * (lateralAccel * body.mass), body.position);
    }
}

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
    rearGripTire.params.muScale *= kRearGripRatio;
    airControl.enabled = VehicleSpec::kAirControl;
    airControl.strength = VehicleSpec::kAirControlStrength;
    airControl.landingAssist = VehicleSpec::kLandingAssist;
    wheels[kFrontLeft].antiRollPartner = &wheels[kFrontRight];
    wheels[kFrontRight].antiRollPartner = &wheels[kFrontLeft];
    wheels[kRearLeft].antiRollPartner = &wheels[kRearRight];
    wheels[kRearRight].antiRollPartner = &wheels[kRearLeft];
}

void Car::UseDriftTires(bool drift) {
    // 車輪に付けるのは update（ドリフト中は driftModeTire に替える）。
    rearDriftTires_ = drift;
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

void Car::SetInertiaScale(float scale) {
    float ratio = scale / inertiaScale;
    inertiaScale = scale;
    body.inertiaYaw *= ratio;
    body.inertiaPitch *= ratio;
    body.inertiaRoll *= ratio;
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
    float massRatio = mass / VehicleSpec::kMass * inertiaScale;
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
    drifting = false;
    driftAngle = 0;
    driftKicked_ = false;
    prevDriftAngle_ = 0;
    pathRate_ = 0;
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
    updateDrift(dt, input, speed);

    // 前の車軸が進む向き（車体から見た角度、右が正）。前輪をこの向きにそろえると、前輪は横に滑らない。
    Vec3 frontAxle = body.ToWorld(Vec3{0, wheels[kFrontLeft].mount.y, wheels[kFrontLeft].mount.z});
    Vec3 frontVelocity = body.VelocityAt(frontAxle);
    float frontTravel = std::atan2(dot(frontVelocity, body.Right()), std::fabs(dot(frontVelocity, body.Forward())));
    float slipLimit = frontTire.PeakSlipAngle() * steerAssistShare;
    if (drifting) {
        // 自動の逆ハンドル。前輪を進む向きにそろえ、ハンドルの入力の分だけ（ピークの滑りの角度まで）内へ切る。
        // ハンドルの入力は、行きたい向きへ前輪の横の力を出すことになる。
        float target = frontTravel + clampf(input.steer, -1.0f, 1.0f) * slipLimit;
        float step = kDriftSteerRate * dt;
        steerAngle += clampf(target - steerAngle, -step, step);
        steerAngle = clampf(steerAngle, -driftMaxSteer, driftMaxSteer);
    } else {
        // 最大の切れ角は、速度が上がるほど小さくする（高速で切りすぎない）。
        float maxSteerNow = maxSteer / (1.0f + std::fabs(speed) * steerSpeedFalloff);
        float target = clampf(input.steer, -1.0f, 1.0f) * maxSteerNow;
        float step = steerRate * dt;
        // ドリフトから抜けた直後は逆ハンドルが大きく残っているので、戻す間は速く戻す。
        if (std::fabs(steerAngle) > maxSteer) {
            step = kDriftSteerRate * dt;
        }
        steerAngle += clampf(target - steerAngle, -step, step);
        // ハンドルの補助。前輪の滑りの角度（進む向きと車輪の向きの差）がピークを越えないように切れ角を抑え、
        // ハンドルを切っただけではタイヤが滑らないようにする。車が横に滑っているときは、進む向きへの逆ハンドルは切れる。
        if (steerAssist && std::fabs(speed) > 3.0f) {
            steerAngle = clampf(steerAngle, frontTravel - slipLimit, frontTravel + slipLimit);
        }
        steerAngle = clampf(steerAngle, -driftMaxSteer, driftMaxSteer);
        if (std::fabs(steerAngle) <= maxSteer + step) {
            steerAngle = clampf(steerAngle, -maxSteer, maxSteer);
        }
    }

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
    // ドリフト中とサイドブレーキ中はギアを固定する。サイドブレーキで後輪が止まると、回転数が下がって 1 速まで落ちるため。
    engine.holdGear = drifting || input.handbrake;
    float axleTorque = powertrain->Update(throttle, reversing, rearOmega, dt);

    // 駆動は後輪の左右に均等（オープンデフ）、ブレーキは前 6 : 後 4。サイドブレーキは後輪だけ。
    const float radius = VehicleSpec::kWheelRadius;
    for (int i = 0; i < kWheelCount; ++i) {
        WheelCollider& wheel = wheels[i];
        bool front = i < kRearLeft;
        wheel.steer = front ? steerAngle : 0.0f;
        if (!front) {
            wheel.tire = drifting ? &driftModeTire : (rearDriftTires_ ? static_cast<const ITireModel*>(&rearDriftTire) : &rearGripTire);
            wheel.drifting = drifting;
        }
        float driveTorque = front ? 0.0f : axleTorque * 0.5f;
        float brakeTorque = mass * brake * radius * 0.5f * (front ? brakeFrontShare : 1.0f - brakeFrontShare);
        // サイドブレーキは後輪だけに掛け、ABS を効かせずにロックさせる。掛けている間は駆動を切る（クラッチを切る）。
        // 切らないと、エンジンブレーキの分でトラクションコントロールが回転を保ち、ロックしない。
        // ドリフト中は、向きが変わった（角度が入る角度を越えた）後はブレーキを外し、アクセルで角度を保つ。
        wheel.handbrake = !front && input.handbrake && !(drifting && driftKicked_);
        if (wheel.handbrake) {
            brakeTorque += mass * handbrakeDecel * radius * 0.5f;
            driveTorque = 0;
        }
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
