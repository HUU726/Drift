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
constexpr float kGripRecoverTime = 0.6f;  // ドリフトを抜けた後、後輪の μ を普段へ戻す時間（秒）
constexpr float kSwitchStick = 0.5f;      // 振り返し: 反対へこれより大きく倒すと始める（パッドの遊びを引いた後の値）
constexpr float kSwitchThrottle = 0.1f;   // 振り返し: アクセルがこれより小さい（抜いている）ときだけ始める
constexpr float kSwitchHoldTime = 0.1f;   // 振り返し: 倒し続ける時間（秒）。修正で一瞬倒したくらいでは始めない
constexpr float kSwitchGain = 16.0f;      // 振り返し: 反対側の目標の角度との差（rad）に掛ける角加速度（1/s²）
constexpr float kSwitchDone = 0.17f;      // 振り返し: 反対側へこの角度（rad、約 10°）移ったら終える
constexpr float kPathRateSmoothing = 0.1f;  // 曲がりの補助: 進む向きの回る速さをならす時間（秒）
constexpr float kMaxCurveAccel = 18.0f;     // 曲がりの補助が出す横の加速度の上限（m/s²）
constexpr float kMaxKeepAccel = 3.5f;       // 速さの補助が出す前向きの加速度の上限（m/s²）
constexpr float kAlignStart = 0.05f;       // ドリフト後の向きの補正: 道の向きとの差がこれ（rad、約 3°）より大きければ始める
constexpr float kAlignDone = 0.01f;        // 差がこれ（rad）より小さくなるか、逆向きになったら終える
constexpr float kAlignMaxTime = 2.5f;      // 長くてもこの時間（秒）で終える
constexpr float kExitBoostTime = 0.6f;     // ドリフトを抜けたときの加速を続ける時間（秒）
constexpr float kAlignRate = 3.0f;         // 差（rad）に掛ける、目標の回る速さ（1/s）
constexpr float kAlignMaxRate = 3.0f;      // 目標の回る速さの上限（rad/s、約 170°/s）。差が大きいほど速く戻す
constexpr float kAlignResponse = 20.0f;     // 目標の回る速さとの差（rad/s）に掛ける角加速度（1/s）

}  // namespace

void Car::addAccel(Vec3 accel, float dt) {
    // 物理は 1 ステップを細かく分けて進め、力は最初の 1 回でしか使われない（AddForceAt で足すと効きが目減りする）。
    // 重心に掛ける補助の加速度は、ステップ全体の分を速度に直接足す。
    body.velocity = body.velocity + accel * dt;
}

void Car::pushForward(Vec3 direction, float accel, float dt) {
    addAccel(direction * accel, dt);
    // 車輪は車体の前向きに転がるので、前向きに増える速さの分だけ回す。
    float forwardGain = accel * dt * dot(direction, body.Forward());
    if (forwardGain <= 0) {
        return;
    }
    for (WheelCollider& wheel : wheels) {
        wheel.omega += forwardGain / wheel.radius;
    }
}

void Car::updateDrift(float dt, const DriveInput& input, float speed) {
    // 車体の滑りの角度。進む向きと車体の向きの差で、右へ滑るのが正（右のコーナーのドリフトでは負）。
    driftAngle = speed > 1.0f ? std::atan2(body.LateralSpeed(), speed) : 0.0f;
    float angle = std::fabs(driftAngle);
    float angleRate = (driftAngle - prevDriftAngle_) / dt;
    prevDriftAngle_ = driftAngle;
    bool airborne = Airborne();

    // ドリフトはサイドブレーキ（Space）を押している間だけ。離すと普段の車へ戻る。
    // 押し始めはサイドブレーキで後ろを流し、角度が driftEnterAngle を越えたらブレーキを外してアクセルで保つ。
    // ブレーキを踏んでも切れない（速さを落としながら角度を保てる）。
    bool held = driftAssist && input.handbrake && !airborne && angle < kDriftSpinAngle;
    const bool wasDrifting = drifting && driftKicked_;
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

    // ドリフト後の向きの補正。ドリフトを抜けたとき（Space を離したとき）、車体の向きと道の向きの差を覚え、
    // 車体を道の向きへ回す。車体が道の向きに届いたか通り越したら（差の向きが逆になったら）、次のドリフトまで補正しない。
    // わざと通り越して次のドリフトに入る操作を邪魔しないため。
    float roadError = ground.found ? wrapPi(ground.yaw - body.yaw) : 0.0f;
    if (wasDrifting && !drifting && ground.found && std::fabs(roadError) > kAlignStart) {
        aligning_ = true;
        alignSide_ = roadError > 0 ? 1.0f : -1.0f;
        alignTime_ = 0;
    }
    if (drifting || airborne || !ground.found) {
        aligning_ = false;
    }
    if (aligning_) {
        alignTime_ += dt;
        if (roadError * alignSide_ <= kAlignDone || alignTime_ > kAlignMaxTime) {
            aligning_ = false;
        }
    }
    if (aligning_ && driftExitAlign > 0) {
        // 進む向きに対する車体の回る速さを、差に比例した目標の速さ（上限 kAlignMaxRate × 強さ）へ寄せる。
        // 差に比例した力で回すと、深い角度で離したときに急に回されて道の向きを行き過ぎる。
        // 戻ったグリップで目標より速く回るときは抑えるので、補正なしより穏やかに戻る。
        float maxRate = kAlignMaxRate * driftExitAlign;
        float targetRate = clampf(kAlignRate * driftExitAlign * roadError, -maxRate, maxRate);
        float accel = kAlignResponse * (targetRate - (body.yawRate - pathRate_));
        body.AddTorque(accel * body.inertiaYaw, 0, 0, dt);
    }

    // ドリフトを抜けたときの加速（ゲームとしての補助）。抜けてから kExitBoostTime の間、アクセルを踏んでいれば
    // コースの進む向きへ アクセル × driftExitBoost で押す。ブレーキを踏む、またドリフトに入ると終える。
    if (wasDrifting && !drifting) {
        exitBoostTime_ = kExitBoostTime;
    }
    if (drifting || airborne || input.brake > 0.05f) {
        exitBoostTime_ = 0;
    }
    if (exitBoostTime_ > 0) {
        exitBoostTime_ -= dt;
        if (ground.found && driftExitBoost > 0 && input.throttle > 0.05f) {
            Vec3 roadForward{std::sin(ground.yaw), 0, std::cos(ground.yaw)};
            pushForward(roadForward, driftExitBoost * clampf(input.throttle, 0.0f, 1.0f), dt);
        }
    }

    // ドリフト中の後輪。普段の後輪の特性のまま、μ だけ前輪の driftRearGrip 倍にする。
    // ドリフトを抜けた後は、kGripRecoverTime かけて普段の μ へ戻す。一気に戻すと、横を向いた車体が強く引き戻されて
    // 逆へ振られる。
    const TireParams& normal = rearDriftTires_ ? rearDriftTire.params : rearGripTire.params;
    gripRecover_ = drifting ? 0.0f : std::min(1.0f, gripRecover_ + dt / kGripRecoverTime);
    float driftMu = frontTire.params.muScale * driftRearGrip;
    driftModeTire.params = normal;
    driftModeTire.params.muScale = driftMu + (normal.muScale - driftMu) * gripRecover_;

    // 振り返し。ドリフト中にアクセルを抜いて、スティックを今のコーナーと反対へ大きく倒し続けると、
    // 反対側のドリフトへ車体を振る。アクセルを踏んだまま反対へ倒すのは、ラインを外へ膨らませる操作なので振り返さない。
    // 右へ滑っている（角度が正）のは左のコーナーなので、反対へ倒すのは右（スティックが正）。
    float side = driftAngle < 0 ? -1.0f : 1.0f;
    if (!drifting || !driftKicked_) {
        switching_ = false;
        switchHold_ = 0;
    } else if (!switching_) {
        bool lifted = input.throttle < kSwitchThrottle;
        bool opposite = angle > driftEnterAngle && lifted && input.steer * side > kSwitchStick;
        switchHold_ = opposite ? switchHold_ + dt : 0.0f;
        if (switchHold_ > kSwitchHoldTime) {
            switching_ = true;
            switchSide_ = -side;
        }
    } else if (driftAngle * switchSide_ > kSwitchDone) {
        // 反対側へ移り終えた。ここからは普段の角度の補助で保つ。
        switching_ = false;
        switchHold_ = 0;
    }

    // 角度の補助。アクセルを踏んでいる間は、踏み込みに応じた角度へ車体を回す（深すぎれば戻す）。
    // アクセルを離すと、角度の変わる速さだけを抑え、車が自然にまっすぐへ戻るのを待つ。
    // 向きを変えている間（ブレーキで流している間）は、どちらへ滑るかが決まっていないので補助しない。
    // 振り返しの間は、反対側の角度を目標に、強めに車体を回す。
    if (drifting && driftKicked_ && driftAngleAssist > 0) {
        float target = driftMinAngle + (driftMaxAngle - driftMinAngle) * clampf(input.throttle, 0.0f, 1.0f);
        // 角度を変える角加速度（右へ滑る向き = 角度が増える向きが正）。
        float accel = -kAngleDamping * angleRate;
        if (switching_) {
            accel += kSwitchGain * (switchSide_ * target - driftAngle);
        } else if (input.throttle > 0.05f) {
            accel += kAngleGain * (side * target - driftAngle);
        }
        // 角度を増やす（右へ滑らせる）には、車体を左へ回す（ヨーが負）。
        body.AddTorque(-accel * driftAngleAssist * body.inertiaYaw, 0, 0, dt);
    }

    // 曲がりの補助。進む向きの回る速さ（車体のヨーレート + 角度の変わる速さ）を、スティックの倒す量 × 倒しきったときの
    // 速さへ寄せる。倒しきったときの速さは、driftCurveRate と、半径 driftMinRadius で回る速さ（速さ / 半径）の大きいほう。
    // 速いときも、ヘアピンを回れる半径まで曲がれる。
    // 進む向きに直角な力を重心に掛けるので、速さは変えずに進む向きだけを曲げる。
    float pathRate = body.yawRate + angleRate;
    pathRate_ += (pathRate - pathRate_) * (1.0f - std::exp(-dt / kPathRateSmoothing));
    // 速さは、車体の前向きの成分ではなく、実際に進む速さ（横を向いているほど前向きの成分より大きい）。
    float travelSpeed = std::sqrt(body.velocity.x * body.velocity.x + body.velocity.z * body.velocity.z);
    if (drifting && driftKicked_ && driftCurveAssist > 0 && travelSpeed > 1.0f) {
        float fullRate = std::max(driftCurveRate, travelSpeed / std::max(driftMinRadius, 1.0f));
        // 振り返しの間は、倒したスティックは振り返しの合図なので、曲がりには使わない（まっすぐ進む）。
        // 使うと、振り返しながら進む向きまで内側へ急に回り、内側のガードレールに当たる。
        float target = switching_ ? 0.0f : clampf(input.steer, -1.0f, 1.0f) * fullRate;
        float lateralAccel = driftCurveAssist * (target - pathRate_) * travelSpeed;
        lateralAccel = clampf(lateralAccel, -kMaxCurveAccel, kMaxCurveAccel);
        Vec3 velocity{body.velocity.x, 0, body.velocity.z};
        float length = std::sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
        // 進む向きの右（ヨーが正の向きへ曲げる向き）。
        Vec3 right{velocity.z / length, 0, -velocity.x / length};
        addAccel(right * lateralAccel, dt);
    }

    // 速さの補助。ドリフト中に落ちた速さを、driftSpeedKeep に応じて進む向きへ押し戻す。落ちた分だけを補い、
    // 速くはしない。直前のステップの減り方を使う。サイドブレーキで向きを変えている間（いちばん速さが落ちる）は
    // いつも補い、角度が付いた後はアクセルの踏み込みに応じて補う（アクセルを抜けば速さは落ちる）。
    // ブレーキを踏んでいる間は補わない。
    // 減り方は、直前に補った分を足し戻した、補わなかったときの減り方にする（補った分で減り方が小さく見えると、
    // 補う量も減って効きが弱まる）。
    float travelDecel = (prevTravelSpeed_ - travelSpeed) / dt + lastKeepAccel_;
    prevTravelSpeed_ = travelSpeed;
    lastKeepAccel_ = 0;
    float keepShare = driftKicked_ ? clampf(input.throttle, 0.0f, 1.0f) : 1.0f;
    if (drifting && driftSpeedKeep > 0 && keepShare > 0.05f && input.brake < 0.05f && travelSpeed > 1.0f &&
        travelDecel > 0) {
        float keep = std::min(travelDecel * driftSpeedKeep * keepShare, kMaxKeepAccel);
        Vec3 along{body.velocity.x / travelSpeed, 0, body.velocity.z / travelSpeed};
        addAccel(along * keep, dt);
        lastKeepAccel_ = keep;
    }

    // ドリフト中の加速。角度が付いた後、アクセルの踏み込み × driftBoost で進む向きへ押す（ゲームとしての補助）。
    // 回る半径はスティックの量で決まり速さによらないので、同じ弧のまま速く回れる。
    if (drifting && driftKicked_ && driftBoost > 0 && input.throttle > 0.05f && input.brake < 0.05f && travelSpeed > 1.0f) {
        Vec3 along{body.velocity.x / travelSpeed, 0, body.velocity.z / travelSpeed};
        pushForward(along, driftBoost * clampf(input.throttle, 0.0f, 1.0f), dt);
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
        wheel.forceAppHeight = kForceAppHeight;
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
    prevTravelSpeed_ = 0;
    lastKeepAccel_ = 0;
    aligning_ = false;
    exitBoostTime_ = 0;
    gripRecover_ = 1;
    switching_ = false;
    switchHold_ = 0;
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
            bool recovering = gripRecover_ < 1.0f;
            wheel.tire = drifting || recovering ? &driftModeTire : (rearDriftTires_ ? static_cast<const ITireModel*>(&rearDriftTire) : &rearGripTire);
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
