#include "Vehicle/WheelCollider.h"

#include "Physics/Rigidbody.h"
#include "Vehicle/TireModel.h"

#include <algorithm>
#include <cmath>

namespace {

// 滑りを求めるときの速度の下限（m/s）。止まりかけで割り算が暴れないようにする。
constexpr float kMinSlipSpeed = 3.0f;

// 回転を止める向きのトルクで、omega を 0 に向けて減らす（0 を越えて逆には回さない）。
float resist(float omega, float amount) {
    if (std::fabs(omega) <= amount) {
        return 0;
    }
    return omega - (omega > 0 ? amount : -amount);
}

}  // namespace

void WheelCollider::Update(const RaycastHit& hit, Vec3 mountWorld, float dt) {
    // 取付点から路面までの距離。車輪の半径の分を引くと、サスペンションの長さになる。
    float reach = hit.hit ? mountWorld.y - hit.point.y - radius : restLength;
    if (hit.hit) {
        lastS = hit.s;
    }
    if (!hit.hit || reach >= restLength) {
        grounded = false;
        compression = 0;
        length = restLength;
        springForce_ = 0;
        load = 0;
        hasPrev_ = false;
        return;
    }
    grounded = true;
    groundY = hit.point.y;
    surface = hit.material;
    // 自然長より縮むことはない（車輪が取付点を越える）。越えた分は ColliderManager が底突きとして押し戻す。
    compression = std::min(restLength - reach, restLength);
    length = std::max(reach, 0.0f);

    float rate = hasPrev_ ? (compression - prevCompression_) / dt : 0.0f;
    prevCompression_ = compression;
    hasPrev_ = true;

    float spring = springRate * compression;
    if (compression > bumpStop) {
        // バンプストップのゴムは、縮むときに受け止めた力の一部しか返さない。着地で跳ね上がりすぎないため。
        spring += bumpStopRate * (compression - bumpStop) * (rate > 0 ? 1.0f : bumpStopReturn);
    }
    float damper = damping * rate * (rate > 0 ? bumpDampingScale : reboundDampingScale);
    // 路面を引っ張る向きの力は出さない。
    springForce_ = std::max(0.0f, spring + damper);
}

float WheelCollider::Force() {
    if (!grounded) {
        return 0;
    }
    float force = springForce_;
    if (antiRollPartner) {
        // 自分の側が多く縮んでいれば押し上げ、反対側を下げる（反対側では符号が逆になる）。
        force += antiRollRate * (compression - antiRollPartner->compression);
    }
    load = std::max(0.0f, force);
    return load;
}

void WheelCollider::Reset() {
    // 置き直すのは路面の上なので、接地しているとみなす。最初の物理ステップで正しい値になる。
    grounded = true;
    compression = 0;
    length = restLength;
    load = 0;
    lastS = -1;
    springForce_ = 0;
    omega = 0;
    spinAngle = 0;
    slipRatio = 0;
    slipAngle = 0;
    sliding = false;
    prevCompression_ = 0;
    hasPrev_ = false;
}

float WheelCollider::limitSlip(float w, float vx, float speed, float slipAngle) const {
    // ABS とトラクションコントロール: スリップ率がピークの slipControlShare を越えないように回転を抑える。
    // ピークを越えると力が下がり、ロックや空転が止まらなくなるため。
    // 曲がっている間は横の力が摩擦円を使っているので、横の滑りが大きいほど前後に許す滑りを小さくする。
    float lateralUse = slipAngle / tire->PeakSlipAngle();
    float remaining = std::sqrt(std::max(0.05f, 1.0f - lateralUse * lateralUse));
    float limit = tire->PeakSlipRatio() * slipControlShare * remaining * speed;
    if (antiLock && !handbrake && brakeTorque > 0) {
        if (vx > 0) w = std::max(w, (vx - limit) / radius);
        if (vx < 0) w = std::min(w, (vx + limit) / radius);
    }
    if (tractionControl && !drifting && driveTorque != 0) {
        if (driveTorque > 0) w = std::min(w, (vx + limit) / radius);
        if (driveTorque < 0) w = std::max(w, (vx - limit) / radius);
    }
    return w;
}

void WheelCollider::StepTire(float load, float dt, float gravity, Vec3& force, Vec3& point) {
    force = Vec3{};
    const Rigidbody& b = *body;
    Vec3 mountWorld = b.ToWorld(mount);
    point = Vec3{mountWorld.x, groundY, mountWorld.z};
    if (!grounded || load <= 0 || !tire) {
        // 浮いている車輪は、駆動とブレーキだけで回る。
        omega = resist(omega + driveTorque / wheelInertia * dt, brakeTorque / wheelInertia * dt);
        spinAngle += omega * dt;
        slipRatio = 0;
        slipAngle = 0;
        sliding = false;
        return;
    }

    // 車輪の向き（前輪は切れ角の分だけ回す）と、接地点の速度。
    float heading = b.yaw + steer;
    Vec3 forward{std::sin(heading), 0, std::cos(heading)};
    Vec3 right{std::cos(heading), 0, -std::sin(heading)};
    Vec3 v = b.VelocityAt(point);
    float vx = dot(v, forward);
    float vy = dot(v, right);
    float mu = surface ? surface->friction : 1.0f;
    float speed = std::max(std::fabs(vx), kMinSlipSpeed);

    // 回転を陰的に進める。タイヤの前後の力を線形（スリップ率に比例）とみなして解くと、
    // 小さい時間刻みでも回転と路面の速度の差が振動しない。
    float stiffness = tire->LongitudinalStiffness(load, mu);
    float k = stiffness * radius / speed;
    float effectiveInertia = wheelInertia + dt * k * radius;
    float w = (wheelInertia * omega + dt * (driveTorque + k * vx)) / effectiveInertia;
    // ブレーキと転がり抵抗は回転を止める向き。転がり抵抗は、荷重に比例して車を減速させる分のトルク。
    float rollDecel = rollingDecel + (surface ? surface->rollingResistance : 0.0f);
    float rolling = load / gravity * rollDecel * radius;
    w = resist(w, (brakeTorque + rolling) * dt / effectiveInertia);

    slipAngle = std::atan2(vy, speed);
    w = limitSlip(w, vx, speed, slipAngle);

    slipRatio = (w * radius - vx) / speed;
    TireForce f = tire->Evaluate(load, slipRatio, slipAngle, mu);
    if (f.sliding) {
        // ピークを越えると前後の力は滑りに比例しないので、実際の力で回転を進め直す。
        w = omega + dt * (driveTorque - f.fx * radius) / wheelInertia;
        w = resist(w, (brakeTorque + rolling) * dt / wheelInertia);
        w = limitSlip(w, vx, speed, slipAngle);
        slipRatio = (w * radius - vx) / speed;
        f = tire->Evaluate(load, slipRatio, slipAngle, mu);
    }
    omega = w;
    spinAngle += omega * dt;
    sliding = f.sliding;
    force = forward * f.fx + right * f.fy;
}
