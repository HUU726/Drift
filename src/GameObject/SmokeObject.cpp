#include "GameObject/SmokeObject.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "Scene/Scene.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr int kMaxParticles = 600;
constexpr float kMinSlipSpeed = 3.0f;    // 接地点の滑る速さがこれを越えると煙が出る（m/s）
constexpr float kFullSlipSpeed = 15.0f;  // この速さで最も多く、濃く出る（m/s）
constexpr float kRatePerWheel = 45.0f;   // 1 輪から 1 秒に出す粒の数（最も多いとき）

}  // namespace

bool SmokeObject::Init() {
    car_ = scene_.Find<CarObject>();
    camera_ = scene_.Find<CameraObject>();
    particles_.reserve(kMaxParticles);
    return car_ && camera_;
}

float SmokeObject::random() {
    seed_ = seed_ * 1664525u + 1013904223u;
    return float(seed_ >> 8) / float(1u << 24);
}

void SmokeObject::PostPhysicsUpdate(float dt) {
    const Car& car = car_->GetCar();
    const Rigidbody& body = car.body;
    float speed = std::max(std::fabs(body.ForwardSpeed()), 0.5f);
    for (int i = 0; i < Car::kWheelCount; ++i) {
        const WheelCollider& wheel = car.wheels[i];
        if (!wheel.grounded) {
            pending_[i] = 0;
            continue;
        }
        // 接地点が路面の上を滑る速さ。前後はスリップ率、横はスリップ角から求める。
        float slipSpeed = speed * std::sqrt(wheel.slipRatio * wheel.slipRatio +
                                            std::tan(wheel.slipAngle) * std::tan(wheel.slipAngle));
        float strength = clampf((slipSpeed - kMinSlipSpeed) / (kFullSlipSpeed - kMinSlipSpeed), 0.0f, 1.0f);
        if (strength <= 0) {
            pending_[i] = 0;
            continue;
        }
        Vec3 contact = body.ToWorld(wheel.mount);
        contact.y = wheel.groundY;
        pending_[i] += kRatePerWheel * strength * dt;
        while (pending_[i] >= 1.0f) {
            pending_[i] -= 1.0f;
            if (static_cast<int>(particles_.size()) >= kMaxParticles) {
                particles_.erase(particles_.begin());  // いちばん古い粒を消す
            }
            Particle p;
            p.position = contact + Vec3{(random() - 0.5f) * 0.4f, 0.25f, (random() - 0.5f) * 0.4f};
            // 車の速さを少しだけ受け継ぎ、ゆっくり上へ広がる。
            p.velocity = body.velocity * 0.15f + Vec3{(random() - 0.5f) * 1.2f, 0.5f + random() * 0.6f, (random() - 0.5f) * 1.2f};
            p.life = 2.0f + random() * 1.2f;
            p.startSize = 0.7f + random() * 0.3f;
            p.endSize = 4.0f + random() * 1.5f;
            p.alpha = 0.1f + 0.25f * strength;
            particles_.push_back(p);
        }
    }
}

void SmokeObject::LateUpdate(float dt) {
    if (scene_.Paused()) {
        return;  // ポーズ中は煙も止める
    }
    float drag = std::exp(-1.6f * dt);
    for (Particle& p : particles_) {
        p.age += dt;
        p.velocity = p.velocity * drag;
        p.position = p.position + p.velocity * dt;
    }
    particles_.erase(std::remove_if(particles_.begin(), particles_.end(), [](const Particle& p) { return p.age >= p.life; }),
                     particles_.end());
}

void SmokeObject::Draw3D(System::Renderer3D& renderer) {
    if (particles_.empty()) {
        return;
    }
    puffs_.clear();
    for (const Particle& p : particles_) {
        float t = p.age / p.life;
        // 出た直後に濃くなり、後はゆっくり薄くなる。
        float fadeIn = std::min(1.0f, p.age / 0.12f);
        float fadeOut = (1.0f - t) * (1.0f - t);
        System::SmokePuff puff;
        puff.position = p.position;
        puff.size = p.startSize + (p.endSize - p.startSize) * std::sqrt(t);
        puff.alpha = p.alpha * fadeIn * fadeOut;
        puffs_.push_back(puff);
    }
    // 板をカメラに向ける。丸い粒なので、右と上の向きの符号は問わない。
    Vec3 view = normalize(camera_->Look() - camera_->Eye());
    Vec3 right = normalize(cross(view, Vec3{0, 1, 0}));
    Vec3 up = cross(right, view);
    renderer.DrawSmoke(puffs_, right, up);
}
