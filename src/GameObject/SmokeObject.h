#pragma once

#include "GameObject/GameObject.h"
#include "System/Renderer3D.h"

#include <cstdint>
#include <vector>

class CameraObject;
class CarObject;

// タイヤの白煙。路面の上で滑っている車輪（ドリフト、空転、ロック）の接地点から煙の粒を出し、
// 広がりながら薄くなって消える。粒は固定ステップで出し、毎フレーム動かす。
class SmokeObject : public GameObject {
public:
    explicit SmokeObject(Scene& scene) : GameObject(scene) { order = 40; }
    bool Init() override;
    void PostPhysicsUpdate(float dt) override;
    void LateUpdate(float dt) override;
    void Draw3D(System::Renderer3D& renderer) override;

private:
    struct Particle {
        Vec3 position;
        Vec3 velocity;
        float age = 0;
        float life = 1;
        float startSize = 1;
        float endSize = 1;
        float alpha = 1;  // 出たときの濃さ
    };

    float random();  // 0..1

    const CarObject* car_ = nullptr;
    const CameraObject* camera_ = nullptr;
    std::vector<Particle> particles_;
    std::vector<System::SmokePuff> puffs_;
    float pending_[4] = {};  // 車輪ごとの、まだ出していない粒の端数
    std::uint32_t seed_ = 12345;
};
