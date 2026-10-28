#pragma once

#include "GameObject/GameObject.h"
#include "Physics/BoxCollider.h"
#include "System/Renderer3D.h"
#include "car.hpp"

// プレイヤーの車。Car を固定ステップで動かし、車体と車輪を描く。
// 車体の Rigidbody と 4 輪の WheelCollider、壁と当たる BoxCollider を ColliderManager に登録する。
class CarObject : public GameObject {
public:
    explicit CarObject(Scene& scene) : GameObject(scene) { order = 10; }
    bool Init() override;
    void FixedUpdate(float dt) override;
    void Draw3D(System::Renderer3D& renderer) override;
    void UnInit() override;

    // スポーン地点に置き直す。
    void Reset();
    const Car& GetCar() const { return car_; }

private:
    const Track* track_ = nullptr;
    Car car_;
    BoxCollider box_;
    System::GpuMesh body_;
    System::GpuMesh wheel_;
    System::GpuMesh shadow_;
};
