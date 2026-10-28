#pragma once

#include "GameObject/GameObject.h"
#include "System/Renderer3D.h"
#include "car.hpp"

// プレイヤーの車。Car を固定ステップで動かし、車体と車輪を描く。
class CarObject : public GameObject {
public:
    explicit CarObject(Scene& scene) : GameObject(scene) { order = 10; }
    bool Init() override;
    void FixedUpdate(float dt) override;
    void LateUpdate(float dt) override;
    void Draw3D(System::Renderer3D& renderer) override;
    void UnInit() override;

    // スポーン地点に置き直す。
    void Reset();
    const Car& GetCar() const { return car_; }

private:
    float computePitch() const;

    const Track* track_ = nullptr;
    Car car_;
    float pitch_ = 0;
    float roll_ = 0;
    System::GpuMesh body_;
    System::GpuMesh wheel_;
    System::GpuMesh shadow_;
};
