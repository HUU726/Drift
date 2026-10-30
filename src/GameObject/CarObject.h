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
    // 壁（ガードレール）やジャンプ台に当たり始めたときに、当たりの強さに合わせて衝突の音を鳴らす。
    void OnCollision(const Contact& contact) override;
    // 車体が横転したら、横転の音を 1 回鳴らす。
    void PostPhysicsUpdate(float dt) override;
    // エンジン音とスキール音の値を、毎フレーム SoundManager に渡す。
    void LateUpdate(float dt) override;
    void Draw3D(System::Renderer3D& renderer) override;
    void UnInit() override;

    // スポーン地点に置き直す。
    void Reset();
    // コースのないシーンで、車を置く位置（原点）と向きを決める。Init より前に呼ぶ。
    void SetSpawn(Vec3 origin, float yaw) {
        spawn_ = origin;
        spawnYaw_ = yaw;
    }
    const Car& GetCar() const { return car_; }
    // 調整の画面が、タイヤや運転の補助の値をプレイ中に書き換えるために使う。
    Car& MutableCar() { return car_; }

private:
    const Track* track_ = nullptr;  // コースのないシーンでは null
    Vec3 spawn_;
    float spawnYaw_ = 0;
    Car car_;
    bool rolled_ = false;  // 横転している（音を鳴らした後、起き上がるまで次を鳴らさない）
    BoxCollider box_;
    System::GpuMesh body_;
    System::GpuMesh wheel_;
    System::GpuMesh shadow_;
};
