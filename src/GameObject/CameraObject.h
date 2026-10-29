#pragma once

#include "GameObject/GameObject.h"
#include "geom.hpp"

class CarObject;

// 車を追うカメラ。C で 3 つの距離を切り替える。
class CameraObject : public GameObject {
public:
    explicit CameraObject(Scene& scene) : GameObject(scene) { order = 30; }
    bool Init() override;
    void StartUpdate(float dt) override;
    void LateUpdate(float dt) override;

    // 次の LateUpdate で補間せずに車の位置へ飛ぶ。
    void Snap() { ready_ = false; }
    Mat4 ViewProj() const;
    Vec3 Eye() const { return eye_; }
    Vec3 Look() const { return look_; }
    int Mode() const { return mode_; }

private:
    const CarObject* car_ = nullptr;
    Vec3 eye_{0, 8, -12};
    Vec3 look_{0, 0, 0};
    bool ready_ = false;
    int mode_ = 0;
};
