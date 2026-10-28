#pragma once

#include "GameObject/GameObject.h"

class CameraObject;
class CarObject;

// 試験コースの計器。速度、ギアと回転数、各輪の荷重・滑り・路面、車体の回転と横の加速度、空中の時間を出す。
class TelemetryHudObject : public GameObject {
public:
    explicit TelemetryHudObject(Scene& scene) : GameObject(scene) { order = 100; }
    bool Init() override;
    void StartUpdate(float dt) override;
    void Draw2D(System::Renderer2D& hud) override;

private:
    const CarObject* car_ = nullptr;
    const CameraObject* camera_ = nullptr;
    bool showHelp_ = true;
};
