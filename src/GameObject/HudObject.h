#pragma once

#include "GameObject/GameObject.h"

class CameraObject;
class CarObject;
class RaceTimerObject;
class TrackObject;

// 速度、ラップ、コーナー名、操作説明、ミニマップ。H で操作説明を出し入れする。
class HudObject : public GameObject {
public:
    explicit HudObject(Scene& scene) : GameObject(scene) { order = 100; }
    bool Init() override;
    void StartUpdate(float dt) override;
    void Draw2D(System::Renderer2D& hud) override;

private:
    void drawMinimap(System::Renderer2D& hud);

    const CarObject* car_ = nullptr;
    const RaceTimerObject* timer_ = nullptr;
    const TrackObject* track_ = nullptr;
    const CameraObject* camera_ = nullptr;
    bool showHelp_ = true;
};
