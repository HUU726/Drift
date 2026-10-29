#pragma once

#include "Scene/Scene.h"

class CameraObject;
class CarObject;
class RaceTimerObject;

// レース本編。リスタート（R）とポーズ（P）はシーンの入れ替えではなく、このシーンの状態として扱う。
// Esc でシーン選択の画面に戻る。
class RaceScene : public Scene {
public:
    explicit RaceScene(System::Context& ctx) : Scene(ctx) {}
    bool Init() override;
    bool Paused() const override { return paused_; }

protected:
    void OnStartUpdate(float dt) override;
    void GetView(Mat4& viewProj, Vec3& eye) const override;

private:
    CarObject* car_ = nullptr;
    RaceTimerObject* timer_ = nullptr;
    CameraObject* camera_ = nullptr;
    bool paused_ = false;
};
