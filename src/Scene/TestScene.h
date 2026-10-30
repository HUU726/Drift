#pragma once

#include "Scene/Scene.h"

class CameraObject;
class CarObject;
class PauseMenuObject;

// 試験コース。広い平面、S 字、角度の違うジャンプ台、ダートとアスファルトで、車の動きを試す。
// Esc でシーン選択の画面に戻る。R で車をスタート地点に戻し、P でポーズの画面（音量、タイトルへ戻る）を開く。Tab で調整の画面を開く。
class TestScene : public Scene {
public:
    explicit TestScene(System::Context& ctx) : Scene(ctx) {}
    bool Init() override;
    bool Paused() const override;

protected:
    void OnStartUpdate(float dt) override;
    void GetView(Mat4& viewProj, Vec3& eye) const override;

private:
    CarObject* car_ = nullptr;
    CameraObject* camera_ = nullptr;
    PauseMenuObject* pause_ = nullptr;
};
