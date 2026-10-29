#pragma once

#include "GameObject/TrackObject.h"
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

    TrackObject::Course course_ = TrackObject::Course::Suzuka;  // 走るコース。鈴鹿のときだけジャンプ台を置く

private:
    CarObject* car_ = nullptr;
    RaceTimerObject* timer_ = nullptr;
    CameraObject* camera_ = nullptr;
    bool paused_ = false;
};

// 峠。山頂から下ってヘアピンを抜け、谷から登って戻る 1 周を、レースと同じ HUD とラップ計測で走る。
class TougeScene : public RaceScene {
public:
    explicit TougeScene(System::Context& ctx) : RaceScene(ctx) { course_ = TrackObject::Course::Touge; }
};

// 走り比べ。形の違う S 字とヘアピンを並べた平らな 1 周を走り、どの形がよいかを決める。区間の名前が HUD に出る。
class LabScene : public RaceScene {
public:
    explicit LabScene(System::Context& ctx) : RaceScene(ctx) { course_ = TrackObject::Course::Lab; }
};
