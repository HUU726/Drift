#include "Scene/TestScene.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "GameObject/TelemetryHudObject.h"
#include "GameObject/TestGroundObject.h"
#include "Scene/RaceScene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/SceneManager.h"

bool TestScene::Init() {
    Add<TestGroundObject>();
    car_ = Add<CarObject>();
    TestCourse layout;  // 配置の既定値（スタート地点）を読むだけ
    car_->SetSpawn(layout.spawn, layout.spawnYaw);
    camera_ = Add<CameraObject>();
    Add<TelemetryHudObject>();
    return InitObjects();
}

void TestScene::OnStartUpdate(float dt) {
    (void)dt;
    System::Input& input = Ctx().input;
    if (input.Pressed(System::Action::Reset)) {
        car_->Reset();
        camera_->Snap();
    }
    if (input.Pressed(System::Action::Pause)) {
        paused_ = !paused_;
    }
    if (input.Pressed(System::Action::SwitchScene)) {
        Ctx().scenes.Change<RaceScene>();
    }
}

void TestScene::GetView(Mat4& viewProj, Vec3& eye) const {
    viewProj = camera_->ViewProj();
    eye = camera_->Eye();
}
