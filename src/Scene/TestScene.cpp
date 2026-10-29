#include "Scene/TestScene.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "GameObject/SmokeObject.h"
#include "GameObject/TelemetryHudObject.h"
#include "GameObject/TestGroundObject.h"
#include "GameObject/TuningObject.h"
#include "Scene/SelectScene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/SceneManager.h"

bool TestScene::Init() {
    Add<TestGroundObject>();
    car_ = Add<CarObject>();
    TestCourse layout;  // 配置の既定値（スタート地点）を読むだけ
    car_->SetSpawn(layout.spawn, layout.spawnYaw);
    camera_ = Add<CameraObject>();
    Add<SmokeObject>();
    Add<TelemetryHudObject>();
    Add<TuningObject>();
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
    if (input.Pressed(System::Action::Back)) {
        Ctx().scenes.Change<SelectScene>();
    }
}

void TestScene::GetView(Mat4& viewProj, Vec3& eye) const {
    viewProj = camera_->ViewProj();
    eye = camera_->Eye();
}
