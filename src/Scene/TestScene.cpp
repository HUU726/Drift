#include "Scene/TestScene.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "GameObject/PauseMenuObject.h"
#include "GameObject/SmokeObject.h"
#include "GameObject/TelemetryHudObject.h"
#include "GameObject/TestGroundObject.h"
#include "GameObject/TuningObject.h"
#include "Scene/SelectScene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/SceneManager.h"
#include "System/SoundManager.h"

bool TestScene::Init() {
    // プレイ中の曲にする（前の曲を少しずつ下げてから切り替わる）。
    Ctx().sound.PlayBgm(System::BgmTrack::Race);
    Add<TestGroundObject>();
    car_ = Add<CarObject>();
    TestCourse layout;  // 配置の既定値（スタート地点）を読むだけ
    car_->SetSpawn(layout.spawn, layout.spawnYaw);
    camera_ = Add<CameraObject>();
    Add<SmokeObject>();
    Add<TelemetryHudObject>();
    Add<TuningObject>();
    pause_ = Add<PauseMenuObject>();
    return InitObjects();
}

void TestScene::OnStartUpdate(float dt) {
    (void)dt;
    System::Input& input = Ctx().input;
    if (input.Pressed(System::Action::Reset)) {
        car_->Reset();
        camera_->Snap();
    }
    // ポーズの画面が開いている間の Esc は、画面を閉じる（PauseMenuObject が扱う）。
    if (input.Pressed(System::Action::Back) && !pause_->IsOpen()) {
        Ctx().scenes.Change<SelectScene>();
    }
}

void TestScene::GetView(Mat4& viewProj, Vec3& eye) const {
    viewProj = camera_->ViewProj();
    eye = camera_->Eye();
}

bool TestScene::Paused() const {
    return pause_ && pause_->IsOpen();
}
