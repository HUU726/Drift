#include "Scene/RaceScene.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "GameObject/HudObject.h"
#include "GameObject/PauseMenuObject.h"
#include "GameObject/RaceTimerObject.h"
#include "GameObject/RampObject.h"
#include "GameObject/SmokeObject.h"
#include "GameObject/TrackObject.h"
#include "Scene/SelectScene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/SceneManager.h"
#include "System/SoundManager.h"

bool RaceScene::Init() {
    // プレイ中の曲にする（前の曲を少しずつ下げてから切り替わる）。
    Ctx().sound.PlayBgm(System::BgmTrack::Race);
    TrackObject* track = Add<TrackObject>();
    track->SetCourse(course_);
    if (course_ == TrackObject::Course::Suzuka) {
        Add<RampObject>();
    }
    car_ = Add<CarObject>();
    timer_ = Add<RaceTimerObject>();
    camera_ = Add<CameraObject>();
    Add<SmokeObject>();
    Add<HudObject>();
    pause_ = Add<PauseMenuObject>();
    return InitObjects();
}

void RaceScene::OnStartUpdate(float dt) {
    (void)dt;
    System::Input& input = Ctx().input;
    if (input.Pressed(System::Action::Reset)) {
        car_->Reset();
        timer_->Reset();
        camera_->Snap();
    }
    // ポーズの画面が開いている間の Esc は、画面を閉じる（PauseMenuObject が扱う）。
    if (input.Pressed(System::Action::Back) && !pause_->IsOpen()) {
        Ctx().scenes.Change<SelectScene>();
    }
}

void RaceScene::GetView(Mat4& viewProj, Vec3& eye) const {
    viewProj = camera_->ViewProj();
    eye = camera_->Eye();
}

bool RaceScene::Paused() const {
    return pause_ && pause_->IsOpen();
}
