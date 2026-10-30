#include "Scene/RaceScene.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "GameObject/HudObject.h"
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
    if (input.Pressed(System::Action::Pause)) {
        paused_ = !paused_;
    }
    if (input.Pressed(System::Action::Back)) {
        Ctx().scenes.Change<SelectScene>();
    }
}

void RaceScene::GetView(Mat4& viewProj, Vec3& eye) const {
    viewProj = camera_->ViewProj();
    eye = camera_->Eye();
}
