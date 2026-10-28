#include "Scene/RaceScene.h"

#include "GameObject/CameraObject.h"
#include "GameObject/CarObject.h"
#include "GameObject/HudObject.h"
#include "GameObject/RaceTimerObject.h"
#include "GameObject/RampObject.h"
#include "GameObject/TrackObject.h"
#include "System/Context.h"
#include "System/Input.h"

bool RaceScene::Init() {
    Add<TrackObject>();
    Add<RampObject>();
    car_ = Add<CarObject>();
    timer_ = Add<RaceTimerObject>();
    camera_ = Add<CameraObject>();
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
}

void RaceScene::GetView(Mat4& viewProj, Vec3& eye) const {
    viewProj = camera_->ViewProj();
    eye = camera_->Eye();
}
