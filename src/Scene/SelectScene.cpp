#include "Scene/SelectScene.h"

#include "GameObject/SceneSelectObject.h"
#include "System/Context.h"
#include "System/SoundManager.h"

bool SelectScene::Init() {
    // タイトルの曲にする（前の曲を少しずつ下げてから切り替わる）。
    Ctx().sound.PlayBgm(System::BgmTrack::Title);
    Add<SceneSelectObject>();
    return InitObjects();
}

void SelectScene::GetView(Mat4& viewProj, Vec3& eye) const {
    // 3D は描かない。
    viewProj = Mat4();
    eye = Vec3();
}
