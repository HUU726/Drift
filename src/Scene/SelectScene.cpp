#include "Scene/SelectScene.h"

#include "GameObject/SceneSelectObject.h"

bool SelectScene::Init() {
    Add<SceneSelectObject>();
    return InitObjects();
}

void SelectScene::GetView(Mat4& viewProj, Vec3& eye) const {
    // 3D は描かない。
    viewProj = Mat4();
    eye = Vec3();
}
