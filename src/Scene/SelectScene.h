#pragma once

#include "Scene/Scene.h"

// シーン選択の画面。起動するとここから始まり、各シーンから Esc で戻る。
// 一覧は SceneSelectObject が持ち、↑↓ で選んで Enter で入る。ここで Esc を押すとアプリを終える。
class SelectScene : public Scene {
public:
    explicit SelectScene(System::Context& ctx) : Scene(ctx) {}
    bool Init() override;

protected:
    void GetView(Mat4& viewProj, Vec3& eye) const override;
};
