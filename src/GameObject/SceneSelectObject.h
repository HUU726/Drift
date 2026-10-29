#pragma once

#include "GameObject/GameObject.h"

#include <functional>
#include <vector>

// シーン選択の一覧。↑↓ で選び、Enter で入る。Esc でアプリを終える（ブラウザでは何もしない）。
// シーンを増やすときは、buildEntries に 1 行足す。
class SceneSelectObject : public GameObject {
public:
    explicit SceneSelectObject(Scene& scene) : GameObject(scene) {}
    bool Init() override;
    void StartUpdate(float dt) override;
    void Draw2D(System::Renderer2D& hud) override;

private:
    struct Entry {
        const char* name;         // 画面に出す名前
        const char* description;  // 一言の説明
        std::function<void()> enter;
    };

    std::vector<Entry> entries_;
    static int selected_;  // 戻ってきたとき、前に選んだシーンを選んだままにする
};
