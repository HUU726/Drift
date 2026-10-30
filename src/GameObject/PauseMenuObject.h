#pragma once

#include "GameObject/GameObject.h"

// ポーズの画面。P / Start で開閉し、開いている間はシーンが止まる（シーンの Paused が IsOpen を返す）。
// ↑↓ で選び、←→ で BGM と効果音の音量を変え、Enter / A で決める。Esc / B でも閉じる。
// 項目は、続ける、BGM の音量、効果音の音量、タイトル（シーン選択の画面）へ戻る、の 4 つ。
class PauseMenuObject : public GameObject {
public:
    explicit PauseMenuObject(Scene& scene) : GameObject(scene) { order = 120; }
    void StartUpdate(float dt) override;
    void Draw2D(System::Renderer2D& hud) override;

    bool IsOpen() const { return open_; }

private:
    enum Row { kResume, kMusic, kEffects, kTitle, kRowCount };

    // 音量の項目の値（0..1）を読む・書く。
    float volume(int row) const;
    void setVolume(int row, float value);

    bool open_ = false;
    int selected_ = kResume;
};
