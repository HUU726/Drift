#pragma once

#include "car.hpp"

#include <SDL.h>

namespace System {

// 押した瞬間だけ立つ操作。運転の入力（DriveInput）とは別に扱う。
enum class Action {
    Reset,
    Pause,
    Camera,
    Help,
    Back,     // Esc / パッドの B。シーン選択の画面へ戻る（シーン選択の画面では終了）
    Confirm,  // Enter / Space / パッドの A。シーン選択の画面で決める
    // 調整の画面（試験コース）。上下左右は押し続けると繰り返す。
    MenuToggle,
    MenuDetail,  // F。調整の画面で、基本の項目と細かい項目を切り替える
    MenuUp,
    MenuDown,
    MenuLeft,
    MenuRight,
    Export,
    Count,
};

// キーボードとパッドの入力。1 フレームの流れは BeginFrame → OnEvent（イベントごと）→ Poll。
// SDL_QUIT は App が処理する。
class Input {
public:
    bool Init();
    void UnInit();
    void BeginFrame();
    void OnEvent(const SDL_Event& event);
    void Poll();

    const DriveInput& Drive() const { return drive_; }
    bool Pressed(Action action) const { return pressed_[static_cast<int>(action)] > 0; }
    // このフレームで押された回数。フレームの間に何度も押したとき（キーの繰り返しなど）も数える。
    int PressCount(Action action) const { return pressed_[static_cast<int>(action)]; }
    // true の間は矢印キーを運転に使わない（調整の画面の操作に使う）。運転は WASD でできる。
    void SetMenuMode(bool menu) { menuMode_ = menu; }

private:
    void openPad();
    void press(Action action) { ++pressed_[static_cast<int>(action)]; }
#ifdef __EMSCRIPTEN__
    void readBrowserPad();
#endif

    SDL_GameController* pad_ = nullptr;
    DriveInput drive_;
    bool menuMode_ = false;
    int pressed_[static_cast<int>(Action::Count)] = {};
#ifdef __EMSCRIPTEN__
    bool prevCamera_ = false;
    bool prevReset_ = false;
    bool prevPause_ = false;
    bool prevBack_ = false;
    bool prevConfirm_ = false;
    bool prevUp_ = false;
    bool prevDown_ = false;
    bool prevLeft_ = false;
    bool prevRight_ = false;
#endif
};

}  // namespace System
