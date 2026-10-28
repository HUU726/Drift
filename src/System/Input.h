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
    Count,
};

// キーボードとパッドの入力。1 フレームの流れは BeginFrame → OnEvent（イベントごと）→ Poll。
// SDL_QUIT と Esc は App が処理する。
class Input {
public:
    bool Init();
    void UnInit();
    void BeginFrame();
    void OnEvent(const SDL_Event& event);
    void Poll();

    const DriveInput& Drive() const { return drive_; }
    bool Pressed(Action action) const { return pressed_[static_cast<int>(action)]; }

private:
    void openPad();
    void press(Action action) { pressed_[static_cast<int>(action)] = true; }
#ifdef __EMSCRIPTEN__
    void readBrowserPad();
#endif

    SDL_GameController* pad_ = nullptr;
    DriveInput drive_;
    bool pressed_[static_cast<int>(Action::Count)] = {};
#ifdef __EMSCRIPTEN__
    bool prevCamera_ = false;
    bool prevReset_ = false;
    bool prevPause_ = false;
#endif
};

}  // namespace System
