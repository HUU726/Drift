#pragma once

#include "System/ColliderManager.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer2D.h"
#include "System/Renderer3D.h"
#include "System/SceneManager.h"
#include "System/SoundManager.h"
#include "track.hpp"

#include <SDL.h>

// アプリケーション全体。main からは Init / Run / UnInit だけを呼ぶ。
// SDL、ウィンドウ、GL コンテキスト、メインループ、固定ステップを受け持ち、System のマネージャを持つ。
class App {
public:
    bool Init(int argc, char** argv);
    // --self-test のときはテスト結果を返す。ブラウザ版では戻らない。
    int Run();
    void UnInit();

private:
    static void frameThunk(void* userdata);
    void frame();
    bool initGl();

    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_ = nullptr;
    System::Renderer3D renderer3d_;
    System::Renderer2D renderer2d_;
    System::SoundManager sound_;
    System::ColliderManager physics_;
    System::Input input_;
    System::SceneManager scenes_;
    System::Context ctx_{renderer3d_, renderer2d_, sound_, physics_, scenes_, input_};

    Track selfTestTrack_;  // --self-test のときだけ使う
    bool selfTest_ = false;
    bool sdlOk_ = false;
    bool imageOk_ = false;
    bool running_ = true;
    double accumulator_ = 0;
    Uint64 prevCounter_ = 0;
};
