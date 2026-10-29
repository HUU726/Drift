#include "Application.h"

#include "GameObject/TrackObject.h"
#include "Scene/SelectScene.h"
#include "SelfTest.h"

#include <SDL_image.h>

#include <cstdio>
#include <cstring>
#include <string>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace {

constexpr double kFixedStepSec = 1.0 / 60.0;
constexpr double kMaxFrameSec = 0.25;

}  // namespace

bool App::Init(int argc, char** argv) {
    selfTest_ = argc > 1 && std::strcmp(argv[1], "--self-test") == 0;
    if (selfTest_) {
        std::string path = TrackObject::FindPath();
        if (!selfTestTrack_.load(path)) {
            std::fprintf(stderr, "コースを読めません: %s\n", path.c_str());
            return false;
        }
        return true;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }
    sdlOk_ = true;
    imageOk_ = (IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG) != 0;
    if (!imageOk_) {
        SDL_Log("IMG_Init failed: %s", IMG_GetError());
    }

    // System のマネージャは Renderer3D → Renderer2D → SoundManager → ColliderManager → SceneManager の順に
    // 初期化し、逆順に解放する。シーンの Init で Collider を登録するので、ColliderManager はシーンより先。
    if (!initGl()) {
        UnInit();
        return false;
    }
    sound_.Init();
    physics_.Init();
    input_.Init();
    if (!scenes_.Init<SelectScene>(ctx_)) {
        UnInit();
        return false;
    }
    prevCounter_ = SDL_GetPerformanceCounter();
    return true;
}

bool App::initGl() {
#ifdef __EMSCRIPTEN__
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#endif
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    window_ = SDL_CreateWindow("Suzuka Circuit", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
                               SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN);
    if (!window_) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        return false;
    }
    gl_ = SDL_GL_CreateContext(window_);
    if (!gl_) {
        SDL_Log("SDL_GL_CreateContext failed: %s", SDL_GetError());
        return false;
    }
    SDL_GL_SetSwapInterval(1);
    if (!renderer3d_.Init() || !renderer2d_.Init()) {
        SDL_Log("GL init failed");
        return false;
    }
    return true;
}

int App::Run() {
    if (selfTest_) {
        return RunSelfTest(selfTestTrack_);
    }
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(frameThunk, this, 0, 1);
#else
    while (running_) {
        Uint64 frameStart = SDL_GetPerformanceCounter();
        frame();
        double frameMs = double(SDL_GetPerformanceCounter() - frameStart) / double(SDL_GetPerformanceFrequency()) * 1000.0;
        if (frameMs < 14.0) {
            SDL_Delay(static_cast<Uint32>(14.0 - frameMs));
        }
    }
#endif
    return 0;
}

void App::frameThunk(void* userdata) {
    static_cast<App*>(userdata)->frame();
}

void App::frame() {
    input_.BeginFrame();
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            running_ = false;
        } else {
            input_.OnEvent(event);
        }
    }
    input_.Poll();

    Uint64 now = SDL_GetPerformanceCounter();
    double elapsed = double(now - prevCounter_) / double(SDL_GetPerformanceFrequency());
    prevCounter_ = now;
    if (elapsed > kMaxFrameSec) elapsed = kMaxFrameSec;
    if (elapsed < 0) elapsed = 0;

    scenes_.StartUpdate(float(elapsed));
    if (scenes_.Paused()) {
        accumulator_ = 0;
    } else {
        accumulator_ += elapsed;
        int guard = 0;
        while (accumulator_ >= kFixedStepSec && guard < 8) {
            scenes_.FixedUpdate(float(kFixedStepSec));
            physics_.Step(float(kFixedStepSec));
            scenes_.PostPhysicsUpdate(float(kFixedStepSec));
            accumulator_ -= kFixedStepSec;
            ++guard;
        }
    }
    scenes_.LateUpdate(float(elapsed));

    int drawableW = 0;
    int drawableH = 0;
    SDL_GL_GetDrawableSize(window_, &drawableW, &drawableH);
    renderer3d_.Resize(drawableW, drawableH);
    renderer2d_.Resize(drawableW, drawableH);
    scenes_.Draw();
    SDL_GL_SwapWindow(window_);

    // シーンの入れ替えは、更新の途中で自分のシーンを壊さないようにフレームの最後で行う。
    if (!scenes_.ApplyPending() || scenes_.QuitRequested()) {
        running_ = false;
    }
}

void App::UnInit() {
    if (selfTest_ || !sdlOk_) {
        return;
    }
    scenes_.UnInit();
    input_.UnInit();
    physics_.UnInit();
    sound_.UnInit();
    if (imageOk_) IMG_Quit();
    if (gl_) {
        renderer2d_.UnInit();
        renderer3d_.UnInit();
        SDL_GL_DeleteContext(gl_);
        gl_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    SDL_Quit();
    sdlOk_ = false;
}
