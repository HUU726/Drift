#include "Application.h"

#include "SelfTest.h"
#include "car.hpp"
#include "System/Input.h"
#include "System/Renderer2D.h"
#include "System/Renderer3D.h"
#include "track.hpp"

#include <SDL.h>
#include <SDL_image.h>
#include <SDL_mixer.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace {

constexpr double kFixedStepSec = 1.0 / 60.0;
constexpr double kMaxFrameSec = 0.25;

struct Race {
    int completed = 0;
    bool timing = false;
    float lap = 0;
    float best = -1;
    float prevS = -1;
};

struct Orbit {
    Vec3 eye{0, 8, -12};
    Vec3 look{0, 0, 0};
    bool ready = false;
    int mode = 0;
};

}  // namespace

struct App::State {
    SDL_Window* window = nullptr;
    SDL_GLContext gl = nullptr;
    System::Input input;
    System::Renderer3D renderer3d;
    System::Renderer2D renderer2d;
    Track track;
    Car car;
    System::GpuMesh course;
    System::GpuMesh body;
    System::GpuMesh wheel;
    System::GpuMesh shadow;
    Race race;
    Orbit camera;
    bool running = true;
    bool paused = false;
    bool showHelp = true;
    double accumulator = 0;
    Uint64 prevCounter = 0;
};

namespace {

std::string findTrackPath() {
    const char* paths[] = {
        "assets/track/suzuka.txt",
        "../assets/track/suzuka.txt",
        "/assets/track/suzuka.txt",
    };
    for (const char* path : paths) {
        std::ifstream in(path);
        if (in) {
            return path;
        }
    }
    return paths[0];
}

void formatTime(char* buf, int size, float seconds) {
    if (seconds < 0) {
        std::snprintf(buf, size, "--:--.---");
        return;
    }
    int minutes = static_cast<int>(seconds / 60.0f);
    float rest = seconds - float(minutes) * 60.0f;
    std::snprintf(buf, size, "%d:%06.3f", minutes, rest);
}

void resetRace(App::State& app) {
    app.car.reset(app.track);
    app.race = Race{};
    app.camera.ready = false;
}

void tickRace(Race& race, float s, float length, float dt) {
    if (race.prevS >= 0) {
        float gap = (length - race.prevS) + s;
        if (race.prevS > length * 0.5f && s < length * 0.5f && gap < 160.0f) {
            if (race.timing) {
                if (race.best < 0 || race.lap < race.best) {
                    race.best = race.lap;
                }
                race.completed += 1;
            }
            race.timing = true;
            race.lap = 0;
        }
    }
    race.prevS = s;
    race.lap += dt;
}

void updateCamera(App::State& app, float dt) {
    const float back[3] = {8.6f, 4.1f, 16.0f};
    const float up[3] = {3.3f, 1.55f, 7.2f};
    const float ahead[3] = {9.0f, 6.0f, 14.0f};
    int mode = app.camera.mode;
    Vec3 forward{std::sin(app.car.yaw), 0, std::cos(app.car.yaw)};
    Vec3 eye = Vec3{app.car.x, app.car.y, app.car.z} - forward * back[mode] + Vec3{0, up[mode], 0};
    Vec3 look = Vec3{app.car.x, app.car.y, app.car.z} + forward * ahead[mode] + Vec3{0, 1.15f, 0};
    TrackPose under = app.track.query(eye.x, eye.z, app.car.y, app.car.ground.s);
    if (under.found && under.xzDist < 30.0f && eye.y < under.centerY + 0.8f) {
        eye.y = under.centerY + 0.8f;
    }
    if (!app.camera.ready) {
        app.camera.eye = eye;
        app.camera.look = look;
        app.camera.ready = true;
        return;
    }
    float blend = 1.0f - std::exp(-dt * 7.0f);
    app.camera.eye = app.camera.eye + (eye - app.camera.eye) * blend;
    app.camera.look = app.camera.look + (look - app.camera.look) * blend;
}

void drawMinimap(App::State& app) {
    float size = 190.0f;
    float x0 = 24.0f;
    float y0 = float(app.renderer2d.Height()) - size - 28.0f;
    app.renderer2d.AddQuad(x0, y0, x0 + size, y0 + size, 0.02f, 0.03f, 0.05f, 0.78f);
    float minX = app.track.samples[0].x;
    float maxX = minX;
    float minZ = app.track.samples[0].z;
    float maxZ = minZ;
    for (const TrackSample& sample : app.track.samples) {
        minX = std::min(minX, sample.x);
        maxX = std::max(maxX, sample.x);
        minZ = std::min(minZ, sample.z);
        maxZ = std::max(maxZ, sample.z);
    }
    float midX = (minX + maxX) * 0.5f;
    float midZ = (minZ + maxZ) * 0.5f;
    float span = std::max(maxX - minX, maxZ - minZ) * 1.12f;
    auto mapX = [&](float x) { return x0 + size * 0.5f + (x - midX) / span * (size - 18.0f); };
    auto mapY = [&](float z) { return y0 + size * 0.5f - (z - midZ) / span * (size - 18.0f); };
    const int n = static_cast<int>(app.track.samples.size());
    for (int i = 0; i < n; ++i) {
        const TrackSample& a = app.track.samples[i];
        const TrackSample& b = app.track.samples[(i + 1) % n];
        float r = 0.75f, g = 0.78f, bl = 0.82f;
        if (a.bridge || b.bridge) {
            r = 0.95f;
            g = 0.72f;
            bl = 0.28f;
        }
        app.renderer2d.DrawLine(mapX(a.x), mapY(a.z), mapX(b.x), mapY(b.z), 2.0f, r, g, bl, 1);
    }
    float mx = mapX(app.car.x);
    float my = mapY(app.car.z);
    Vec3 nose{std::sin(app.car.yaw), 0, std::cos(app.car.yaw)};
    Vec3 side{std::cos(app.car.yaw), 0, -std::sin(app.car.yaw)};
    float nx = mx + nose.x * 9.0f;
    float ny = my - nose.z * 9.0f;
    float lx = mx - nose.x * 5.0f + side.x * 5.0f;
    float ly = my + nose.z * 5.0f - side.z * 5.0f;
    float rx = mx - nose.x * 5.0f - side.x * 5.0f;
    float ry = my + nose.z * 5.0f + side.z * 5.0f;
    app.renderer2d.DrawLine(lx, ly, nx, ny, 2.0f, 0.9f, 0.2f, 0.18f, 1);
    app.renderer2d.DrawLine(rx, ry, nx, ny, 2.0f, 0.9f, 0.2f, 0.18f, 1);
    app.renderer2d.DrawLine(lx, ly, rx, ry, 2.0f, 0.9f, 0.2f, 0.18f, 1);
}

void drawHud(App::State& app) {
    System::Renderer2D& hud = app.renderer2d;
    hud.Begin();
    char line[96];
    int kmh = static_cast<int>(std::lround(app.car.speed * 3.6f));
    std::snprintf(line, sizeof(line), "%d KM/H", kmh);
    hud.DrawText(24, 22, 4.0f, line, 0.95f, 0.96f, 0.98f, 1);

    if (!app.race.timing) {
        std::snprintf(line, sizeof(line), "OUT LAP");
    } else {
        std::snprintf(line, sizeof(line), "LAP %d", app.race.completed + 1);
    }
    hud.DrawText(24, 64, 2.2f, line, 0.85f, 0.88f, 0.92f, 1);
    formatTime(line, sizeof(line), app.race.lap);
    hud.DrawText(150, 64, 2.2f, line, 0.95f, 0.96f, 0.98f, 1);

    char best[32];
    formatTime(best, sizeof(best), app.race.best);
    std::snprintf(line, sizeof(line), "BEST %s", best);
    hud.DrawText(24, 90, 2.0f, line, 0.75f, 0.8f, 0.7f, 1);

    int percent = static_cast<int>(app.car.ground.s / app.track.length * 100.0f);
    const char* corner = app.track.nameOf(app.car.ground.nameId);
    if (app.car.ground.corner > 0 && corner[0]) {
        std::snprintf(line, sizeof(line), "T%d %s  %d%%", app.car.ground.corner, corner, percent);
    } else if (corner[0]) {
        std::snprintf(line, sizeof(line), "%s  %d%%", corner, percent);
    } else {
        std::snprintf(line, sizeof(line), "%d%%", percent);
    }
    hud.DrawText(24, 116, 2.0f, line, 0.9f, 0.9f, 0.92f, 1);

    const char* camName = "CHASE";
    if (app.camera.mode == 1) camName = "CLOSE";
    if (app.camera.mode == 2) camName = "FAR";
    hud.DrawText(float(hud.Width()) - 140, 22, 2.0f, camName, 0.8f, 0.84f, 0.88f, 1);

    if (app.showHelp) {
        hud.DrawText(24, float(hud.Height()) - 78, 1.7f,
                     "W/UP ACCEL  S/DOWN BRAKE  A/D STEER  R RESET  C CAMERA  P PAUSE",
                     0.9f, 0.92f, 0.94f, 1);
        hud.DrawText(24, float(hud.Height()) - 54, 1.7f,
                     "PAD: RT ACCEL  LT BRAKE  STICK STEER  Y CAMERA  BACK RESET  START PAUSE",
                     0.75f, 0.8f, 0.84f, 1);
    }
    hud.DrawText(float(hud.Width()) - 250, float(hud.Height()) - 28, 1.6f, "(C) OSM / GSI", 0.85f, 0.88f, 0.9f, 1);
    if (app.paused) {
        hud.DrawText(float(hud.Width()) * 0.5f - 70, float(hud.Height()) * 0.45f, 5.0f, "PAUSE", 1, 0.95f, 0.9f, 1);
    }
    drawMinimap(app);
    hud.Flush();
}

float carPitch(const App::State& app) {
    Vec3 forward{std::sin(app.car.yaw), 0, std::cos(app.car.yaw)};
    float preferS = app.car.ground.found ? app.car.ground.s : -1.0f;
    TrackPose ahead = app.track.query(app.car.x + forward.x * 2.4f, app.car.z + forward.z * 2.4f, app.car.y, preferS);
    TrackPose behind = app.track.query(app.car.x - forward.x * 2.4f, app.car.z - forward.z * 2.4f, app.car.y, preferS);
    if (!ahead.found || !behind.found) {
        return 0;
    }
    return std::atan2(ahead.centerY - behind.centerY, 4.8f);
}

void frame(void* userdata) {
    App::State& app = *static_cast<App::State*>(userdata);
    app.input.BeginFrame();
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            app.running = false;
        } else if (event.type == SDL_KEYDOWN && !event.key.repeat && event.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
            app.running = false;
        } else {
            app.input.OnEvent(event);
        }
    }
    app.input.Poll();
    const DriveInput& input = app.input.Drive();
    if (app.input.Pressed(System::Action::Reset)) resetRace(app);
    if (app.input.Pressed(System::Action::Pause)) app.paused = !app.paused;
    if (app.input.Pressed(System::Action::Camera)) app.camera.mode = (app.camera.mode + 1) % 3;
    if (app.input.Pressed(System::Action::Help)) app.showHelp = !app.showHelp;

    Uint64 now = SDL_GetPerformanceCounter();
    double elapsed = double(now - app.prevCounter) / double(SDL_GetPerformanceFrequency());
    app.prevCounter = now;
    if (elapsed > kMaxFrameSec) elapsed = kMaxFrameSec;
    if (elapsed < 0) elapsed = 0;

    if (app.paused) {
        app.accumulator = 0;
    } else {
        app.accumulator += elapsed;
        int guard = 0;
        while (app.accumulator >= kFixedStepSec && guard < 8) {
            app.car.update(float(kFixedStepSec), input, app.track);
            tickRace(app.race, app.car.ground.s, app.track.length, float(kFixedStepSec));
            app.accumulator -= kFixedStepSec;
            ++guard;
        }
    }
    updateCamera(app, float(elapsed));

    int drawableW = 0;
    int drawableH = 0;
    SDL_GL_GetDrawableSize(app.window, &drawableW, &drawableH);
    app.renderer3d.Resize(drawableW, drawableH);
    app.renderer2d.Resize(drawableW, drawableH);

    float aspect = float(drawableW) / float(drawableH > 0 ? drawableH : 1);
    Mat4 proj = perspective(60.0f * 3.14159265f / 180.0f, aspect, 0.15f, 4000.0f);
    Mat4 view = lookAt(app.camera.eye, app.camera.look, {0, 1, 0});
    app.renderer3d.BeginFrame(mul(proj, view), app.camera.eye);

    Mat4 identity;
    app.renderer3d.Draw(app.course, identity);
    Mat4 shadow = mul(translation(app.car.x, app.car.y + 0.08f, app.car.z), rotationY(app.car.yaw));
    app.renderer3d.Draw(app.shadow, shadow);

    float pitch = carPitch(app);
    float roll = std::atan(app.car.ground.cross);
    Mat4 model = carModelMatrix(app.car, pitch, roll);
    app.renderer3d.Draw(app.body, model);
    const float wheelX = 0.82f;
    const float wheelZ[2] = {1.42f, -1.38f};
    const float steerForWheel[2] = {app.car.steerAngle, 0};
    for (int i = 0; i < 2; ++i) {
        for (int side = -1; side <= 1; side += 2) {
            Mat4 place = mul(translation(float(side) * wheelX, 0.34f, wheelZ[i]),
                             mul(rotationY(steerForWheel[i]), rotationX(app.car.wheelSpin)));
            app.renderer3d.Draw(app.wheel, mul(model, place));
        }
    }
    drawHud(app);
    SDL_GL_SwapWindow(app.window);

#ifndef __EMSCRIPTEN__
    if (!app.running) {
        // ネイティブの while が抜ける。
    }
#endif
}

bool initGl(App::State& app) {
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
    app.window = SDL_CreateWindow("Suzuka Circuit", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720,
                                  SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN);
    if (!app.window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        return false;
    }
    app.gl = SDL_GL_CreateContext(app.window);
    if (!app.gl) {
        SDL_Log("SDL_GL_CreateContext failed: %s", SDL_GetError());
        return false;
    }
    SDL_GL_SetSwapInterval(1);
    if (!app.renderer3d.Init() || !app.renderer2d.Init()) {
        SDL_Log("GL init failed");
        return false;
    }
    return true;
}

}  // namespace

App::App() = default;
App::~App() = default;

bool App::Init(int argc, char** argv) {
    selfTest_ = argc > 1 && std::strcmp(argv[1], "--self-test") == 0;
    state_ = std::make_unique<State>();
    State& app = *state_;
    std::string path = findTrackPath();
    if (!app.track.load(path)) {
        std::fprintf(stderr, "コースを読めません: %s\n", path.c_str());
        return false;
    }
    if (selfTest_) {
        return true;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }
    imageOk_ = (IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG) != 0;
    if (!imageOk_) {
        SDL_Log("IMG_Init failed: %s", IMG_GetError());
    }
    audioOk_ = Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) == 0;
    if (!audioOk_) {
        SDL_Log("Mix_OpenAudio failed: %s", Mix_GetError());
    }

    if (!initGl(app)) {
        SDL_Quit();
        return false;
    }
    app.input.Init();
    app.course = app.renderer3d.Upload(app.track.visual);
    CpuMesh body;
    CpuMesh wheel;
    CpuMesh shadow;
    buildCarBody(body);
    buildCarWheel(wheel);
    buildCarShadow(shadow);
    app.body = app.renderer3d.Upload(body);
    app.wheel = app.renderer3d.Upload(wheel);
    app.shadow = app.renderer3d.Upload(shadow);
    app.car.reset(app.track);
    app.prevCounter = SDL_GetPerformanceCounter();
    return true;
}

int App::Run() {
    State& app = *state_;
    if (selfTest_) {
        return RunSelfTest(app.track);
    }
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(frame, &app, 0, 1);
#else
    while (app.running) {
        Uint64 frameStart = SDL_GetPerformanceCounter();
        frame(&app);
        double frameMs = double(SDL_GetPerformanceCounter() - frameStart) / double(SDL_GetPerformanceFrequency()) * 1000.0;
        if (frameMs < 14.0) {
            SDL_Delay(static_cast<Uint32>(14.0 - frameMs));
        }
    }
#endif
    return 0;
}

void App::UnInit() {
    if (selfTest_ || !state_) {
        return;
    }
    State& app = *state_;
    app.input.UnInit();
    if (audioOk_) {
        Mix_CloseAudio();
        Mix_Quit();
    }
    if (imageOk_) IMG_Quit();
    app.renderer3d.Release(app.course);
    app.renderer3d.Release(app.body);
    app.renderer3d.Release(app.wheel);
    app.renderer3d.Release(app.shadow);
    app.renderer2d.UnInit();
    app.renderer3d.UnInit();
    SDL_GL_DeleteContext(app.gl);
    SDL_DestroyWindow(app.window);
    SDL_Quit();
}
