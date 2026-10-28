#include "System/Input.h"

#include "geom.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#ifdef __EMSCRIPTEN__
#include <emscripten/html5.h>
#endif

namespace System {

namespace {

float stickDead(float value) {
    const float dead = 0.14f;
    if (std::fabs(value) < dead) {
        return 0;
    }
    float sign = value > 0 ? 1.0f : -1.0f;
    return (value - sign * dead) / (1.0f - dead);
}

#ifndef __EMSCRIPTEN__
float stickAxis(SDL_GameController* pad, SDL_GameControllerAxis axis) {
    return stickDead(SDL_GameControllerGetAxis(pad, axis) / 32767.0f);
}
#endif

#ifdef __EMSCRIPTEN__
// ブラウザの Gamepad API は 0 が離した状態、1 が押し込み。古いパッドは -1..1 で、離すと -1。
float triggerLevel(float value) {
    if (value < 0) {
        value = (value + 1.0f) * 0.5f;
    }
    return value < 0.04f ? 0.0f : clampf(value, 0.0f, 1.0f);
}
#endif

}  // namespace

bool Input::Init() {
    openPad();
    return true;
}

void Input::UnInit() {
    if (pad_) {
        SDL_GameControllerClose(pad_);
        pad_ = nullptr;
    }
}

void Input::BeginFrame() {
    std::fill(std::begin(pressed_), std::end(pressed_), false);
}

void Input::OnEvent(const SDL_Event& event) {
    if (event.type == SDL_KEYDOWN && !event.key.repeat) {
        SDL_Scancode code = event.key.keysym.scancode;
        if (code == SDL_SCANCODE_R) press(Action::Reset);
        else if (code == SDL_SCANCODE_C) press(Action::Camera);
        else if (code == SDL_SCANCODE_P) press(Action::Pause);
        else if (code == SDL_SCANCODE_H) press(Action::Help);
    } else if (event.type == SDL_CONTROLLERDEVICEADDED) {
        openPad();
    } else if (event.type == SDL_CONTROLLERDEVICEREMOVED && pad_ &&
               event.cdevice.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(pad_))) {
        SDL_GameControllerClose(pad_);
        pad_ = nullptr;
    } else if (event.type == SDL_CONTROLLERBUTTONDOWN) {
        if (event.cbutton.button == SDL_CONTROLLER_BUTTON_BACK) press(Action::Reset);
        else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_START) press(Action::Pause);
        else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_Y) press(Action::Camera);
    }
}

void Input::Poll() {
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    DriveInput input;
    if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) input.throttle = 1;
    if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) input.brake = 1;
    if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) input.steer -= 1;
    if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) input.steer += 1;
#ifndef __EMSCRIPTEN__
    if (pad_) {
        float throttle = stickAxis(pad_, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        float brake = stickAxis(pad_, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        if (throttle < 0) throttle = 0;
        if (brake < 0) brake = 0;
        input.throttle = std::max(input.throttle, throttle);
        input.brake = std::max(input.brake, brake);
        input.steer += stickAxis(pad_, SDL_CONTROLLER_AXIS_LEFTX);
        input.steer += stickAxis(pad_, SDL_CONTROLLER_AXIS_RIGHTX);
        if (SDL_GameControllerGetButton(pad_, SDL_CONTROLLER_BUTTON_DPAD_UP)) input.throttle = 1;
        if (SDL_GameControllerGetButton(pad_, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) input.brake = 1;
        if (SDL_GameControllerGetButton(pad_, SDL_CONTROLLER_BUTTON_DPAD_LEFT)) input.steer -= 1;
        if (SDL_GameControllerGetButton(pad_, SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) input.steer += 1;
    }
#endif
    drive_ = input;
#ifdef __EMSCRIPTEN__
    // SDL はブラウザのパッドをゲームコントローラと認識しないことが多い。
    // 標準マッピングを Gamepad API から直接読む。
    readBrowserPad();
#endif
    drive_.throttle = clampf(drive_.throttle, 0, 1);
    drive_.brake = clampf(drive_.brake, 0, 1);
    drive_.steer = clampf(drive_.steer, -1, 1);
}

void Input::openPad() {
#ifdef __EMSCRIPTEN__
    // ブラウザのパッドは Gamepad API 側で読む。SDL で開くと入力が二重になる。
    return;
#else
    if (pad_) {
        return;
    }
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            pad_ = SDL_GameControllerOpen(i);
            if (pad_) {
                SDL_Log("controller: %s", SDL_GameControllerName(pad_));
                break;
            }
        }
    }
#endif
}

#ifdef __EMSCRIPTEN__
void Input::readBrowserPad() {
    DriveInput& input = drive_;
    bool camera = false;
    bool reset = false;
    bool pause = false;
    if (emscripten_sample_gamepad_data() != EMSCRIPTEN_RESULT_SUCCESS) {
        prevCamera_ = prevReset_ = prevPause_ = false;
        return;
    }
    const int count = emscripten_get_num_gamepads();
    for (int i = 0; i < count; ++i) {
        EmscriptenGamepadEvent pad;
        if (emscripten_get_gamepad_status(i, &pad) != EMSCRIPTEN_RESULT_SUCCESS || !pad.connected) {
            continue;
        }
        auto axis = [&](int index) {
            return index < pad.numAxes ? static_cast<float>(pad.axis[index]) : 0.0f;
        };
        auto analog = [&](int index) {
            return index < pad.numButtons ? static_cast<float>(pad.analogButton[index]) : 0.0f;
        };
        auto held = [&](int index) {
            return index < pad.numButtons && pad.digitalButton[index];
        };
        const bool standard = std::strcmp(pad.mapping, "standard") == 0 || (pad.numButtons >= 16 && pad.numAxes >= 2);
        input.steer += stickDead(axis(0));
        if (standard) {
            input.steer += stickDead(axis(2));
            if (held(14)) input.steer -= 1;
            if (held(15)) input.steer += 1;
            if (held(12)) input.throttle = 1;
            if (held(13)) input.brake = 1;
            float throttle = analog(7);
            float brake = analog(6);
            if (throttle < 0.01f && pad.numAxes > 5) throttle = triggerLevel(axis(5));
            if (brake < 0.01f && pad.numAxes > 4) brake = triggerLevel(axis(4));
            input.throttle = std::max(input.throttle, throttle);
            input.brake = std::max(input.brake, brake);
            camera = camera || held(3);
            reset = reset || held(8);
            pause = pause || held(9);
        } else if (pad.numAxes >= 6) {
            input.throttle = std::max(input.throttle, triggerLevel(axis(5)));
            input.brake = std::max(input.brake, triggerLevel(axis(4)));
        }
    }
    if (camera && !prevCamera_) press(Action::Camera);
    if (reset && !prevReset_) press(Action::Reset);
    if (pause && !prevPause_) press(Action::Pause);
    prevCamera_ = camera;
    prevReset_ = reset;
    prevPause_ = pause;
}
#endif

}  // namespace System
