#include "GameObject/PauseMenuObject.h"

#include "Scene/SelectScene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer2D.h"
#include "System/SceneManager.h"
#include "System/SoundManager.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

constexpr float kVolumeStep = 0.1f;  // ←→ 1 回で変える音量

}  // namespace

float PauseMenuObject::volume(int row) const {
    const System::SoundManager& sound = scene_.Ctx().sound;
    return row == kMusic ? sound.MusicVolume() : sound.EffectsVolume();
}

void PauseMenuObject::setVolume(int row, float value) {
    System::SoundManager& sound = scene_.Ctx().sound;
    if (row == kMusic) {
        sound.SetMusicVolume(value);
    } else {
        sound.SetEffectsVolume(value);
    }
}

void PauseMenuObject::StartUpdate(float dt) {
    (void)dt;
    System::Input& input = scene_.Ctx().input;
    System::SoundManager& sound = scene_.Ctx().sound;
    if (input.Pressed(System::Action::Pause)) {
        open_ = !open_;
        selected_ = kResume;
        return;
    }
    if (!open_) {
        return;
    }
    if (input.Pressed(System::Action::Back)) {
        open_ = false;
        return;
    }
    // 押した回数だけ動かす。選ぶ・値を変えるたびに短い音を鳴らす（効果音の音量の確かめにもなる）。
    int move = input.PressCount(System::Action::MenuDown) - input.PressCount(System::Action::MenuUp);
    if (move != 0) {
        selected_ = ((selected_ + move) % kRowCount + kRowCount) % kRowCount;
        sound.PlaySe(System::SoundEffect::Tick);
    }
    int direction = input.PressCount(System::Action::MenuRight) - input.PressCount(System::Action::MenuLeft);
    if (direction != 0 && (selected_ == kMusic || selected_ == kEffects)) {
        // 刻みの倍数に丸めて、足し算の誤差がたまらないようにする。
        float value = std::round((volume(selected_) + kVolumeStep * float(direction)) / kVolumeStep) * kVolumeStep;
        setVolume(selected_, std::clamp(value, 0.0f, 1.0f));
        sound.PlaySe(System::SoundEffect::Tick);
    }
    if (input.Pressed(System::Action::Confirm)) {
        if (selected_ == kResume) {
            open_ = false;
        } else if (selected_ == kTitle) {
            scene_.Ctx().scenes.Change<SelectScene>();
        }
    }
}

void PauseMenuObject::Draw2D(System::Renderer2D& hud) {
    if (!open_) {
        return;
    }
    const float w = float(hud.Width());
    const float h = float(hud.Height());
    hud.AddQuad(0, 0, w, h, 0.02f, 0.03f, 0.05f, 0.6f);

    const float left = w * 0.5f - 230;
    float y = h * 0.5f - 190;
    hud.DrawText(left, y, 5.0f, "PAUSE", 1, 0.95f, 0.9f, 1);
    y += 80;

    const char* names[kRowCount] = {"RESUME", "BGM VOLUME", "SE VOLUME", "BACK TO TITLE"};
    for (int i = 0; i < kRowCount; ++i) {
        const bool on = i == selected_;
        if (on) {
            hud.AddQuad(left - 16, y - 10, left + 476, y + 36, 0.95f, 0.8f, 0.35f, 0.18f);
            hud.DrawText(left - 4, y, 3.0f, ">", 0.95f, 0.8f, 0.35f, 1);
        }
        const float c = on ? 1.0f : 0.6f;
        hud.DrawText(left + 28, y, 3.0f, names[i], c, c, c, 1);
        if (i == kMusic || i == kEffects) {
            // 音量のバー（10 段）と、％の数字
            const float v = volume(i);
            const float barX = left + 270;
            const float barW = 130;
            hud.AddQuad(barX, y + 4, barX + barW, y + 18, 0.25f, 0.27f, 0.3f, 1);
            hud.AddQuad(barX, y + 4, barX + barW * v, y + 18, 0.95f * c, 0.8f * c, 0.35f * c, 1);
            char text[16];
            std::snprintf(text, sizeof(text), "%d", int(std::lround(v * 100.0f)));
            hud.DrawText(barX + barW + 14, y + 4, 2.0f, text, c, c, c, 1);
        }
        y += 56;
    }
    hud.DrawText(left, y + 10, 1.7f, "UP/DOWN SELECT  LEFT/RIGHT VOLUME  ENTER OK  P/ESC RESUME", 0.75f, 0.8f, 0.84f, 1);
}
