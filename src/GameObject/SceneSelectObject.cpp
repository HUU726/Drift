#include "GameObject/SceneSelectObject.h"

#include "Scene/RaceScene.h"
#include "Scene/TestScene.h"
#include "System/Context.h"
#include "System/Input.h"
#include "System/Renderer2D.h"
#include "System/SceneManager.h"

int SceneSelectObject::selected_ = 0;

bool SceneSelectObject::Init() {
    System::SceneManager& scenes = scene_.Ctx().scenes;
    entries_ = {
        {"RACE", "LAP THE CIRCUIT", [&scenes] { scenes.Change<RaceScene>(); }},
        {"TEST COURSE", "DEBUG AND TUNE THE CAR  (TAB TUNING)", [&scenes] { scenes.Change<TestScene>(); }},
    };
    if (selected_ >= static_cast<int>(entries_.size())) {
        selected_ = 0;
    }
    return true;
}

void SceneSelectObject::StartUpdate(float dt) {
    (void)dt;
    System::Input& input = scene_.Ctx().input;
    const int count = static_cast<int>(entries_.size());
    int move = input.PressCount(System::Action::MenuDown) - input.PressCount(System::Action::MenuUp);
    selected_ = ((selected_ + move) % count + count) % count;
    if (input.Pressed(System::Action::Confirm)) {
        entries_[selected_].enter();
    }
#ifndef __EMSCRIPTEN__
    if (input.Pressed(System::Action::Back)) {
        scene_.Ctx().scenes.RequestQuit();
    }
#endif
}

void SceneSelectObject::Draw2D(System::Renderer2D& hud) {
    const float w = float(hud.Width());
    const float h = float(hud.Height());
    hud.AddQuad(0, 0, w, h, 0.05f, 0.06f, 0.08f, 0.85f);

    const float left = w * 0.5f - 260;
    float y = h * 0.28f;
    hud.DrawText(left, y, 6.0f, "DRIFT", 0.95f, 0.96f, 0.98f, 1);
    hud.DrawText(left, y + 70, 2.0f, "SELECT SCENE", 0.95f, 0.8f, 0.35f, 1);

    y += 130;
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
        const bool on = i == selected_;
        if (on) {
            hud.AddQuad(left - 16, y - 10, left + 540, y + 62, 0.95f, 0.8f, 0.35f, 0.18f);
            hud.DrawText(left - 4, y, 3.0f, ">", 0.95f, 0.8f, 0.35f, 1);
        }
        float c = on ? 1.0f : 0.6f;
        hud.DrawText(left + 28, y, 3.0f, entries_[i].name, c, c, c, 1);
        hud.DrawText(left + 28, y + 34, 1.7f, entries_[i].description, c * 0.85f, c * 0.88f, c * 0.92f, 1);
        y += 88;
    }

#ifdef __EMSCRIPTEN__
    const char* help = "UP/DOWN SELECT  ENTER START  (ESC IN A SCENE RETURNS HERE)";
#else
    const char* help = "UP/DOWN SELECT  ENTER START  ESC QUIT  (ESC IN A SCENE RETURNS HERE)";
#endif
    hud.DrawText(24, h - 54, 1.7f, help, 0.75f, 0.8f, 0.84f, 1);
}
