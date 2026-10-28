#include "System/SceneManager.h"

#include <SDL.h>

namespace System {

void SceneManager::UnInit() {
    if (current_) {
        current_->UnInit();
        current_.reset();
    }
    pending_.reset();
}

void SceneManager::StartUpdate(float dt) {
    if (current_) current_->StartUpdate(dt);
}

void SceneManager::FixedUpdate(float dt) {
    if (current_) current_->FixedUpdate(dt);
}

void SceneManager::PostPhysicsUpdate(float dt) {
    if (current_) current_->PostPhysicsUpdate(dt);
}

void SceneManager::LateUpdate(float dt) {
    if (current_) current_->LateUpdate(dt);
}

void SceneManager::Draw() {
    if (current_) current_->Draw();
}

bool SceneManager::Paused() const {
    return current_ && current_->Paused();
}

bool SceneManager::ApplyPending() {
    if (!pending_) {
        return true;
    }
    if (current_) {
        current_->UnInit();
    }
    current_ = std::move(pending_);
    if (!current_->Init()) {
        SDL_Log("scene init failed");
        current_->UnInit();
        current_.reset();
        return false;
    }
    return true;
}

}  // namespace System
