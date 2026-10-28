#include "System/SoundManager.h"

#include <SDL.h>
#include <SDL_mixer.h>

namespace System {

bool SoundManager::Init() {
    open_ = Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) == 0;
    if (!open_) {
        SDL_Log("Mix_OpenAudio failed: %s", Mix_GetError());
    }
    return true;
}

void SoundManager::UnInit() {
    if (open_) {
        Mix_CloseAudio();
        Mix_Quit();
        open_ = false;
    }
}

}  // namespace System
