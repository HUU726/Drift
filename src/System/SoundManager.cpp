#include "System/SoundManager.h"

#include <SDL_mixer.h>

#include <algorithm>
#include <cmath>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace System {

bool SoundManager::Init() {
#ifndef __EMSCRIPTEN__
    // ネイティブ版はすぐ開く。ブラウザ版は最初の操作を待つ（OnEvent）。
    open();
#endif
    return true;
}

void SoundManager::open() {
    tried_ = true;
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024) != 0) {
        SDL_Log("Mix_OpenAudio failed: %s", Mix_GetError());
        return;
    }
    // 実際に開けた形式に合わせる（ブラウザでは周波数が 48000 Hz などに変わる）。
    int freq = 44100;
    Uint16 format = 0;
    int channels = 2;
    Mix_QuerySpec(&freq, &format, &channels);
    if (format != AUDIO_S16SYS && format != AUDIO_F32SYS) {
        SDL_Log("unsupported audio format 0x%x", unsigned(format));
        Mix_CloseAudio();
        return;
    }
    format_ = format;
    channels_ = std::max(1, channels);
    synth_.SetSampleRate(freq);
    buffer_.resize(4096 * size_t(channels_));
    Mix_HookMusic(&SoundManager::mixCallback, this);
    open_ = true;
    SDL_Log("audio opened: %d Hz, %d ch", freq, channels_);
}

void SoundManager::OnEvent(const SDL_Event& event) {
#ifdef __EMSCRIPTEN__
    bool gesture = event.type == SDL_KEYDOWN || event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_FINGERDOWN ||
                   event.type == SDL_CONTROLLERBUTTONDOWN;
    if (!gesture) {
        return;
    }
    if (!tried_) {
        open();
    }
    // ブラウザが音を止めた（AudioContext が suspended のまま、またはタブを切り替えた後）ときに、操作のたびに再開する。
    if (open_) {
        EM_ASM({
            var sdl = Module['SDL2'];
            if (sdl && sdl.audioContext && sdl.audioContext.state === 'suspended') {
                sdl.audioContext.resume();
            }
        });
    }
#else
    (void)event;
#endif
}

void SoundManager::mixCallback(void* userdata, Uint8* stream, int len) {
    // 音を出すスレッド（ブラウザではメインスレッド）から呼ばれる。
    SoundManager& self = *static_cast<SoundManager*>(userdata);
    const int sampleBytes = self.format_ == AUDIO_F32SYS ? 4 : 2;
    const int frames = len / (sampleBytes * self.channels_);
    const size_t samples = size_t(frames) * size_t(self.channels_);
    if (self.buffer_.size() < samples) {
        self.buffer_.resize(samples);
    }
    self.synth_.Render(self.buffer_.data(), frames, self.channels_);
    if (self.format_ == AUDIO_F32SYS) {
        SDL_memcpy(stream, self.buffer_.data(), samples * sizeof(float));
    } else {
        Sint16* out = reinterpret_cast<Sint16*>(stream);
        for (size_t i = 0; i < samples; ++i) {
            out[i] = static_cast<Sint16>(std::lround(std::clamp(self.buffer_[i], -1.0f, 1.0f) * 32767.0f));
        }
    }
}

void SoundManager::UnInit() {
    if (open_) {
        Mix_HookMusic(nullptr, nullptr);
        Mix_CloseAudio();
        open_ = false;
    }
    Mix_Quit();
}

}  // namespace System
