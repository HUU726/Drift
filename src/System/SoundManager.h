#pragma once

#include "System/SoundSynth.h"

#include <SDL.h>

#include <vector>

namespace System {

// 音の出力。音はすべて SoundSynth で実行時に合成し、SDL_mixer の音楽の差し込み口（Mix_HookMusic）から流す。
// 音が開けなくてもゲームは止めず、IsOpen() が false になるだけ（BGM や音量の指定は受け付けて覚えておく）。
// ブラウザでは、自動再生の制限があるので、最初のキー入力・クリック・タッチ（OnEvent）で音を開く。
class SoundManager {
public:
    bool Init();
    void UnInit();
    // App がイベントごとに呼ぶ。ブラウザでは、最初の操作で音を開き、止まっていれば再開する。
    void OnEvent(const SDL_Event& event);
    bool IsOpen() const { return open_; }

    // BGM を切り替える（同じ曲なら何もしない）。前の曲は少しずつ下げてから切り替わる。
    void PlayBgm(BgmTrack track) { synth_.SetBgm(track); }
    // strength（0..1）は当たりの強さなど、効果音の大きさ。
    void PlaySe(SoundEffect effect, float strength = 1.0f) { synth_.Play(effect, strength); }
    // 車の音（エンジン音とスキール音）。車のいるシーンが毎フレーム渡し、いなくなるときに StopCar を呼ぶ。
    void SetCar(const CarSoundParams& params) { synth_.SetCar(params); }
    void StopCar() { synth_.SetCar(CarSoundParams{}); }

    // 音量（0..1）。調整の画面から変える。
    float MasterVolume() const { return synth_.masterVolume.load(); }
    float MusicVolume() const { return synth_.musicVolume.load(); }
    float EffectsVolume() const { return synth_.effectsVolume.load(); }
    void SetMasterVolume(float v) { synth_.masterVolume.store(v); }
    void SetMusicVolume(float v) { synth_.musicVolume.store(v); }
    void SetEffectsVolume(float v) { synth_.effectsVolume.store(v); }

private:
    void open();
    static void mixCallback(void* userdata, Uint8* stream, int len);

    SoundSynth synth_;
    std::vector<float> buffer_;  // 合成した音（float）を出力の形式に直す前に置く
    SDL_AudioFormat format_ = 0;
    int channels_ = 2;
    bool open_ = false;
    bool tried_ = false;  // 開こうとしたか（失敗しても何度も試さない）
};

}  // namespace System
