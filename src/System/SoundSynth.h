#pragma once

#include <atomic>
#include <cstdint>

namespace System {

// 車の音の入力。車のいるシーンが毎フレーム渡す。
struct CarSoundParams {
    bool active = false;  // 車の音を出すか（シーン選択の画面やポーズ中は false）
    float rpm = 0;        // エンジンの回転数
    float throttle = 0;   // アクセル（0..1）
    float speed = 0;      // 車の速さ（m/s）
    float skid = 0;       // スキール音の強さ（0..1）
};

enum class BgmTrack { None, Title, Race };
// 効果音。Crash はガードレールや壁に当たったとき、Rollover は車体が横転したとき、Tick はメニューの操作。
enum class SoundEffect { Lap, BestLap, Crash, Rollover, Tick, Count };

// 音の合成。エンジン音、スキール音、BGM、効果音をすべて実行時に作る（音のファイルは使わない）。
// Render は音を出すスレッドから呼ばれ、Set / Play はゲームのスレッドから呼ばれるので、受け渡しは atomic で行う。
// SDL を使わないので、音の出ない環境（self-test）でも Render を呼んで確かめられる。
class SoundSynth {
public:
    SoundSynth();
    void SetSampleRate(int rate);
    // frames 個分の音を out に上書きで作る。channels 個ずつ交互に並べ、値は -1..1。
    void Render(float* out, int frames, int channels);

    void SetCar(const CarSoundParams& params);
    void SetBgm(BgmTrack track) { requestedBgm_.store(static_cast<int>(track)); }
    // strength（0..1）は当たりの強さなど、効果音の大きさ。同じフレームに何度も呼んだときは最後の値を使う。
    void Play(SoundEffect effect, float strength = 1.0f);

    // 音量（0..1）。master は全体、music は BGM、effects はエンジン音、スキール音、効果音に掛ける。
    // 既定の全体の音量は 0.8 の 3 分の 2。
    std::atomic<float> masterVolume{0.8f * 2.0f / 3.0f};
    std::atomic<float> musicVolume{0.1f};
    std::atomic<float> effectsVolume{0.2f};

private:
    // 1 音ずつ鳴らす声部（BGM のパート）。鳴らすたびに音の高さと包絡線を決め直す。
    struct Voice {
        float freq = 0;
        float phase = 0;
        float env = 0;       // 今の大きさ
        float attack = 0;    // 立ち上がりの残り（0 なら減衰中）
        float decay = 0;     // 1 サンプルごとに env に掛ける値
        float sweep = 1;     // 1 サンプルごとに freq に掛ける値（バスドラムの音の下がり）
    };
    // 効果音の 1 音。delay サンプル待ってから鳴る。
    struct Chime {
        float freq = 0;
        float phase = 0;
        float time = 0;
        int delay = 0;
        float decay = 5;     // 小さくなる速さ（1/秒）
        float gain = 0.22f;
        bool on = false;
    };
    // ぶつかる音の 1 回。低い「ドン」、雑音の「ガシャ」、金属の響き（ガードレール）を重ねる。delay サンプル待ってから鳴る。
    struct Hit {
        float time = 0;
        int delay = 0;
        float gain = 0;
        float thump = 0;        // 「ドン」の最初の高さ（Hz）。時間とともに下がる
        float thumpPhase = 0;
        float metal = 0;        // 金属の響きの大きさ
        float metalPhase[3] = {};
        float noiseDecay = 25;  // 雑音が小さくなる速さ（1/秒）。小さいほど長く「ガシャー」と残る
        float noiseLow = 0;     // 雑音のローパスの状態
        bool on = false;
    };
    float noise();
    float renderEngine();
    float renderSkid();
    float renderChimes();
    float renderHits();
    float renderMusic();
    void stepSequencer();
    void startEffect(SoundEffect effect, float strength);
    void startChime(float freq, float delaySec, float decay = 5, float gain = 0.22f);
    void startHit(const Hit& hit, float delaySec);
    void trigger(Voice& voice, float freq, float attackSec, float decaySec, float sweep = 1);

    int sampleRate_ = 44100;
    float dt_ = 1.0f / 44100.0f;
    uint32_t noiseState_ = 0x12345678u;

    // ゲームのスレッドから受け取る値
    std::atomic<bool> carActive_{false};
    std::atomic<float> carRpm_{0};
    std::atomic<float> carThrottle_{0};
    std::atomic<float> carSpeed_{0};
    std::atomic<float> carSkid_{0};
    std::atomic<int> requestedBgm_{0};
    // 効果音ごとの、鳴らすように頼まれた回数と強さ。音のスレッドは seen_ と比べて、増えた分を鳴らす。
    static constexpr int kEffectCount = static_cast<int>(SoundEffect::Count);
    std::atomic<int> requests_[kEffectCount] = {};
    std::atomic<float> strengths_[kEffectCount] = {};

    // 以下は音のスレッドだけが触る
    // エンジン
    float rpm_ = 0, throttle_ = 0, carGain_ = 0;
    float enginePhase_ = 0, engineSubPhase_ = 0;
    float engineLow1_ = 0, engineLow2_ = 0;
    // スキール
    float skid_ = 0, squealPhase_ = 0, wobblePhase_ = 0;
    float skidLow_ = 0, skidBand_ = 0;
    // 効果音
    Chime chimes_[8];
    Hit hits_[8];
    int seen_[kEffectCount] = {};
    // BGM
    int bgm_ = 0;
    float bgmFade_ = 0;  // 曲の切り替えで下げる大きさ（0..1）
    int step_ = 0;
    float stepSamples_ = 0;  // 次のステップまでのサンプル数
    Voice lead_, bass_, arp_, pad_[3], kick_, snare_, hat_;
    float leadVibrato_ = 0;
    float padLow_ = 0, hatHigh_ = 0, hatPrev_ = 0;
};

}  // namespace System
