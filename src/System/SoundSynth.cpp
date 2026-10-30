#include "System/SoundSynth.h"

#include <algorithm>
#include <cmath>

namespace System {

namespace {

constexpr float kTwoPi = 6.2831853f;
constexpr float kCrossfadeSec = 0.5f;  // 曲を切り替えるとき、前の曲を下げきるまでの時間

// MIDI の音の番号から周波数（Hz）。69 が A4（440 Hz）。
float noteFreq(int note) {
    return 440.0f * std::pow(2.0f, float(note - 69) / 12.0f);
}

// 時定数 tau（秒）で目標へ寄せるとき、1 サンプルで寄せる割合。
float smoothing(float tau, float dt) {
    return 1.0f - std::exp(-dt / tau);
}

// カットオフ fc（Hz）の 1 次のローパスで、1 サンプルで寄せる割合。
float lowpassCoef(float fc, float dt) {
    return 1.0f - std::exp(-kTwoPi * fc * dt);
}

float wrap(float phase) {
    return phase - std::floor(phase);
}

// のこぎり波。音の高さが変わる角で出る折り返し（エイリアス）を polyBLEP で抑える。
float saw(float phase, float inc) {
    float v = 2.0f * phase - 1.0f;
    if (phase < inc) {
        float t = phase / inc;
        v -= t + t - t * t - 1.0f;
    } else if (phase > 1.0f - inc) {
        float t = (phase - 1.0f) / inc;
        v -= t * t + t + t + 1.0f;
    }
    return v;
}

float square(float phase, float duty) {
    return phase < duty ? 1.0f : -1.0f;
}

float triangle(float phase) {
    return 4.0f * std::fabs(phase - 0.5f) - 1.0f;
}

// 大きい音だけをなめらかに頭打ちにする（小さい音はほぼそのまま）。
float softClip(float x) {
    x = std::clamp(x, -3.0f, 3.0f);
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// BGM の 1 曲。4 小節（16 分音符で 64 ステップ）を繰り返す。曲はどちらもこのリポジトリのための自作。
struct Track {
    float bpm;
    int chords[4][3];    // 小節ごとの和音（MIDI の音の番号）
    int bassRoot[4];     // 小節ごとのベースの根音
    const char* bass;    // 1 小節 16 ステップ。'R' 根音、'O' 1 オクターブ上、'.' 鳴らさない
    const char* arp;     // 1 小節 16 ステップ。'0'..'2' 和音の何番目を 1 オクターブ上で鳴らすか
    const char* kick;    // 1 小節 16 ステップ。'x' で鳴らす
    const char* snare;
    const char* hat;
    int lead[64];        // 主旋律。0 は鳴らさない（前の音が減衰していく）
    bool pad;            // 和音を伸ばして鳴らすか
    float leadDecay;     // 主旋律の 1 音が小さくなる時定数（秒）
};

// タイトル（シーン選択）の曲。ゆったりした Am - F - C - G。
const Track kTitle = {
    92.0f,
    {{57, 60, 64}, {57, 60, 65}, {55, 60, 64}, {55, 59, 62}},
    {45, 41, 48, 43},
    "R.......R...O...",
    "0.1.2.1.0.1.2.1.",
    "................",
    "................",
    "....x.......x...",
    {76, 0, 0, 0, 0, 0, 74, 0, 72, 0, 0, 0, 69, 0, 0, 0,
     72, 0, 0, 0, 0, 0, 0, 0, 69, 0, 72, 0, 77, 0, 0, 0,
     76, 0, 0, 0, 0, 0, 72, 0, 74, 0, 0, 0, 76, 0, 79, 0,
     74, 0, 0, 0, 0, 0, 0, 0, 71, 0, 0, 0, 0, 0, 0, 0},
    true,
    0.6f,
};

// プレイ中の曲。速い Em - C - D - B。
const Track kRace = {
    148.0f,
    {{64, 67, 71}, {64, 67, 72}, {62, 66, 69}, {63, 66, 71}},
    {40, 36, 38, 35},
    "R.R.O.R.R.R.O.R.",
    "0120012001200120",
    "x...x...x...x...",
    "....x.......x...",
    "..x...x...x...xx",
    {76, 0, 79, 0, 83, 0, 0, 0, 81, 0, 79, 0, 76, 0, 0, 0,
     76, 0, 79, 0, 84, 0, 0, 0, 83, 0, 79, 0, 76, 0, 72, 0,
     74, 0, 78, 0, 81, 0, 0, 0, 86, 0, 0, 0, 81, 0, 78, 0,
     75, 0, 0, 0, 78, 0, 0, 0, 83, 0, 0, 0, 0, 0, 0, 0},
    false,
    0.25f,
};

const Track* trackOf(int bgm) {
    switch (static_cast<BgmTrack>(bgm)) {
        case BgmTrack::Title:
            return &kTitle;
        case BgmTrack::Race:
            return &kRace;
        default:
            return nullptr;
    }
}

}  // namespace

SoundSynth::SoundSynth() {
    SetSampleRate(44100);
}

void SoundSynth::SetSampleRate(int rate) {
    sampleRate_ = std::max(8000, rate);
    dt_ = 1.0f / float(sampleRate_);
}

void SoundSynth::SetCar(const CarSoundParams& params) {
    carActive_.store(params.active);
    carRpm_.store(params.rpm);
    carThrottle_.store(std::clamp(params.throttle, 0.0f, 1.0f));
    carSpeed_.store(params.speed);
    carSkid_.store(std::clamp(params.skid, 0.0f, 1.0f));
}

void SoundSynth::Play(SoundEffect effect) {
    if (effect == SoundEffect::BestLap) {
        bestRequests_.fetch_add(1);
    } else {
        lapRequests_.fetch_add(1);
    }
}

float SoundSynth::noise() {
    // xorshift32。-1..1 の白色雑音
    noiseState_ ^= noiseState_ << 13;
    noiseState_ ^= noiseState_ >> 17;
    noiseState_ ^= noiseState_ << 5;
    return float(noiseState_ >> 8) / float(1u << 23) - 1.0f;
}

void SoundSynth::Render(float* out, int frames, int channels) {
    // ブロックの頭で、ゲームのスレッドからの値と音量を読む。
    const float master = std::clamp(masterVolume.load(), 0.0f, 1.0f);
    const float music = std::clamp(musicVolume.load(), 0.0f, 1.0f);
    const float effects = std::clamp(effectsVolume.load(), 0.0f, 1.0f);
    int lap = lapRequests_.load();
    int best = bestRequests_.load();
    // 周回の音は 2 音、ベストラップの音は 3 音を、少しずつずらして鳴らす。
    for (; lapSeen_ < lap; ++lapSeen_) {
        startChime(noteFreq(83), 0.0f);   // B5
        startChime(noteFreq(88), 0.12f);  // E6
    }
    for (; bestSeen_ < best; ++bestSeen_) {
        startChime(noteFreq(84), 0.0f);   // C6
        startChime(noteFreq(88), 0.1f);   // E6
        startChime(noteFreq(91), 0.2f);   // G6
        startChime(noteFreq(96), 0.3f);   // C7
    }

    for (int i = 0; i < frames; ++i) {
        float fx = renderEngine() + renderSkid() + renderChimes();
        float bgm = renderMusic();
        float v = softClip((fx * effects + bgm * music) * master);
        for (int c = 0; c < channels; ++c) {
            out[i * channels + c] = v;
        }
    }
}

float SoundSynth::renderEngine() {
    // 値をなめらかに寄せる（ゲームからはフレームごとにしか来ないので、そのままだと段がついて聞こえる）。
    const bool active = carActive_.load(std::memory_order_relaxed);
    rpm_ += (std::max(carRpm_.load(std::memory_order_relaxed), 700.0f) - rpm_) * smoothing(0.06f, dt_);
    throttle_ += (carThrottle_.load(std::memory_order_relaxed) - throttle_) * smoothing(0.08f, dt_);
    carGain_ += ((active ? 1.0f : 0.0f) - carGain_) * smoothing(0.25f, dt_);
    if (carGain_ < 1e-4f) {
        return 0;
    }

    // 4 気筒の 4 ストロークなので、1 回転に 2 回爆発する。その周波数を基音にする。
    const float fire = rpm_ / 60.0f * 2.0f;
    const float inc = std::min(fire * dt_, 0.5f);
    enginePhase_ = wrap(enginePhase_ + inc);
    engineSubPhase_ = wrap(engineSubPhase_ + inc * 0.5f);
    // のこぎり波に、1 オクターブ下（クランクの回転）と爆発ごとの雑音を重ねる。
    float pulse = std::exp(-enginePhase_ * 6.0f);
    float tone = 0.55f * saw(enginePhase_, inc) + 0.35f * std::sin(kTwoPi * engineSubPhase_) +
                 0.3f * std::sin(kTwoPi * enginePhase_) + 0.25f * noise() * pulse * (0.3f + throttle_);
    // アクセルを踏むほど、回るほど明るい音にする。2 段のローパスで角を丸める。
    const float rpmNorm = std::clamp((rpm_ - 700.0f) / 7000.0f, 0.0f, 1.0f);
    const float cutoff = 220.0f + 1800.0f * throttle_ + 900.0f * rpmNorm;
    const float a = lowpassCoef(cutoff, dt_);
    engineLow1_ += (tone - engineLow1_) * a;
    engineLow2_ += (engineLow1_ - engineLow2_) * a;
    const float gain = carGain_ * (0.16f + 0.2f * throttle_ + 0.12f * rpmNorm);
    return engineLow2_ * gain;
}

float SoundSynth::renderSkid() {
    float target = carActive_.load(std::memory_order_relaxed) ? carSkid_.load(std::memory_order_relaxed) : 0.0f;
    skid_ += (target - skid_) * smoothing(0.07f, dt_);
    if (skid_ < 1e-4f) {
        return 0;
    }
    // タイヤの鳴き。揺れる高さの音と、帯域を絞った雑音を重ねる。速いほど少し高くする。
    const float speed = carSpeed_.load(std::memory_order_relaxed);
    wobblePhase_ = wrap(wobblePhase_ + 6.5f * dt_);
    const float wobble = std::sin(kTwoPi * wobblePhase_);
    const float pitch = 760.0f + 4.0f * std::min(speed, 40.0f) + 25.0f * wobble;
    squealPhase_ = wrap(squealPhase_ + pitch * dt_);
    float squeal = std::sin(kTwoPi * squealPhase_) + 0.3f * std::sin(2.0f * kTwoPi * squealPhase_);

    // 状態変数フィルタのバンドパス（中心 1.5 kHz 前後）
    const float f = 2.0f * std::sin(3.1415927f * (1500.0f + 200.0f * wobble) * dt_);
    const float q = 0.35f;
    float high = noise() - skidLow_ - q * skidBand_;
    skidBand_ += f * high;
    skidLow_ += f * skidBand_;

    const float amount = skid_ * skid_;
    return (0.22f * squeal + 0.3f * skidBand_) * amount * 0.6f;
}

void SoundSynth::startChime(float freq, float delaySec) {
    // 空いている枠を使う。空きがなければ、いちばん古い音を止めて使う。
    Chime* slot = &chimes_[0];
    for (Chime& c : chimes_) {
        if (!c.on) {
            slot = &c;
            break;
        }
        if (c.time > slot->time) {
            slot = &c;
        }
    }
    *slot = Chime{freq, 0, 0, int(delaySec * float(sampleRate_)), true};
}

float SoundSynth::renderChimes() {
    float sum = 0;
    for (Chime& c : chimes_) {
        if (!c.on) {
            continue;
        }
        if (c.delay > 0) {
            --c.delay;
            continue;
        }
        // ベルの音。基音に 2 倍と 3 倍の音を少し重ね、短い立ち上がりのあと指数で減衰する。
        c.phase = wrap(c.phase + c.freq * dt_);
        float env = std::min(1.0f, c.time / 0.004f) * std::exp(-c.time * 5.0f);
        float v = std::sin(kTwoPi * c.phase) + 0.3f * std::sin(2.0f * kTwoPi * c.phase) +
                  0.12f * std::sin(3.0f * kTwoPi * c.phase);
        sum += v * env * 0.22f;
        c.time += dt_;
        if (c.time > 1.5f) {
            c.on = false;
        }
    }
    return sum;
}

void SoundSynth::trigger(Voice& voice, float freq, float attackSec, float decaySec, float sweep) {
    voice.freq = freq;
    voice.attack = std::max(1.0f, attackSec * float(sampleRate_));
    voice.decay = std::exp(-dt_ / decaySec);
    voice.sweep = sweep;
}

// 1 ステップ（16 分音符）ごとに、各パートの音を鳴らす。
void SoundSynth::stepSequencer() {
    const Track* track = trackOf(bgm_);
    if (!track) {
        return;
    }
    const int bar = (step_ / 16) % 4;
    const int s = step_ % 16;
    const int* chord = track->chords[bar];

    if (track->bass[s] == 'R' || track->bass[s] == 'O') {
        int note = track->bassRoot[bar] + (track->bass[s] == 'O' ? 12 : 0);
        trigger(bass_, noteFreq(note), 0.004f, track->pad ? 0.5f : 0.18f);
    }
    if (track->arp[s] >= '0' && track->arp[s] <= '2') {
        trigger(arp_, noteFreq(chord[track->arp[s] - '0'] + 12), 0.003f, 0.09f);
    }
    if (track->kick[s] == 'x') {
        trigger(kick_, 150.0f, 0.002f, 0.18f, std::pow(45.0f / 150.0f, dt_ / 0.08f));
    }
    if (track->snare[s] == 'x') {
        trigger(snare_, 190.0f, 0.002f, 0.1f);
    }
    if (track->hat[s] == 'x') {
        trigger(hat_, 0, 0.001f, track->pad ? 0.08f : 0.035f);
    }
    if (track->lead[step_ % 64] != 0) {
        trigger(lead_, noteFreq(track->lead[step_ % 64]), 0.01f, track->leadDecay);
    }
    if (track->pad && s == 0) {
        for (int i = 0; i < 3; ++i) {
            trigger(pad_[i], noteFreq(chord[i]), 0.5f, 2.5f);
        }
    }
    step_ = (step_ + 1) % 64;
}

float SoundSynth::renderMusic() {
    // 曲の切り替え。前の曲を下げきってから、新しい曲を頭から鳴らす。
    const int requested = requestedBgm_.load(std::memory_order_relaxed);
    const float fadeStep = dt_ / kCrossfadeSec;
    if (requested != bgm_) {
        bgmFade_ -= fadeStep;
        if (bgmFade_ <= 0) {
            bgmFade_ = 0;
            bgm_ = requested;
            step_ = 0;
            stepSamples_ = 0;
            for (Voice* v : {&lead_, &bass_, &arp_, &pad_[0], &pad_[1], &pad_[2], &kick_, &snare_, &hat_}) {
                *v = Voice{};
            }
        }
    } else {
        bgmFade_ = std::min(1.0f, bgmFade_ + fadeStep);
    }
    const Track* track = trackOf(bgm_);
    if (!track) {
        return 0;
    }

    stepSamples_ -= 1.0f;
    if (stepSamples_ <= 0) {
        stepSamples_ += float(sampleRate_) * 60.0f / track->bpm / 4.0f;
        stepSequencer();
    }

    // 包絡線を進める。立ち上がりの間は 1 へ近づけ、その後は指数で下げる。
    auto envelope = [](Voice& v) {
        if (v.attack > 0) {
            v.env += (1.0f - v.env) / v.attack;
            v.attack -= 1.0f;
            if (v.attack <= 0) v.attack = 0;
        } else {
            v.env *= v.decay;
        }
        return v.env;
    };
    auto advance = [this](Voice& v) {
        v.phase = wrap(v.phase + v.freq * dt_);
        return v.phase;
    };

    float sum = 0;
    // 主旋律: 少し揺らした矩形波
    leadVibrato_ = wrap(leadVibrato_ + 5.0f * dt_);
    lead_.phase = wrap(lead_.phase + lead_.freq * (1.0f + 0.004f * std::sin(kTwoPi * leadVibrato_)) * dt_);
    sum += 0.09f * square(lead_.phase, 0.5f) * envelope(lead_);
    // ベース: 三角波と矩形波を混ぜる
    float bp = advance(bass_);
    sum += (0.22f * triangle(bp) + 0.06f * square(bp, 0.5f)) * envelope(bass_);
    // アルペジオ: 細い矩形波を小さく
    sum += 0.04f * square(advance(arp_), 0.25f) * envelope(arp_);
    // パッド: 三角波の和音を、ローパスでやわらかく
    if (track->pad) {
        float pad = 0;
        for (Voice& v : pad_) {
            pad += triangle(advance(v)) * envelope(v);
        }
        padLow_ += (pad - padLow_) * lowpassCoef(900.0f, dt_);
        sum += 0.06f * padLow_;
    }
    // バスドラム: 音の高さを下げていく正弦波
    kick_.freq *= kick_.sweep;
    sum += 0.4f * std::sin(kTwoPi * advance(kick_)) * envelope(kick_);
    // スネア: 雑音と低い音
    float sn = envelope(snare_);
    sum += (0.14f * noise() + 0.08f * std::sin(kTwoPi * advance(snare_))) * sn;
    // ハイハット: 雑音の高いところだけ（1 次のハイパス）
    float n = noise();
    hatHigh_ = 0.6f * (hatHigh_ + n - hatPrev_);
    hatPrev_ = n;
    sum += 0.07f * hatHigh_ * envelope(hat_);

    return sum * bgmFade_;
}

}  // namespace System
