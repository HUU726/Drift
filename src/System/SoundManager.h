#pragma once

namespace System {

// 音の出力。音が開けなくてもゲームは止めず、IsOpen() が false になるだけ。
// 今は音のアセットがないので、開いて閉じるだけ。PlaySe / PlayBgm は音を足すときに作る。
class SoundManager {
public:
    bool Init();
    void UnInit();
    bool IsOpen() const { return open_; }

private:
    bool open_ = false;
};

}  // namespace System
