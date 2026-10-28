#pragma once

#include <memory>

// アプリケーション全体。main からは Init / Run / UnInit だけを呼ぶ。
// 移行手順 3 の段階では、ゲームの中身も State に仮置きしている（手順 4 で Scene と GameObject に移す）。
class App {
public:
    App();
    ~App();

    bool Init(int argc, char** argv);
    // --self-test のときはテスト結果を返す。ブラウザ版では戻らない。
    int Run();
    void UnInit();

    struct State;

private:
    std::unique_ptr<State> state_;
    bool selfTest_ = false;
    bool imageOk_ = false;
    bool audioOk_ = false;
};
