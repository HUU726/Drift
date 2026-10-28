#pragma once

#include "Scene/Scene.h"

#include <memory>

namespace System {

struct Context;

// シーンの保持と遷移。各更新タイミングを今のシーンに中継する。
// 遷移は Change で予約し、フレームの最後に App が ApplyPending で適用する。
class SceneManager {
public:
    template <class First>
    bool Init(Context& ctx) {
        ctx_ = &ctx;
        Change<First>();
        return ApplyPending();
    }
    void UnInit();

    void StartUpdate(float dt);
    void FixedUpdate(float dt);
    void PostPhysicsUpdate(float dt);
    void LateUpdate(float dt);
    void Draw();
    bool Paused() const;

    template <class T>
    void Change() { pending_ = std::make_unique<T>(*ctx_); }
    // 予約があれば今のシーンを UnInit し、新しいシーンを Init する。Init に失敗したら false。
    bool ApplyPending();

private:
    Context* ctx_ = nullptr;
    std::unique_ptr<Scene> current_;
    std::unique_ptr<Scene> pending_;
};

}  // namespace System
