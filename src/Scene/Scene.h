#pragma once

#include "GameObject/GameObject.h"
#include "geom.hpp"

#include <memory>
#include <vector>

namespace System {
struct Context;
}  // namespace System

// GameObject を持ち、更新タイミングごとに order 順で呼ぶ。
class Scene {
public:
    explicit Scene(System::Context& ctx) : ctx_(ctx) {}
    virtual ~Scene() = default;

    // 派生クラスは Add で GameObject を足し、最後に InitObjects を呼ぶ。
    virtual bool Init() = 0;
    virtual void UnInit();

    void StartUpdate(float dt);
    void FixedUpdate(float dt);
    void PostPhysicsUpdate(float dt);
    void LateUpdate(float dt);
    void Draw();

    // true の間、App は固定ステップを回さない。
    virtual bool Paused() const { return false; }

    System::Context& Ctx() { return ctx_; }

    template <class T>
    T* Add() {
        auto object = std::make_unique<T>(*this);
        T* raw = object.get();
        objects_.push_back(std::move(object));
        return raw;
    }

    template <class T>
    T* Find() const {
        for (const auto& object : objects_) {
            if (T* found = dynamic_cast<T*>(object.get())) {
                return found;
            }
        }
        return nullptr;
    }

protected:
    // GameObject より先に、シーン自身が入力を処理する。
    virtual void OnStartUpdate(float dt) { (void)dt; }
    virtual void GetView(Mat4& viewProj, Vec3& eye) const = 0;
    // order 順に並べ、その順で Init を呼ぶ。1 つでも失敗したら false。
    bool InitObjects();

private:
    System::Context& ctx_;
    std::vector<std::unique_ptr<GameObject>> objects_;
};
