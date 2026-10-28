#pragma once

class Scene;

namespace System {
class Renderer3D;
class Renderer2D;
}  // namespace System

// 更新と描画を持つ最小単位。Scene が持ち、order の小さい順に呼ぶ。
// OnCollision は ColliderManager を入れる段階で足す。
class GameObject {
public:
    explicit GameObject(Scene& scene) : scene_(scene) {}
    virtual ~GameObject() = default;
    virtual bool Init() { return true; }
    virtual void StartUpdate(float dt) { (void)dt; }        // 毎フレーム 1 回。入力と切り替え
    virtual void FixedUpdate(float dt) { (void)dt; }        // 固定ステップ（1/60 秒）。物理への力と操作量
    virtual void PostPhysicsUpdate(float dt) { (void)dt; }  // 固定ステップ。物理の結果を読む
    virtual void LateUpdate(float dt) { (void)dt; }         // 毎フレーム 1 回。カメラの追従と補間
    virtual void Draw3D(System::Renderer3D&) {}
    virtual void Draw2D(System::Renderer2D&) {}
    virtual void UnInit() {}
    bool active = true;
    int order = 0;  // 更新と描画の順（小さいほうが先）

protected:
    Scene& scene_;
};
