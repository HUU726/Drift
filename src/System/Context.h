#pragma once

// マネージャへの参照をまとめたもの。App が作り、SceneManager を通して各 Scene に渡す。
// ColliderManager は物理の段階 4 で足す。
namespace System {

class Renderer3D;
class Renderer2D;
class SoundManager;
class SceneManager;
class Input;

struct Context {
    Renderer3D& renderer3D;
    Renderer2D& renderer2D;
    SoundManager& sound;
    SceneManager& scenes;
    Input& input;
};

}  // namespace System
