#pragma once

// マネージャへの参照をまとめたもの。App が作り、SceneManager を通して各 Scene に渡す。
// SoundManager と ColliderManager は、それぞれを作る手順で足す。
namespace System {

class Renderer3D;
class Renderer2D;
class SceneManager;
class Input;

struct Context {
    Renderer3D& renderer3D;
    Renderer2D& renderer2D;
    SceneManager& scenes;
    Input& input;
};

}  // namespace System
