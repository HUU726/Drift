#pragma once

#include "GameObject/GameObject.h"
#include "Physics/RampCollider.h"
#include "System/Renderer3D.h"

#include <memory>
#include <string>
#include <vector>

// ジャンプ台。ramps.txt の配置から RampCollider を作って登録し、台を描く。
// ramps.txt がなければ台を置かずに続ける。
class RampObject : public GameObject {
public:
    explicit RampObject(Scene& scene) : GameObject(scene) { order = 5; }
    bool Init() override;
    void Draw3D(System::Renderer3D& renderer) override;
    void UnInit() override;

    // 実行場所によってパスが変わるので、候補を順に試す。
    static std::string FindPath();

private:
    std::vector<std::unique_ptr<RampCollider>> ramps_;
    System::GpuMesh mesh_;
};
