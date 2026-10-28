#pragma once

#include "GameObject/GameObject.h"
#include "Physics/TrackCollider.h"
#include "System/Renderer3D.h"
#include "track.hpp"

#include <string>

// コース。Track を読み込み、コースのメッシュを描き、壁の当たり判定を登録する。
class TrackObject : public GameObject {
public:
    explicit TrackObject(Scene& scene) : GameObject(scene) { order = 0; }
    bool Init() override;
    void Draw3D(System::Renderer3D& renderer) override;
    void UnInit() override;

    const Track& GetTrack() const { return track_; }

    // 実行場所によってパスが変わるので、候補を順に試す。
    static std::string FindPath();

private:
    Track track_;
    TrackCollider collider_{track_};
    System::GpuMesh course_;
};
