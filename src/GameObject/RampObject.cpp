#include "GameObject/RampObject.h"

#include "GameObject/TrackObject.h"
#include "Scene/Scene.h"
#include "System/ColliderManager.h"
#include "System/Context.h"

#include <fstream>

std::string RampObject::FindPath() {
    const char* paths[] = {
        "assets/track/ramps.txt",
        "../assets/track/ramps.txt",
        "/assets/track/ramps.txt",
    };
    for (const char* path : paths) {
        std::ifstream in(path);
        if (in) {
            return path;
        }
    }
    return paths[0];
}

bool RampObject::Init() {
    TrackObject* track = scene_.Find<TrackObject>();
    if (!track) {
        return false;
    }
    CpuMesh mesh;
    for (const RampSpec& spec : RampCollider::Load(FindPath())) {
        auto ramp = std::make_unique<RampCollider>(track->GetTrack(), spec);
        ramp->owner = this;
        ramp->BuildMesh(mesh);
        scene_.Ctx().physics.Register(ramp.get());
        ramps_.push_back(std::move(ramp));
    }
    mesh_ = scene_.Ctx().renderer3D.Upload(mesh);
    return true;
}

void RampObject::Draw3D(System::Renderer3D& renderer) {
    Mat4 identity;
    renderer.Draw(mesh_, identity);
}

void RampObject::UnInit() {
    for (auto& ramp : ramps_) {
        scene_.Ctx().physics.Unregister(ramp.get());
    }
    ramps_.clear();
    scene_.Ctx().renderer3D.Release(mesh_);
}
