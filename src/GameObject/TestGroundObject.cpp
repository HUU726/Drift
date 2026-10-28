#include "GameObject/TestGroundObject.h"

#include "Scene/Scene.h"
#include "System/ColliderManager.h"
#include "System/Context.h"

bool TestGroundObject::Init() {
    course_.Build();
    System::ColliderManager& physics = scene_.Ctx().physics;
    course_.ground.owner = this;
    physics.Register(&course_.ground);
    for (auto& ramp : course_.ramps) {
        ramp->owner = this;
        physics.Register(ramp.get());
    }
    CpuMesh mesh;
    course_.BuildMesh(mesh);
    mesh_ = scene_.Ctx().renderer3D.Upload(mesh);
    return true;
}

void TestGroundObject::Draw3D(System::Renderer3D& renderer) {
    Mat4 identity;
    renderer.Draw(mesh_, identity);
}

void TestGroundObject::UnInit() {
    System::ColliderManager& physics = scene_.Ctx().physics;
    physics.Unregister(&course_.ground);
    for (auto& ramp : course_.ramps) {
        physics.Unregister(ramp.get());
    }
    scene_.Ctx().renderer3D.Release(mesh_);
}
