#include "Scene/Scene.h"

#include "System/Context.h"
#include "System/Renderer2D.h"
#include "System/Renderer3D.h"

#include <algorithm>

bool Scene::InitObjects() {
    std::stable_sort(objects_.begin(), objects_.end(),
                     [](const auto& a, const auto& b) { return a->order < b->order; });
    for (auto& object : objects_) {
        if (!object->Init()) {
            return false;
        }
    }
    return true;
}

void Scene::UnInit() {
    for (auto it = objects_.rbegin(); it != objects_.rend(); ++it) {
        (*it)->UnInit();
    }
    objects_.clear();
}

void Scene::StartUpdate(float dt) {
    OnStartUpdate(dt);
    for (auto& object : objects_) {
        if (object->active) object->StartUpdate(dt);
    }
}

void Scene::FixedUpdate(float dt) {
    for (auto& object : objects_) {
        if (object->active) object->FixedUpdate(dt);
    }
}

void Scene::PostPhysicsUpdate(float dt) {
    for (auto& object : objects_) {
        if (object->active) object->PostPhysicsUpdate(dt);
    }
}

void Scene::LateUpdate(float dt) {
    for (auto& object : objects_) {
        if (object->active) object->LateUpdate(dt);
    }
}

void Scene::Draw() {
    Mat4 viewProj;
    Vec3 eye;
    GetView(viewProj, eye);
    ctx_.renderer3D.BeginFrame(viewProj, eye);
    for (auto& object : objects_) {
        if (object->active) object->Draw3D(ctx_.renderer3D);
    }
    ctx_.renderer2D.Begin();
    for (auto& object : objects_) {
        if (object->active) object->Draw2D(ctx_.renderer2D);
    }
    ctx_.renderer2D.Flush();
}
