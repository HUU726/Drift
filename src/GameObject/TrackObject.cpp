#include "GameObject/TrackObject.h"

#include "Scene/Scene.h"
#include "System/Context.h"

#include <cstdio>
#include <fstream>

std::string TrackObject::FindPath() {
    const char* paths[] = {
        "assets/track/suzuka.txt",
        "../assets/track/suzuka.txt",
        "/assets/track/suzuka.txt",
    };
    for (const char* path : paths) {
        std::ifstream in(path);
        if (in) {
            return path;
        }
    }
    return paths[0];
}

bool TrackObject::Init() {
    std::string path = FindPath();
    if (!track_.load(path)) {
        std::fprintf(stderr, "コースを読めません: %s\n", path.c_str());
        return false;
    }
    course_ = scene_.Ctx().renderer3D.Upload(track_.visual);
    return true;
}

void TrackObject::Draw3D(System::Renderer3D& renderer) {
    Mat4 identity;
    renderer.Draw(course_, identity);
}

void TrackObject::UnInit() {
    scene_.Ctx().renderer3D.Release(course_);
}
