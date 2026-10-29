#include "GameObject/TrackObject.h"

#include "Scene/Scene.h"
#include "System/ColliderManager.h"
#include "System/Context.h"
#include "TougeCourse.h"

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
    if (which_ == Course::Touge || which_ == Course::Lab) {
        bool built = which_ == Course::Touge ? BuildTougeCourse(track_) : BuildLabCourse(track_);
        if (!built) {
            std::fprintf(stderr, "コースを作れません\n");
            return false;
        }
    } else {
        std::string path = FindPath();
        if (!track_.load(path)) {
            std::fprintf(stderr, "コースを読めません: %s\n", path.c_str());
            return false;
        }
    }
    course_ = scene_.Ctx().renderer3D.Upload(track_.visual);
    collider_.owner = this;
    scene_.Ctx().physics.Register(&collider_);
    return true;
}

void TrackObject::Draw3D(System::Renderer3D& renderer) {
    Mat4 identity;
    renderer.Draw(course_, identity);
}

void TrackObject::UnInit() {
    scene_.Ctx().physics.Unregister(&collider_);
    scene_.Ctx().renderer3D.Release(course_);
}
