#pragma once

#include "GameObject/GameObject.h"
#include "System/Renderer3D.h"
#include "TestCourse.h"

// 試験コースの地面とジャンプ台。TestCourse を作って ColliderManager に登録し、描く。
class TestGroundObject : public GameObject {
public:
    explicit TestGroundObject(Scene& scene) : GameObject(scene) { order = 0; }
    bool Init() override;
    void Draw3D(System::Renderer3D& renderer) override;
    void UnInit() override;

    const TestCourse& Course() const { return course_; }

private:
    TestCourse course_;
    System::GpuMesh mesh_;
};
