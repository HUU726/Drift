#pragma once

#include "geom.hpp"
#include "meshbuild.hpp"

#include <vector>

namespace System {

// 煙の粒 1 つ。カメラに向いた丸い板として、半透明で描く。
struct SmokePuff {
    Vec3 position;
    float size = 1;   // 直径（m）
    float alpha = 1;  // 濃さ（0..1）
};

struct GpuMesh {
    unsigned vao = 0;
    unsigned vbo = 0;
    unsigned ibo = 0;
    int indexCount = 0;
};

// 3D の描画（空、コース、車）。GL コンテキストは App が作り、Init より先に用意しておく。
class Renderer3D {
public:
    bool Init();
    void UnInit();
    void Resize(int width, int height);
    // 画面をクリアして空を描き、ワールド描画の準備をする。
    void BeginFrame(const Mat4& viewProj, Vec3 cameraPos);
    void Draw(const GpuMesh& mesh, const Mat4& model);
    // 煙を描く。不透明なものを描いた後に呼ぶ。camRight / camUp はカメラの右と上（板をカメラに向ける）。
    void DrawSmoke(const std::vector<SmokePuff>& puffs, Vec3 camRight, Vec3 camUp);
    GpuMesh Upload(const CpuMesh& mesh);
    void Release(GpuMesh& mesh);

    int Width() const { return width_; }
    int Height() const { return height_; }

private:
    void drawSky();

    int width_ = 1280;
    int height_ = 720;
    unsigned worldProgram_ = 0;
    unsigned skyProgram_ = 0;
    unsigned dummyVao_ = 0;
    int uViewProj_ = -1;
    int uModel_ = -1;
    int uCamPos_ = -1;
    // 煙
    unsigned smokeProgram_ = 0;
    unsigned smokeVao_ = 0;
    unsigned smokeVbo_ = 0;
    int uSmokeViewProj_ = -1;
    int uSmokeCamPos_ = -1;
    Mat4 viewProj_;
    Vec3 cameraPos_;
    std::vector<float> smokeVertices_;
};

}  // namespace System
