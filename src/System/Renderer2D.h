#pragma once

#include <vector>

namespace System {

// 2D の描画（HUD、ミニマップ、文字）。Begin から Flush までに積んだ四角形をまとめて描く。
class Renderer2D {
public:
    bool Init();
    void UnInit();
    void Resize(int width, int height);
    void Begin();
    void AddQuad(float x0, float y0, float x1, float y1, float r, float g, float b, float a);
    void DrawLine(float x0, float y0, float x1, float y1, float thickness, float r, float g, float b, float a);
    void DrawText(float x, float y, float scale, const char* text, float r, float g, float b, float a);
    void Flush();

    int Width() const { return width_; }
    int Height() const { return height_; }

private:
    int width_ = 1280;
    int height_ = 720;
    unsigned program_ = 0;
    unsigned vao_ = 0;
    unsigned vbo_ = 0;
    unsigned ibo_ = 0;
    int uOrtho_ = -1;
    std::vector<float> verts_;
    std::vector<unsigned> indices_;
};

}  // namespace System
