#include "System/Renderer2D.h"

#include "System/GlCommon.h"
#include "geom.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace System {

namespace {

const char* kHudVs = R"GLSL(
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uOrtho;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uOrtho * vec4(aPos, 0.0, 1.0);
}
)GLSL";

const char* kHudFs = R"GLSL(
in vec4 vColor;
out vec4 fragColor;
void main() {
    fragColor = vColor;
}
)GLSL";

// 5x7。bit4 が左端。並びは 0-9 A-Z . : - / %
constexpr char kGlyphOrder[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.:-/%";
const std::uint8_t kFont[][7] = {
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E},
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}, {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}, {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    {0x0E, 0x11, 0x10, 0x0E, 0x01, 0x11, 0x0E}, {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}, {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}, {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00},
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}, {0x01, 0x02, 0x04, 0x04, 0x08, 0x10, 0x10},
    {0x19, 0x1A, 0x02, 0x04, 0x08, 0x0B, 0x13},
};
static_assert(sizeof(kFont) / sizeof(kFont[0]) == sizeof(kGlyphOrder) - 1, "glyph count");

int glyphIndex(char c) {
    if (c >= 'a' && c <= 'z') {
        c = static_cast<char>(c - 'a' + 'A');
    }
    const char* found = std::strchr(kGlyphOrder, c);
    if (!found) {
        return -1;
    }
    return static_cast<int>(found - kGlyphOrder);
}

}  // namespace

bool Renderer2D::Init() {
    program_ = Gl::LinkProgram(kHudVs, kHudFs);
    if (!program_) {
        return false;
    }
    uOrtho_ = glGetUniformLocation(program_, "uOrtho");

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ibo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(2 * sizeof(float)));
    glBindVertexArray(0);
    return true;
}

void Renderer2D::UnInit() {
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ibo_);
    glDeleteVertexArrays(1, &vao_);
    glDeleteProgram(program_);
    vbo_ = 0;
    ibo_ = 0;
    vao_ = 0;
    program_ = 0;
}

void Renderer2D::Resize(int width, int height) {
    width_ = width > 0 ? width : 1;
    height_ = height > 0 ? height : 1;
}

void Renderer2D::Begin() {
    verts_.clear();
    indices_.clear();
}

void Renderer2D::AddQuad(float x0, float y0, float x1, float y1, float r, float g, float b, float a) {
    unsigned base = static_cast<unsigned>(verts_.size() / 6);
    float verts[] = {
        x0, y0, r, g, b, a, x1, y0, r, g, b, a, x1, y1, r, g, b, a, x0, y1, r, g, b, a,
    };
    verts_.insert(verts_.end(), verts, verts + 24);
    unsigned idx[] = {base, base + 1, base + 2, base, base + 2, base + 3};
    indices_.insert(indices_.end(), idx, idx + 6);
}

void Renderer2D::DrawLine(float x0, float y0, float x1, float y1, float thickness, float r, float g, float b, float a) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.001f) {
        return;
    }
    dx /= len;
    dy /= len;
    float px = -dy * thickness * 0.5f;
    float py = dx * thickness * 0.5f;
    unsigned base = static_cast<unsigned>(verts_.size() / 6);
    float verts[] = {
        x0 + px, y0 + py, r, g, b, a, x0 - px, y0 - py, r, g, b, a,
        x1 - px, y1 - py, r, g, b, a, x1 + px, y1 + py, r, g, b, a,
    };
    verts_.insert(verts_.end(), verts, verts + 24);
    unsigned idx[] = {base, base + 1, base + 2, base, base + 2, base + 3};
    indices_.insert(indices_.end(), idx, idx + 6);
}

void Renderer2D::DrawText(float x, float y, float scale, const char* text, float r, float g, float b, float a) {
    float pen = x;
    for (const char* cursor = text; *cursor; ++cursor) {
        char c = *cursor;
        if (c >= 'a' && c <= 'z') {
            c = static_cast<char>(c - 'a' + 'A');
        }
        if (c == ' ') {
            pen += 4.0f * scale;
            continue;
        }
        int index = glyphIndex(c);
        if (index < 0) {
            pen += 6.0f * scale;
            continue;
        }
        const std::uint8_t* rows = kFont[index];
        for (int row = 0; row < 7; ++row) {
            for (int col = 0; col < 5; ++col) {
                if ((rows[row] & (0x10 >> col)) == 0) {
                    continue;
                }
                float x0 = pen + float(col) * scale;
                float y0 = y + float(row) * scale;
                AddQuad(x0, y0, x0 + scale, y0 + scale, r, g, b, a);
            }
        }
        pen += 6.0f * scale;
    }
}

void Renderer2D::Flush() {
    if (indices_.empty()) {
        return;
    }
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(program_);
    Mat4 ortho = orthoTopLeft(float(width_), float(height_));
    glUniformMatrix4fv(uOrtho_, 1, GL_FALSE, ortho.m);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts_.size() * sizeof(float)), verts_.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices_.size() * sizeof(unsigned)), indices_.data(), GL_DYNAMIC_DRAW);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices_.size()), GL_UNSIGNED_INT, nullptr);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

}  // namespace System
