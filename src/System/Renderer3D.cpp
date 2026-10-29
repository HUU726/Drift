#include "System/Renderer3D.h"

#include "System/GlCommon.h"

#include <cstdint>

namespace System {

namespace {

const char* kWorldVs = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uViewProj;
uniform mat4 uModel;
out vec3 vNormal;
out vec3 vColor;
out vec3 vWorld;
void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    vColor = aColor;
    gl_Position = uViewProj * world;
}
)GLSL";

const char* kWorldFs = R"GLSL(
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorld;
uniform vec3 uCamPos;
out vec4 fragColor;
void main() {
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing) n = -n;
    vec3 sun = normalize(vec3(0.35, 0.86, 0.28));
    float light = 0.36 + 0.64 * max(dot(n, sun), 0.0);
    vec3 color = vColor * light;
    float dist = distance(vWorld, uCamPos);
    float fog = clamp((dist - 90.0) / 1500.0, 0.0, 1.0);
    vec3 haze = vec3(0.62, 0.74, 0.84);
    fragColor = vec4(mix(color, haze, fog), 1.0);
}
)GLSL";

const char* kSkyVs = R"GLSL(
out float vY;
void main() {
    vec2 pos = vec2((gl_VertexID == 2) ? 3.0 : -1.0, (gl_VertexID == 1) ? 3.0 : -1.0);
    vY = pos.y;
    gl_Position = vec4(pos, 1.0, 1.0);
}
)GLSL";

const char* kSkyFs = R"GLSL(
in float vY;
out vec4 fragColor;
void main() {
    float t = clamp(vY * 0.5 + 0.5, 0.0, 1.0);
    vec3 horizon = vec3(0.62, 0.74, 0.84);
    vec3 zenith = vec3(0.28, 0.48, 0.78);
    fragColor = vec4(mix(horizon, zenith, smoothstep(0.0, 1.0, t)), 1.0);
}
)GLSL";

// 煙。頂点ごとに位置、板の中の位置（-1..1）、濃さを持つ。中心が濃く、縁へ向けてなめらかに消える丸。
const char* kSmokeVs = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aCorner;
layout(location = 2) in float aAlpha;
uniform mat4 uViewProj;
out vec2 vCorner;
out float vAlpha;
out vec3 vWorld;
void main() {
    vCorner = aCorner;
    vAlpha = aAlpha;
    vWorld = aPos;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

const char* kSmokeFs = R"GLSL(
in vec2 vCorner;
in float vAlpha;
in vec3 vWorld;
uniform vec3 uCamPos;
out vec4 fragColor;
void main() {
    float r = dot(vCorner, vCorner);
    float a = vAlpha * (1.0 - smoothstep(0.25, 1.0, r));
    if (a <= 0.003) discard;
    float dist = distance(vWorld, uCamPos);
    float fog = clamp((dist - 90.0) / 1500.0, 0.0, 1.0);
    vec3 color = mix(vec3(0.93, 0.93, 0.94), vec3(0.62, 0.74, 0.84), fog);
    fragColor = vec4(color, a);
}
)GLSL";

}  // namespace

bool Renderer3D::Init() {
    glGenVertexArrays(1, &dummyVao_);
    worldProgram_ = Gl::LinkProgram(kWorldVs, kWorldFs);
    skyProgram_ = Gl::LinkProgram(kSkyVs, kSkyFs);
    if (!worldProgram_ || !skyProgram_) {
        return false;
    }
    uViewProj_ = glGetUniformLocation(worldProgram_, "uViewProj");
    uModel_ = glGetUniformLocation(worldProgram_, "uModel");
    uCamPos_ = glGetUniformLocation(worldProgram_, "uCamPos");

    smokeProgram_ = Gl::LinkProgram(kSmokeVs, kSmokeFs);
    if (!smokeProgram_) {
        return false;
    }
    uSmokeViewProj_ = glGetUniformLocation(smokeProgram_, "uViewProj");
    uSmokeCamPos_ = glGetUniformLocation(smokeProgram_, "uCamPos");
    glGenVertexArrays(1, &smokeVao_);
    glGenBuffers(1, &smokeVbo_);
    glBindVertexArray(smokeVao_);
    glBindBuffer(GL_ARRAY_BUFFER, smokeVbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(5 * sizeof(float)));
    glBindVertexArray(dummyVao_);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    // lookAt は左右を直すために鏡映になっている。巻き方向が反転するので、表の判定も合わせる。
    glFrontFace(GL_CW);
    Resize(width_, height_);
    return true;
}

void Renderer3D::UnInit() {
    glDeleteProgram(worldProgram_);
    glDeleteProgram(skyProgram_);
    glDeleteProgram(smokeProgram_);
    glDeleteBuffers(1, &smokeVbo_);
    glDeleteVertexArrays(1, &smokeVao_);
    glDeleteVertexArrays(1, &dummyVao_);
    worldProgram_ = 0;
    skyProgram_ = 0;
    smokeProgram_ = 0;
    smokeVbo_ = 0;
    smokeVao_ = 0;
    dummyVao_ = 0;
}

void Renderer3D::Resize(int width, int height) {
    width_ = width > 0 ? width : 1;
    height_ = height > 0 ? height : 1;
    glViewport(0, 0, width_, height_);
}

void Renderer3D::BeginFrame(const Mat4& viewProj, Vec3 cameraPos) {
    glClearColor(0.62f, 0.74f, 0.84f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawSky();

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(worldProgram_);
    glUniformMatrix4fv(uViewProj_, 1, GL_FALSE, viewProj.m);
    glUniform3f(uCamPos_, cameraPos.x, cameraPos.y, cameraPos.z);
    viewProj_ = viewProj;
    cameraPos_ = cameraPos;
}

void Renderer3D::DrawSmoke(const std::vector<SmokePuff>& puffs, Vec3 camRight, Vec3 camUp) {
    if (puffs.empty()) {
        return;
    }
    // 1 粒を 2 つの三角形（6 頂点）の板にする。
    smokeVertices_.clear();
    smokeVertices_.reserve(puffs.size() * 36);
    const float corners[6][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, -1}, {1, 1}, {-1, 1}};
    for (const SmokePuff& puff : puffs) {
        float half = puff.size * 0.5f;
        for (const auto& c : corners) {
            Vec3 p = puff.position + camRight * (c[0] * half) + camUp * (c[1] * half);
            smokeVertices_.insert(smokeVertices_.end(), {p.x, p.y, p.z, c[0], c[1], puff.alpha});
        }
    }
    // 半透明なので、奥行きは読むが書かない。重なりの順は気にしない（白い煙なので目立たない）。
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUseProgram(smokeProgram_);
    glUniformMatrix4fv(uSmokeViewProj_, 1, GL_FALSE, viewProj_.m);
    glUniform3f(uSmokeCamPos_, cameraPos_.x, cameraPos_.y, cameraPos_.z);
    glBindVertexArray(smokeVao_);
    glBindBuffer(GL_ARRAY_BUFFER, smokeVbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(smokeVertices_.size() * sizeof(float)), smokeVertices_.data(),
                 GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(smokeVertices_.size() / 6));
    // 後に描くもののために、ワールド描画の状態へ戻す。
    glBindVertexArray(dummyVao_);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glUseProgram(worldProgram_);
}

void Renderer3D::drawSky() {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glUseProgram(skyProgram_);
    glBindVertexArray(dummyVao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void Renderer3D::Draw(const GpuMesh& mesh, const Mat4& model) {
    if (mesh.indexCount <= 0) {
        return;
    }
    glBindVertexArray(mesh.vao);
    glUniformMatrix4fv(uModel_, 1, GL_FALSE, model.m);
    glDrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_INT, nullptr);
}

GpuMesh Renderer3D::Upload(const CpuMesh& mesh) {
    GpuMesh gpu;
    if (mesh.indices.empty()) {
        return gpu;
    }
    glGenVertexArrays(1, &gpu.vao);
    glGenBuffers(1, &gpu.vbo);
    glGenBuffers(1, &gpu.ibo);
    glBindVertexArray(gpu.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(float)), mesh.vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(std::uint32_t)), mesh.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
    gpu.indexCount = static_cast<int>(mesh.indices.size());
    glBindVertexArray(dummyVao_);
    return gpu;
}

void Renderer3D::Release(GpuMesh& mesh) {
    glDeleteBuffers(1, &mesh.vbo);
    glDeleteBuffers(1, &mesh.ibo);
    glDeleteVertexArrays(1, &mesh.vao);
    mesh = GpuMesh{};
}

}  // namespace System
