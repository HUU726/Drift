#pragma once

// Renderer3D と Renderer2D が共有する GL のヘッダとシェーダの組み立て。System の外からは使わない。

#ifdef __EMSCRIPTEN__
#include <GLES3/gl3.h>
#else
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>
#endif

namespace System::Gl {

// 頂点シェーダとフラグメントシェーダを結合する。#version 行は環境に合わせて足す。失敗したら 0。
GLuint LinkProgram(const char* vs, const char* fs);

}  // namespace System::Gl
