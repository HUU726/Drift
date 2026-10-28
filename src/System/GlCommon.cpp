#include "System/GlCommon.h"

#include <SDL.h>

namespace System::Gl {

namespace {

const char* kVersion =
#ifdef __EMSCRIPTEN__
    "#version 300 es\nprecision highp float;\n";
#else
    "#version 330 core\n";
#endif

GLuint compileShader(GLenum type, const char* source) {
    const char* parts[] = {kVersion, source};
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 2, parts, nullptr);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        SDL_Log("shader compile failed: %s", log);
        return 0;
    }
    return shader;
}

}  // namespace

GLuint LinkProgram(const char* vs, const char* fs) {
    GLuint vertex = compileShader(GL_VERTEX_SHADER, vs);
    GLuint fragment = compileShader(GL_FRAGMENT_SHADER, fs);
    if (!vertex || !fragment) {
        return 0;
    }
    GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        SDL_Log("shader link failed: %s", log);
        return 0;
    }
    return program;
}

}  // namespace System::Gl
