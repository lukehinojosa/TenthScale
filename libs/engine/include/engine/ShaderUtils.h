#pragma once

#include <raylib.h>

namespace engine
{
    // Loads a shader from version-less GLSL source, adding "#version 330" on desktop
    // or "#version 300 es" plus precision qualifiers on the web, so one source runs on both
    Shader loadPortableShader(const char* vertexBody, const char* fragmentBody);
}
