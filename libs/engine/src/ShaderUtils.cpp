#include "engine/ShaderUtils.h"

#include <string>

namespace engine
{
    Shader loadPortableShader(const char* vertexBody, const char* fragmentBody)
    {
#ifdef __EMSCRIPTEN__
        const std::string header = "#version 300 es\nprecision highp float;\nprecision highp int;\nprecision highp sampler2D;\n";
#else
        const std::string header = "#version 330\n";
#endif
        std::string vertex = header + vertexBody;
        std::string fragment = header + fragmentBody;
        return LoadShaderFromMemory(vertex.c_str(), fragment.c_str());
    }
}
