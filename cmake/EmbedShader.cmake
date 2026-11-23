# Run as a script at build time: writes a header holding one shader file's source as a string.
# Takes INPUT (the shader file), OUTPUT (the header to write), and NAME (the constant to declare).
file(READ "${INPUT}" SOURCE)
get_filename_component(FILE_NAME "${INPUT}" NAME)
file(WRITE "${OUTPUT}" "#pragma once\n\n// Made at build time from ${FILE_NAME}; edit that file instead\nnamespace shaders\n{\n    inline constexpr const char* ${NAME} = R\"glsl(\n${SOURCE})glsl\";\n}\n")
