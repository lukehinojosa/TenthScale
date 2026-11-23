add_library(engine STATIC
    src/Audio.cpp
    src/DrawUtils.cpp
    src/Game.cpp
    src/Scene.cpp
    src/SceneManager.cpp
    src/InputSystem.cpp
    src/FlyCamera.cpp
    src/ShaderUtils.cpp
    src/Lighting.cpp
    src/Sky.cpp
    src/Terrain.cpp
    src/GraphicsBuffer.cpp
    src/GraphicsBufferManager.cpp
    src/Animation.cpp
    src/Unit.cpp
)
target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(engine PUBLIC core cyclone cyclone2d raylib imgui rlImGui PRIVATE FastNoiseLite)
template_set_warnings(engine)
template_embed_shaders(engine ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
