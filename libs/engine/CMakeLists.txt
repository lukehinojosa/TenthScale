add_library(engine STATIC
    src/Audio.cpp
    src/Game.cpp
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
target_link_libraries(engine PUBLIC core cyclone raylib imgui rlImGui PRIVATE FastNoiseLite)
template_set_warnings(engine)
