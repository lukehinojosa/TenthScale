set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
add_subdirectory(raylib)

add_library(imgui STATIC
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_demo.cpp
)
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imgui)

add_library(rlImGui STATIC rlImGui/rlImGui.cpp)
target_include_directories(rlImGui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/rlImGui)
target_link_libraries(rlImGui PUBLIC raylib imgui)

# Single-header noise library (MIT, Jordan Peck)
add_library(FastNoiseLite INTERFACE)
target_include_directories(FastNoiseLite SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/FastNoiseLite)
