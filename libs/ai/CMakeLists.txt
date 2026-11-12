# Engine-agnostic game AI: steering and flocking, grid pathfinding, maze generation, cellular automata
add_library(ai STATIC
    src/Flocking.cpp
    src/GridPath.cpp
    src/Maze.cpp
)
target_include_directories(ai PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
template_set_warnings(ai)
