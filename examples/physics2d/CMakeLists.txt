# The 2D physics engine on its own; see the README for what it shows
add_executable(physics2d
    src/main.cpp
    src/Physics2DGame.cpp
)
target_link_libraries(physics2d PRIVATE engine)
template_set_warnings(physics2d)
template_add_data_dir(physics2d ${CMAKE_CURRENT_SOURCE_DIR}/config config)
template_make_web_page(physics2d physics2d)
