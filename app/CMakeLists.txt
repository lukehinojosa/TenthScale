add_executable(app
    src/main.cpp
    src/Game.cpp
)
target_link_libraries(app PRIVATE engine)
template_set_warnings(app)
template_add_data_dir(app ${CMAKE_CURRENT_SOURCE_DIR}/assets assets)
template_add_data_dir(app ${CMAKE_CURRENT_SOURCE_DIR}/config config)
template_make_web_page(app index)
