# Every library working together; see the README for what it shows
add_executable(sandbox
    src/main.cpp
    src/SandboxGame.cpp
    src/InputTranslator.cpp
)
target_link_libraries(sandbox PRIVATE engine net ai)
if(TARGET net_sockets)
    target_link_libraries(sandbox PRIVATE net_sockets)
endif()
if(TARGET net_relay)
    target_link_libraries(sandbox PRIVATE net_relay)
endif()
if(TARGET gpu)
    target_link_libraries(sandbox PRIVATE gpu)
    template_copy_webgpu_runtime(sandbox)
endif()
template_set_warnings(sandbox)
template_add_data_dir(sandbox ${CMAKE_CURRENT_SOURCE_DIR}/config config)
template_make_web_page(sandbox sandbox)
