# Helpers for our own targets: warning flags, data folders, and the web page.

# Applies the project's warning flags to one of our own targets (never to vendor code)
function(template_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

# Copies a data folder next to the executable as <name>/, or preloads it at /<name> in the web build.
function(template_add_data_dir target sourceDir name)
    if(EMSCRIPTEN)
        # SHELL: stops CMake from merging repeated --preload-file flags into one
        target_link_options(${target} PRIVATE "SHELL:--preload-file \"${sourceDir}@/${name}\"")
        set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS ${sourceDir})
    else()
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${sourceDir}" "$<TARGET_FILE_DIR:${target}>/${name}"
            COMMENT "Copying ${name} for ${target}"
        )
    endif()
endfunction()

# Web build only: links the target as <pageName>.html inside the shared page shell.
function(template_make_web_page target pageName)
    if(NOT EMSCRIPTEN)
        return()
    endif()
    set_target_properties(${target} PROPERTIES SUFFIX ".html" OUTPUT_NAME "${pageName}")
    target_link_options(${target} PRIVATE
        "-sUSE_GLFW=3"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sSTACK_SIZE=1MB"
        "--shell-file=${PROJECT_SOURCE_DIR}/cmake/shell.html"
    )
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS ${PROJECT_SOURCE_DIR}/cmake/shell.html)
endfunction()
