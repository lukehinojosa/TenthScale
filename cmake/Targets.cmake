# Helpers for our own targets: warning flags, data folders, embedded shaders, and the web page.

set(TEMPLATE_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})

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

# Turns each .vert and .frag file in a folder into a header holding its source, so the target can include
# "shaders/<file>.h" and use it as shaders::<FILE>_VERT or shaders::<FILE>_FRAG. Editing one rebuilds what uses it.
function(template_embed_shaders target sourceDir)
    file(GLOB shaderFiles CONFIGURE_DEPENDS "${sourceDir}/*.vert" "${sourceDir}/*.frag")
    set(outDir "${CMAKE_CURRENT_BINARY_DIR}/${target}_shaders")
    set(headers)
    foreach(shader ${shaderFiles})
        get_filename_component(fileName "${shader}" NAME)
        string(TOUPPER "${fileName}" name)
        string(REPLACE "." "_" name "${name}")
        set(header "${outDir}/shaders/${fileName}.h")
        add_custom_command(OUTPUT "${header}"
            COMMAND ${CMAKE_COMMAND} -DINPUT=${shader} -DOUTPUT=${header} -DNAME=${name} -P "${TEMPLATE_CMAKE_DIR}/EmbedShader.cmake"
            DEPENDS "${shader}" "${TEMPLATE_CMAKE_DIR}/EmbedShader.cmake"
            COMMENT "Embedding ${fileName}"
        )
        list(APPEND headers "${header}")
    endforeach()
    target_sources(${target} PRIVATE ${headers})
    target_include_directories(${target} PRIVATE "${outDir}")
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
