# WebGPU for libs/gpu: the browser's own through Emscripten's emdawnwebgpu port, prebuilt wgpu-native on the desktop.
# Defines the `webgpu` target and template_copy_webgpu_runtime(target).

if(TARGET webgpu)
    return()
endif()

if(EMSCRIPTEN)
    add_library(webgpu INTERFACE)
    target_compile_options(webgpu INTERFACE --use-port=emdawnwebgpu)
    target_link_options(webgpu INTERFACE --use-port=emdawnwebgpu)

    function(template_copy_webgpu_runtime target)
    endfunction()
    return()
endif()

set(WGPU_NATIVE_VERSION "v29.0.1.1")
if(WIN32)
    set(wgpuArchive "wgpu-windows-x86_64-msvc-release.zip")
    set(wgpuHash "7e67d7445c42aeb85e30f88930fd8d7d83ee769e3390aeb1ada75ebf3cf78132")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(wgpuArchive "wgpu-linux-x86_64-release.zip")
    set(wgpuHash "95a4d90c071005a98d03eab348beaa6b07e16eb00d1dcdb9f8348f75eb97ec5a")
else()
    message(FATAL_ERROR "TEMPLATE_GPU_COMPUTE has prebuilt wgpu-native for Windows and Linux x86_64 only")
endif()

set(wgpuDir "${CMAKE_SOURCE_DIR}/.cpm-cache/wgpu-native/${WGPU_NATIVE_VERSION}")
if(NOT EXISTS "${wgpuDir}/.extracted")
    message(STATUS "Downloading wgpu-native ${WGPU_NATIVE_VERSION}")
    file(DOWNLOAD
        "https://github.com/gfx-rs/wgpu-native/releases/download/${WGPU_NATIVE_VERSION}/${wgpuArchive}"
        "${wgpuDir}/${wgpuArchive}"
        EXPECTED_HASH SHA256=${wgpuHash}
        SHOW_PROGRESS
    )
    file(ARCHIVE_EXTRACT INPUT "${wgpuDir}/${wgpuArchive}" DESTINATION "${wgpuDir}")
    file(REMOVE "${wgpuDir}/${wgpuArchive}")
    file(TOUCH "${wgpuDir}/.extracted")
endif()

add_library(webgpu SHARED IMPORTED GLOBAL)
set_target_properties(webgpu PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${wgpuDir}/include")
if(WIN32)
    set_target_properties(webgpu PROPERTIES
        IMPORTED_LOCATION "${wgpuDir}/lib/wgpu_native.dll"
        IMPORTED_IMPLIB "${wgpuDir}/lib/wgpu_native.dll.lib"
    )
else()
    set_target_properties(webgpu PROPERTIES IMPORTED_LOCATION "${wgpuDir}/lib/libwgpu_native.so")
endif()

# The shared library has to sit next to each executable that uses it
function(template_copy_webgpu_runtime target)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:webgpu>" "$<TARGET_FILE_DIR:${target}>"
    )
    if(NOT WIN32)
        set_target_properties(${target} PROPERTIES BUILD_RPATH "$ORIGIN")
    endif()
endfunction()
