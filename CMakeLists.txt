cmake_minimum_required(VERSION 3.25)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Configure into a separate build directory.")
endif()

project(CppTemplate VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(TEMPLATE_BUILD_APP "Build the game in app/" ON)
option(TEMPLATE_BUILD_EXAMPLES "Build the examples, such as the sandbox that shows every library" ON)
option(TEMPLATE_BUILD_TESTS "Build the unit tests" ON)
option(TEMPLATE_NET_SOCKETS "Build TCP and UDP sockets on top of standalone Asio (desktop only)" ON)
option(TEMPLATE_NET_RELAY "Build the WebSocket relay client and server (libdatachannel on desktop)" ON)
option(TEMPLATE_GPU_COMPUTE "Build libs/gpu: WebGPU compute (wgpu-native on desktop, the browser on the web)" OFF)
# Always on: a 1:10 Solar System is too large for float positions. A plain variable, so old build caches follow it.
set(CYCLONE_DOUBLE_PRECISION ON)

option(TEMPLATE_YAML_CONFIG "Load runtime settings from YAML with yaml-cpp" ON)

if(EMSCRIPTEN)
    # Single-threaded on purpose: wasm pthreads need COOP/COEP headers that static hosts can't send
    set(PLATFORM "Web" CACHE STRING "raylib platform" FORCE)
    set(OPENGL_VERSION "ES 3.0" CACHE STRING "WebGL2" FORCE)
    set(TEMPLATE_NET_SOCKETS OFF CACHE BOOL "" FORCE)
    set(TEMPLATE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fwasm-exceptions>)
    add_link_options(-fwasm-exceptions)
    # One list for everything: a second flag would replace this one. HEAPF32 is for raylib's web audio.
    add_link_options("-sEXPORTED_RUNTIME_METHODS=['ccall','UTF8ToString','HEAPF32']")
endif()

include(cmake/CPM.cmake)
include(cmake/Targets.cmake)

add_subdirectory(libs/vendor)
add_subdirectory(libs/cyclone)
add_subdirectory(libs/core)
add_subdirectory(libs/net)
add_subdirectory(libs/ai)
add_subdirectory(libs/engine)
if(TEMPLATE_GPU_COMPUTE)
    include(cmake/GpuDeps.cmake)
    add_subdirectory(libs/gpu)
endif()

if(TEMPLATE_NET_RELAY AND NOT EMSCRIPTEN)
    add_subdirectory(server)
endif()

if(TEMPLATE_BUILD_APP)
    add_subdirectory(app)
endif()

if(TEMPLATE_BUILD_EXAMPLES)
    add_subdirectory(examples/sandbox)
endif()

if(TEMPLATE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
