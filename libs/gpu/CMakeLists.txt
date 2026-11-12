# GPU compute on WebGPU: runs the same WGSL on the desktop (wgpu-native) and in the browser
add_library(gpu STATIC
    src/Context.cpp
    src/Compute.cpp
    src/BallSimulation.cpp
    src/BallShader.cpp
)
target_include_directories(gpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(gpu PUBLIC cxx_std_20)
target_link_libraries(gpu PUBLIC webgpu)
target_compile_definitions(gpu PUBLIC TEMPLATE_HAS_GPU_COMPUTE)
template_set_warnings(gpu)
