add_library(core STATIC
    src/MemoryTracker.cpp
    src/MemoryPool.cpp
    src/MemoryManager.cpp
    src/EventSystem.cpp
    src/EventListener.cpp
    src/CallbackManager.cpp
)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
template_set_warnings(core)

if(TEMPLATE_YAML_CONFIG)
    # yaml-cpp 0.8.0 asks for an old CMake minimum that CMake 4 rejects; this only affects fetched code
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    CPMAddPackage(
        NAME yaml-cpp
        GITHUB_REPOSITORY jbeder/yaml-cpp
        GIT_TAG 0.8.0
        OPTIONS "YAML_CPP_BUILD_TESTS OFF" "YAML_CPP_BUILD_TOOLS OFF" "YAML_CPP_BUILD_CONTRIB OFF"
    )
    unset(CMAKE_POLICY_VERSION_MINIMUM)

    target_sources(core PRIVATE src/Config.cpp)
    target_link_libraries(core PUBLIC yaml-cpp::yaml-cpp)
    target_compile_definitions(core PUBLIC TEMPLATE_HAS_CONFIG)
endif()
