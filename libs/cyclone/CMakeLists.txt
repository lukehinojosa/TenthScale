add_library(cyclone STATIC
    src/broadPhase.cpp
    src/collision.cpp
    src/constraintRope.cpp
    src/contactGeneration.cpp
    src/convex.cpp
    src/core.cpp
    src/forceGenerator.cpp
    src/fracture.cpp
    src/joints.cpp
    src/octree.cpp
    src/particle.cpp
    src/particleContacts.cpp
    src/particleWorld.cpp
    src/rigidBody.cpp
    src/rigidBodyForces.cpp
    src/rigidContactResolver.cpp
    src/simultaneousResolver.cpp
    src/threadPool.cpp
    src/uniformGrid.cpp
    src/verletRope.cpp
    src/world.cpp
)

# Consumers write <cyclone/cyclone.h>; the sources include their siblings directly
# SYSTEM keeps Cyclone's nameless unions from tripping /W4 in code that includes it
target_include_directories(cyclone SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(cyclone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/cyclone)

find_package(Threads REQUIRED)
target_link_libraries(cyclone PUBLIC Threads::Threads)

if(CYCLONE_DOUBLE_PRECISION)
    target_compile_definitions(cyclone PUBLIC CYCLONE_DOUBLE_PRECISION)
endif()
