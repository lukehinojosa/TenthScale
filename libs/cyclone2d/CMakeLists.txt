add_library(cyclone2d STATIC
    src/collision.cpp
    src/contactResolver.cpp
    src/core.cpp
    src/particle.cpp
    src/particleContacts.cpp
    src/particleForces.cpp
    src/particleWorld.cpp
    src/rigidBody.cpp
    src/rigidBodyForces.cpp
    src/world.cpp
)

# Consumers write <cyclone2d/cyclone2d.h>; it shares Cyclone's precision setting through <cyclone/precision.h>
target_include_directories(cyclone2d PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(cyclone2d PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/cyclone2d)
target_link_libraries(cyclone2d PUBLIC cyclone)
template_set_warnings(cyclone2d)
