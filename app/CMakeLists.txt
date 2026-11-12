# Shared: the interface between the modules, plus time warp, the map view, and the camera (both)
add_library(shared STATIC
    src/shared/SolarSystem.cpp
)
target_include_directories(shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
template_set_warnings(shared)

# Solar System: planet and moon data, orbits, and sphere-of-influence checks (Calvin)
add_library(solar STATIC
    src/solar/CircularSolarSystem.cpp
)
target_link_libraries(solar PUBLIC shared)
template_set_warnings(solar)

# Rocket and Flight: rocket physics, engines, aerodynamics, landing, staging, the build screen, and the HUD (Luke)
add_library(flight STATIC
    src/flight/Rocket.cpp
)
target_link_libraries(flight PUBLIC shared cyclone)
template_set_warnings(flight)

add_executable(app
    src/main.cpp
    src/Game.cpp
)
target_link_libraries(app PRIVATE engine shared solar flight)
template_set_warnings(app)
template_add_data_dir(app ${CMAKE_CURRENT_SOURCE_DIR}/assets assets)
template_add_data_dir(app ${CMAKE_CURRENT_SOURCE_DIR}/config config)
template_make_web_page(app index)
