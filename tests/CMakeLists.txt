CPMAddPackage("gh:doctest/doctest@2.4.12")

add_executable(tests
    main.cpp
    CoreTests.cpp
    NetTests.cpp
    NetSocketTests.cpp
    CycloneTests.cpp
    PhysicsTests.cpp
    AdvancedPhysicsTests.cpp
    Physics2DTests.cpp
    AiTests.cpp
    EngineTests.cpp
    SceneTests.cpp
    GpuTests.cpp
)
target_link_libraries(tests PRIVATE core net cyclone cyclone2d ai engine doctest::doctest)
template_set_warnings(tests)

if(TARGET net_sockets)
    target_link_libraries(tests PRIVATE net_sockets)
endif()

if(TARGET gpu)
    target_link_libraries(tests PRIVATE gpu)
    template_copy_webgpu_runtime(tests)
endif()

if(TARGET net_relay)
    target_link_libraries(tests PRIVATE net_relay)
endif()

if(TARGET flight)
    target_sources(tests PRIVATE SolarSystemTests.cpp FlightTests.cpp RocketDesignTests.cpp DesignStatsTests.cpp DesignFileTests.cpp VehicleTests.cpp RocketEngineTests.cpp StagingTests.cpp AerodynamicsTests.cpp OrbitInfoTests.cpp OrbitPathTests.cpp SteeringTests.cpp SasTests.cpp OrbitFlightTests.cpp TimeWarpTests.cpp LaunchPadTests.cpp)
    target_link_libraries(tests PRIVATE solar flight)
endif()

add_test(NAME tests COMMAND tests)
