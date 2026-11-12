# Serialization and packet framing: header-only and portable, so it also builds for the web
add_library(net INTERFACE)
target_include_directories(net INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(net INTERFACE cxx_std_20)

include(${CMAKE_SOURCE_DIR}/cmake/NetDeps.cmake)

if(TEMPLATE_NET_SOCKETS)
    add_library(net_sockets STATIC
        src/UdpSocket.cpp
        src/TcpSocket.cpp
        src/LanDiscovery.cpp
    )
    target_link_libraries(net_sockets PUBLIC net PRIVATE asio)
    target_compile_definitions(net_sockets PUBLIC TEMPLATE_HAS_SOCKETS)
    template_set_warnings(net_sockets)
endif()

if(TEMPLATE_NET_RELAY)
    # WebSocket relay: libdatachannel on desktop, the browser's WebSocket on the web
    add_library(net_relay STATIC
        src/WebSocketNative.cpp
        src/WebSocketWeb.cpp
        src/RelayClient.cpp
    )
    target_link_libraries(net_relay PUBLIC net PRIVATE nlohmann_json::nlohmann_json)
    target_compile_definitions(net_relay PUBLIC TEMPLATE_HAS_RELAY)
    template_set_warnings(net_relay)

    # On the web, its JavaScript needs ccall and UTF8ToString, which the root CMakeLists exports
    if(NOT EMSCRIPTEN)
        target_link_libraries(net_relay PRIVATE datachannel-static mbedtls mbedx509 mbedcrypto)
    endif()
endif()
