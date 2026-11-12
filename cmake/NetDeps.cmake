# Third-party networking libraries: Asio for UDP, libdatachannel + MbedTLS for WebSockets, nlohmann_json for lobby messages

if(TEMPLATE_NET_SOCKETS OR (TEMPLATE_NET_RELAY AND NOT EMSCRIPTEN))
    CPMAddPackage(
        NAME asio
        GITHUB_REPOSITORY chriskohlhoff/asio
        GIT_TAG asio-1-30-2
        DOWNLOAD_ONLY YES
    )
    add_library(asio INTERFACE)
    target_include_directories(asio SYSTEM INTERFACE ${asio_SOURCE_DIR}/asio/include)
    target_compile_definitions(asio INTERFACE ASIO_STANDALONE ASIO_NO_DEPRECATED)
    if(WIN32)
        target_compile_definitions(asio INTERFACE _WIN32_WINNT=0x0A00)
        target_link_libraries(asio INTERFACE ws2_32 mswsock)
    endif()
endif()

if(NOT TEMPLATE_NET_RELAY)
    return()
endif()

CPMAddPackage(
    NAME nlohmann_json
    GITHUB_REPOSITORY nlohmann/json
    VERSION 3.12.0
    OPTIONS "JSON_BuildTests OFF"
)

if(EMSCRIPTEN)
    return() # The browser provides WebSockets
endif()

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# libdatachannel needs DTLS-SRTP from MbedTLS, and both have to agree on the config
add_compile_definitions(MBEDTLS_SSL_DTLS_SRTP)

# MbedTLS first, then point libdatachannel's find logic at it instead of a system install
set(ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(ENABLE_PROGRAMS OFF CACHE BOOL "" FORCE)
set(MBEDTLS_FATAL_WARNINGS OFF CACHE BOOL "" FORCE)
CPMAddPackage(
    NAME mbedtls
    GITHUB_REPOSITORY Mbed-TLS/mbedtls
    GIT_TAG v3.6.2
)

set(MbedTLS_FOUND TRUE CACHE BOOL "" FORCE)
set(MbedTLS_VERSION "3.6.2" CACHE STRING "" FORCE)
set(MbedTLS_INCLUDE_DIRS "${mbedtls_SOURCE_DIR}/include" CACHE PATH "" FORCE)
set(MbedTLS_INCLUDE_DIR "${mbedtls_SOURCE_DIR}/include" CACHE PATH "" FORCE)
set(MbedTLS_LIBRARY mbedtls CACHE STRING "" FORCE)
set(MbedX509_LIBRARY mbedx509 CACHE STRING "" FORCE)
set(MbedCrypto_LIBRARY mbedcrypto CACHE STRING "" FORCE)
set(MBEDTLS_LIBRARIES mbedtls mbedx509 mbedcrypto CACHE STRING "" FORCE)
if(NOT TARGET MbedTLS::MbedTLS)
    # Imported, so libdatachannel's install(EXPORT) doesn't demand we export it too
    add_library(MbedTLS::MbedTLS INTERFACE IMPORTED GLOBAL)
    target_link_libraries(MbedTLS::MbedTLS INTERFACE mbedtls mbedx509 mbedcrypto)
endif()

set(USE_MBEDTLS ON CACHE BOOL "" FORCE)
set(NO_MEDIA ON CACHE BOOL "" FORCE)
set(NO_TESTS ON CACHE BOOL "" FORCE)
set(NO_EXAMPLES ON CACHE BOOL "" FORCE)
CPMAddPackage(
    NAME libdatachannel
    GITHUB_REPOSITORY paullouisageneau/libdatachannel
    GIT_TAG v0.24.1
    GIT_SUBMODULES_RECURSE YES
)

# Visual Studio generators need to be told where the MbedTLS .lib files land
if(TARGET datachannel-static)
    add_dependencies(datachannel-static mbedtls mbedcrypto mbedx509)
    target_link_directories(datachannel-static PUBLIC
        $<TARGET_FILE_DIR:mbedtls>
        $<TARGET_FILE_DIR:mbedx509>
        $<TARGET_FILE_DIR:mbedcrypto>
    )
endif()
