add_executable(relay_server relay/main.cpp)
target_link_libraries(relay_server PRIVATE
    datachannel-static
    mbedtls
    mbedx509
    mbedcrypto
    asio
    nlohmann_json::nlohmann_json
)
template_set_warnings(relay_server)
