#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace net
{
    struct Address
    {
        std::string host = "127.0.0.1";
        std::uint16_t port = 0;
    };

    // The largest payload that crosses the internet without being split into fragments
    constexpr std::size_t kMaxDatagramSize = 1200;

    // Non-blocking UDP socket, safe to poll once per frame (desktop only, built on Asio)
    class UdpSocket
    {
    public:
        UdpSocket();
        ~UdpSocket();

        UdpSocket(const UdpSocket&) = delete;
        UdpSocket& operator=(const UdpSocket&) = delete;

        // Port 0 lets the OS pick a free one
        bool open(std::uint16_t port = 0);
        void close();
        bool isOpen() const;

        bool sendTo(const Address& to, const std::uint8_t* data, std::size_t size);

        // Sends to every machine on the local network listening on `port`
        bool broadcast(std::uint16_t port, const std::uint8_t* data, std::size_t size);

        // Returns the datagram size, or nothing when no datagram is waiting
        std::optional<std::size_t> receive(Address& from, std::uint8_t* buffer, std::size_t capacity);

        std::uint16_t getLocalPort() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> mpImpl;
    };
}
