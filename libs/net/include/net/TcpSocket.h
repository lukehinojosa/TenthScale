#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "UdpSocket.h"

namespace net
{
    // A TCP connection that sends and receives whole messages, safe to poll once per frame (desktop only, built on Asio)
    class TcpConnection
    {
    public:
        static constexpr std::size_t kMaxMessageSize = 1024 * 1024;

        TcpConnection();
        ~TcpConnection();

        TcpConnection(const TcpConnection&) = delete;
        TcpConnection& operator=(const TcpConnection&) = delete;

        // Blocks until connected or refused; the host can be a name or an IP
        bool connect(const Address& to);

        // Sends what it can of the queue, then shuts the connection down cleanly
        void disconnect();
        bool isConnected() const;

        // Queues one message; false when disconnected or the message is too big
        bool send(const std::uint8_t* data, std::size_t size);

        // The next whole message, or nothing when none has fully arrived
        std::optional<std::vector<std::uint8_t>> receive();

        // Bytes queued but not yet handed to the OS, for when the other side reads slowly
        std::size_t getPendingSendBytes() const;
        Address getRemoteAddress() const;

    private:
        friend class TcpListener;

        void pump();

        struct Impl;
        std::unique_ptr<Impl> mpImpl;
    };

    // Accepts incoming TCP connections without blocking
    class TcpListener
    {
    public:
        TcpListener();
        ~TcpListener();

        TcpListener(const TcpListener&) = delete;
        TcpListener& operator=(const TcpListener&) = delete;

        // Port 0 lets the OS pick a free one
        bool open(std::uint16_t port = 0);
        void close();
        bool isOpen() const;
        std::uint16_t getLocalPort() const;

        // A new connection, or null when nobody is waiting
        std::unique_ptr<TcpConnection> accept();

    private:
        struct Impl;
        std::unique_ptr<Impl> mpImpl;
    };
}
