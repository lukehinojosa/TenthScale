#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace net
{
    enum class SocketEventType
    {
        OPENED,
        TEXT,
        BINARY,
        CLOSED,
        FAILED
    };

    struct SocketEvent
    {
        SocketEventType type = SocketEventType::CLOSED;
        std::string text; // TEXT payload, or the FAILED reason
        std::vector<std::uint8_t> binary;
    };

    // WebSocket client for desktop (libdatachannel) and the web (browser WebSocket).
    // Events queue up on whatever thread they arrive on; pollEvent hands them out on yours.
    class WebSocket
    {
    public:
        WebSocket();
        ~WebSocket();

        WebSocket(const WebSocket&) = delete;
        WebSocket& operator=(const WebSocket&) = delete;

        // ws:// or wss://; OPENED or FAILED arrives later through pollEvent
        void open(const std::string& url);
        void close();
        bool isOpen() const;

        bool sendText(const std::string& text);
        bool sendBinary(const std::uint8_t* data, std::size_t size);

        // Returns false once the queue is empty
        bool pollEvent(SocketEvent& outEvent);

        struct Impl;

    private:
        std::shared_ptr<Impl> mpImpl;
    };
}
