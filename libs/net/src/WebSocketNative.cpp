#ifndef __EMSCRIPTEN__

#include "net/WebSocket.h"

#include <deque>
#include <mutex>

#include <rtc/rtc.hpp>

namespace net
{
    struct WebSocket::Impl
    {
        std::shared_ptr<rtc::WebSocket> socket;
        std::mutex mutex;
        std::deque<SocketEvent> events;

        void push(SocketEvent event)
        {
            std::lock_guard<std::mutex> lock(mutex);
            events.push_back(std::move(event));
        }
    };

    WebSocket::WebSocket() : mpImpl(std::make_shared<Impl>())
    {
    }

    WebSocket::~WebSocket()
    {
        close();
    }

    void WebSocket::open(const std::string& url)
    {
        close();
        mpImpl->socket = std::make_shared<rtc::WebSocket>();

        // Weak captures: libdatachannel calls these from its own thread, possibly after we are gone
        std::weak_ptr<Impl> weakImpl = mpImpl;

        mpImpl->socket->onOpen([weakImpl]()
        {
            if (auto pImpl = weakImpl.lock())
            {
                pImpl->push(SocketEvent{ SocketEventType::OPENED, {}, {} });
            }
        });

        mpImpl->socket->onMessage([weakImpl](rtc::message_variant data)
        {
            auto pImpl = weakImpl.lock();
            if (!pImpl)
            {
                return;
            }

            SocketEvent event;
            if (std::holds_alternative<std::string>(data))
            {
                event.type = SocketEventType::TEXT;
                event.text = std::move(std::get<std::string>(data));
            }
            else
            {
                const rtc::binary& bytes = std::get<rtc::binary>(data);
                event.type = SocketEventType::BINARY;
                event.binary.resize(bytes.size());
                for (std::size_t i = 0; i < bytes.size(); i++)
                {
                    event.binary[i] = static_cast<std::uint8_t>(bytes[i]);
                }
            }
            pImpl->push(std::move(event));
        });

        mpImpl->socket->onError([weakImpl](std::string error)
        {
            if (auto pImpl = weakImpl.lock())
            {
                pImpl->push(SocketEvent{ SocketEventType::FAILED, std::move(error), {} });
            }
        });

        mpImpl->socket->onClosed([weakImpl]()
        {
            if (auto pImpl = weakImpl.lock())
            {
                pImpl->push(SocketEvent{ SocketEventType::CLOSED, {}, {} });
            }
        });

        try
        {
            mpImpl->socket->open(url);
        }
        catch (const std::exception& e)
        {
            mpImpl->push(SocketEvent{ SocketEventType::FAILED, e.what(), {} });
        }
    }

    void WebSocket::close()
    {
        if (mpImpl->socket)
        {
            mpImpl->socket->resetCallbacks();
            mpImpl->socket->close();
            mpImpl->socket.reset();
        }
        std::lock_guard<std::mutex> lock(mpImpl->mutex);
        mpImpl->events.clear();
    }

    bool WebSocket::isOpen() const
    {
        return mpImpl->socket && mpImpl->socket->isOpen();
    }

    bool WebSocket::sendText(const std::string& text)
    {
        if (!isOpen())
        {
            return false;
        }
        return mpImpl->socket->send(text);
    }

    bool WebSocket::sendBinary(const std::uint8_t* data, std::size_t size)
    {
        if (!isOpen())
        {
            return false;
        }
        return mpImpl->socket->send(reinterpret_cast<const std::byte*>(data), size);
    }

    bool WebSocket::pollEvent(SocketEvent& outEvent)
    {
        std::lock_guard<std::mutex> lock(mpImpl->mutex);
        if (mpImpl->events.empty())
        {
            return false;
        }
        outEvent = std::move(mpImpl->events.front());
        mpImpl->events.pop_front();
        return true;
    }
}

#endif
