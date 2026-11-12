#include "net/TcpSocket.h"

#include <algorithm>

#include <asio.hpp>

namespace net
{
    using asio::ip::tcp;

    namespace
    {
        constexpr std::size_t HEADER_SIZE = 4; // Big-endian message length

        void configure(tcp::socket& socket)
        {
            asio::error_code error;
            socket.non_blocking(true, error);
            socket.set_option(tcp::no_delay(true), error); // Games send small messages that shouldn't wait to be batched
            socket.set_option(asio::socket_base::keep_alive(true), error); // Lets the OS notice a peer that vanished
        }
    }

    struct TcpConnection::Impl
    {
        asio::io_context context;
        tcp::socket socket{ context };
        std::vector<std::uint8_t> outgoing;
        std::vector<std::uint8_t> incoming;
        bool connected = false;
    };

    TcpConnection::TcpConnection() : mpImpl(std::make_unique<Impl>())
    {
    }

    TcpConnection::~TcpConnection()
    {
        disconnect();
    }

    bool TcpConnection::connect(const Address& to)
    {
        disconnect();

        asio::error_code error;
        tcp::resolver resolver(mpImpl->context);
        auto endpoints = resolver.resolve(to.host, std::to_string(to.port), error);
        if (error)
        {
            return false;
        }

        asio::connect(mpImpl->socket, endpoints, error);
        if (error)
        {
            mpImpl->socket.close(error);
            return false;
        }

        configure(mpImpl->socket);
        mpImpl->connected = true;
        return true;
    }

    void TcpConnection::disconnect()
    {
        if (mpImpl->socket.is_open())
        {
            pump();
            asio::error_code error;
            mpImpl->socket.shutdown(tcp::socket::shutdown_both, error);
            mpImpl->socket.close(error);
        }
        mpImpl->connected = false;
        mpImpl->outgoing.clear();
        mpImpl->incoming.clear();
    }

    bool TcpConnection::isConnected() const
    {
        return mpImpl->connected;
    }

    bool TcpConnection::send(const std::uint8_t* data, std::size_t size)
    {
        if (!mpImpl->connected || size > kMaxMessageSize)
        {
            return false;
        }

        auto& out = mpImpl->outgoing;
        out.push_back((std::uint8_t)(size >> 24));
        out.push_back((std::uint8_t)(size >> 16));
        out.push_back((std::uint8_t)(size >> 8));
        out.push_back((std::uint8_t)size);
        out.insert(out.end(), data, data + size);
        pump();
        return true;
    }

    std::optional<std::vector<std::uint8_t>> TcpConnection::receive()
    {
        pump();

        auto& in = mpImpl->incoming;
        if (in.size() < HEADER_SIZE)
        {
            return std::nullopt;
        }

        std::size_t size = ((std::size_t)in[0] << 24) | ((std::size_t)in[1] << 16) | ((std::size_t)in[2] << 8) | in[3];
        if (size > kMaxMessageSize)
        {
            // The other side isn't speaking this protocol, so nothing after this can be trusted
            disconnect();
            return std::nullopt;
        }
        if (in.size() < HEADER_SIZE + size)
        {
            return std::nullopt;
        }

        std::vector<std::uint8_t> message(in.begin() + HEADER_SIZE, in.begin() + HEADER_SIZE + size);
        in.erase(in.begin(), in.begin() + HEADER_SIZE + size);
        return message;
    }

    std::size_t TcpConnection::getPendingSendBytes() const
    {
        return mpImpl->outgoing.size();
    }

    Address TcpConnection::getRemoteAddress() const
    {
        asio::error_code error;
        tcp::endpoint remote = mpImpl->socket.remote_endpoint(error);
        if (error)
        {
            return Address{ "", 0 };
        }
        return Address{ remote.address().to_string(), remote.port() };
    }

    void TcpConnection::pump()
    {
        if (!mpImpl->connected)
        {
            return;
        }

        asio::error_code error;
        auto& out = mpImpl->outgoing;
        while (!out.empty())
        {
            std::size_t written = mpImpl->socket.write_some(asio::buffer(out), error);
            if (error)
            {
                break;
            }
            out.erase(out.begin(), out.begin() + (std::ptrdiff_t)written);
        }
        if (error && error != asio::error::would_block)
        {
            mpImpl->connected = false;
            return;
        }

        std::uint8_t buffer[16 * 1024];
        while (true)
        {
            std::size_t read = mpImpl->socket.read_some(asio::buffer(buffer), error);
            if (error)
            {
                break;
            }
            mpImpl->incoming.insert(mpImpl->incoming.end(), buffer, buffer + read);
        }

        // End of file or a reset means the peer is gone; messages already received can still be read
        if (error != asio::error::would_block)
        {
            mpImpl->connected = false;
        }
    }

    struct TcpListener::Impl
    {
        asio::io_context context;
        tcp::acceptor acceptor{ context };
    };

    TcpListener::TcpListener() : mpImpl(std::make_unique<Impl>())
    {
    }

    TcpListener::~TcpListener()
    {
        close();
    }

    bool TcpListener::open(std::uint16_t port)
    {
        close();

        asio::error_code error;
        tcp::endpoint endpoint(tcp::v4(), port);
        mpImpl->acceptor.open(endpoint.protocol(), error);
        if (!error)
        {
            mpImpl->acceptor.set_option(tcp::acceptor::reuse_address(true), error);
            mpImpl->acceptor.bind(endpoint, error);
        }
        if (!error)
        {
            mpImpl->acceptor.listen(asio::socket_base::max_listen_connections, error);
        }
        if (!error)
        {
            mpImpl->acceptor.non_blocking(true, error);
        }
        if (error)
        {
            close();
            return false;
        }
        return true;
    }

    void TcpListener::close()
    {
        if (mpImpl->acceptor.is_open())
        {
            asio::error_code error;
            mpImpl->acceptor.close(error);
        }
    }

    bool TcpListener::isOpen() const
    {
        return mpImpl->acceptor.is_open();
    }

    std::uint16_t TcpListener::getLocalPort() const
    {
        asio::error_code error;
        tcp::endpoint local = mpImpl->acceptor.local_endpoint(error);
        return error ? 0 : local.port();
    }

    std::unique_ptr<TcpConnection> TcpListener::accept()
    {
        if (!mpImpl->acceptor.is_open())
        {
            return nullptr;
        }

        auto pConnection = std::make_unique<TcpConnection>();
        asio::error_code error;
        mpImpl->acceptor.accept(pConnection->mpImpl->socket, error);
        if (error)
        {
            return nullptr;
        }

        configure(pConnection->mpImpl->socket);
        pConnection->mpImpl->connected = true;
        return pConnection;
    }
}
