#include "net/UdpSocket.h"

#include <asio.hpp>

namespace net
{
    using asio::ip::udp;

    struct UdpSocket::Impl
    {
        asio::io_context context;
        udp::socket socket{ context };
    };

    UdpSocket::UdpSocket() : mpImpl(std::make_unique<Impl>())
    {
    }

    UdpSocket::~UdpSocket()
    {
        close();
    }

    bool UdpSocket::open(std::uint16_t port)
    {
        close();

        asio::error_code error;
        mpImpl->socket.open(udp::v4(), error);
        if (error)
        {
            return false;
        }

        mpImpl->socket.bind(udp::endpoint(udp::v4(), port), error);
        if (error)
        {
            close();
            return false;
        }

#ifdef _WIN32
        // Otherwise Windows fails the next receive whenever an earlier send reached a closed port
        DWORD reportResets = FALSE;
        DWORD bytesReturned = 0;
        WSAIoctl(mpImpl->socket.native_handle(), _WSAIOW(IOC_VENDOR, 12), &reportResets, sizeof(reportResets), nullptr, 0, &bytesReturned, nullptr, nullptr);
#endif

        mpImpl->socket.non_blocking(true, error);
        return !error;
    }

    void UdpSocket::close()
    {
        if (mpImpl->socket.is_open())
        {
            asio::error_code error;
            mpImpl->socket.close(error);
        }
    }

    bool UdpSocket::isOpen() const
    {
        return mpImpl->socket.is_open();
    }

    bool UdpSocket::sendTo(const Address& to, const std::uint8_t* data, std::size_t size)
    {
        asio::error_code error;
        asio::ip::address address = asio::ip::make_address(to.host, error);
        if (error)
        {
            return false;
        }

        std::size_t sent = mpImpl->socket.send_to(asio::buffer(data, size), udp::endpoint(address, to.port), 0, error);
        return !error && sent == size;
    }

    bool UdpSocket::broadcast(std::uint16_t port, const std::uint8_t* data, std::size_t size)
    {
        asio::error_code error;
        mpImpl->socket.set_option(asio::socket_base::broadcast(true), error);
        if (error)
        {
            return false;
        }

        udp::endpoint everyone(asio::ip::address_v4::broadcast(), port);
        std::size_t sent = mpImpl->socket.send_to(asio::buffer(data, size), everyone, 0, error);
        return !error && sent == size;
    }

    std::optional<std::size_t> UdpSocket::receive(Address& from, std::uint8_t* buffer, std::size_t capacity)
    {
        udp::endpoint sender;
        asio::error_code error;
        std::size_t received = mpImpl->socket.receive_from(asio::buffer(buffer, capacity), sender, 0, error);
        if (error)
        {
            return std::nullopt;
        }

        from.host = sender.address().to_string();
        from.port = sender.port();
        return received;
    }

    std::uint16_t UdpSocket::getLocalPort() const
    {
        asio::error_code error;
        udp::endpoint local = mpImpl->socket.local_endpoint(error);
        return error ? 0 : local.port();
    }
}
