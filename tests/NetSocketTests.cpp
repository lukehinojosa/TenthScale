#include <doctest/doctest.h>

#ifdef TEMPLATE_HAS_SOCKETS
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <net/LanDiscovery.h>
#include <net/TcpSocket.h>
#include <net/UdpSocket.h>

namespace
{
    // Polls until the condition holds or two seconds pass
    bool waitFor(const std::function<bool()>& condition)
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (condition())
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return condition();
    }

    std::vector<std::uint8_t> bytes(std::initializer_list<std::uint8_t> values)
    {
        return std::vector<std::uint8_t>(values);
    }
}

TEST_CASE("UdpSocket sends a datagram over loopback")
{
    net::UdpSocket server;
    net::UdpSocket client;
    REQUIRE(server.open());
    REQUIRE(client.open());

    auto hello = bytes({ 1, 2, 3 });
    REQUIRE(client.sendTo(net::Address{ "127.0.0.1", server.getLocalPort() }, hello.data(), hello.size()));

    std::uint8_t buffer[net::kMaxDatagramSize];
    net::Address from;
    std::optional<std::size_t> size;
    REQUIRE(waitFor([&]() { return (size = server.receive(from, buffer, sizeof(buffer))).has_value(); }));
    CHECK(*size == 3);
    CHECK(buffer[2] == 3);
    CHECK(from.port == client.getLocalPort());

    // Sending to a port nobody listens on must not break later receives (a Windows quirk)
    std::uint16_t closedPort = 0;
    {
        net::UdpSocket gone;
        REQUIRE(gone.open());
        closedPort = gone.getLocalPort();
    }
    client.sendTo(net::Address{ "127.0.0.1", closedPort }, hello.data(), hello.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    REQUIRE(server.sendTo(net::Address{ "127.0.0.1", client.getLocalPort() }, hello.data(), hello.size()));
    CHECK(waitFor([&]() { return client.receive(from, buffer, sizeof(buffer)).has_value(); }));
}

TEST_CASE("TcpConnection delivers whole messages in order, however large")
{
    net::TcpListener listener;
    REQUIRE(listener.open());

    net::TcpConnection client;
    REQUIRE(client.connect(net::Address{ "127.0.0.1", listener.getLocalPort() }));

    std::unique_ptr<net::TcpConnection> pServer;
    REQUIRE(waitFor([&]() { return (pServer = listener.accept()) != nullptr; }));
    CHECK(pServer->isConnected());
    CHECK(pServer->getRemoteAddress().host == "127.0.0.1");

    // Big enough that the OS takes it in pieces on both ends
    std::vector<std::uint8_t> big(300 * 1024);
    for (std::size_t i = 0; i < big.size(); i++)
    {
        big[i] = (std::uint8_t)(i * 31);
    }
    auto small = bytes({ 7 });
    REQUIRE(client.send(small.data(), small.size()));
    REQUIRE(client.send(big.data(), big.size()));
    REQUIRE(client.send(nullptr, 0));

    std::vector<std::vector<std::uint8_t>> received;
    REQUIRE(waitFor([&]()
    {
        client.receive(); // Keeps flushing the client's queue
        while (auto message = pServer->receive())
        {
            received.push_back(*message);
        }
        return received.size() == 3;
    }));
    CHECK(received[0] == small);
    CHECK(received[1] == big);
    CHECK(received[2].empty());
    CHECK(client.getPendingSendBytes() == 0);

    // And back the other way
    auto reply = bytes({ 4, 5 });
    REQUIRE(pServer->send(reply.data(), reply.size()));
    std::optional<std::vector<std::uint8_t>> answer;
    REQUIRE(waitFor([&]() { return (answer = client.receive()).has_value(); }));
    CHECK(*answer == reply);

    std::vector<std::uint8_t> tooBig(net::TcpConnection::kMaxMessageSize + 1);
    CHECK_FALSE(client.send(tooBig.data(), tooBig.size()));
}

TEST_CASE("TcpConnection notices when the other side leaves")
{
    net::TcpListener listener;
    REQUIRE(listener.open());
    net::TcpConnection client;
    REQUIRE(client.connect(net::Address{ "127.0.0.1", listener.getLocalPort() }));
    std::unique_ptr<net::TcpConnection> pServer;
    REQUIRE(waitFor([&]() { return (pServer = listener.accept()) != nullptr; }));

    // A goodbye sent just before leaving still arrives
    auto goodbye = bytes({ 9 });
    client.send(goodbye.data(), goodbye.size());
    client.disconnect();
    CHECK_FALSE(client.isConnected());

    std::optional<std::vector<std::uint8_t>> last;
    REQUIRE(waitFor([&]() { return (last = pServer->receive()).has_value(); }));
    CHECK(*last == goodbye);
    CHECK(waitFor([&]()
    {
        pServer->receive();
        return !pServer->isConnected();
    }));
}

TEST_CASE("TcpConnection fails cleanly when nobody is listening")
{
    net::TcpListener listener;
    REQUIRE(listener.open());
    std::uint16_t port = listener.getLocalPort();
    listener.close();

    net::TcpConnection client;
    CHECK_FALSE(client.connect(net::Address{ "127.0.0.1", port }));
    CHECK_FALSE(client.isConnected());
    auto data = bytes({ 1 });
    CHECK_FALSE(client.send(data.data(), data.size()));
}

TEST_CASE("LanDiscovery finds a host by direct query and ignores stray datagrams")
{
    // Find a free port for the host, then host on it
    std::uint16_t port = 0;
    {
        net::UdpSocket probe;
        REQUIRE(probe.open());
        port = probe.getLocalPort();
    }

    net::LanDiscovery host;
    REQUIRE(host.host("Sandbox tcp=5000", port));

    net::UdpSocket stray;
    REQUIRE(stray.open());
    auto junk = bytes({ 'h', 'i' });
    stray.sendTo(net::Address{ "127.0.0.1", port }, junk.data(), junk.size());

    net::LanDiscovery searcher;
    REQUIRE(searcher.query(net::Address{ "127.0.0.1", port }));
    REQUIRE(searcher.query(net::Address{ "127.0.0.1", port })); // Duplicate answers collapse into one entry
    REQUIRE(waitFor([&]()
    {
        host.poll();
        searcher.poll();
        return !searcher.getServers().empty();
    }));
    host.poll();
    searcher.poll();

    REQUIRE(searcher.getServers().size() == 1);
    CHECK(searcher.getServers()[0].info == "Sandbox tcp=5000");
    CHECK(searcher.getServers()[0].address.port == port);

    // Nothing answers the stray sender
    std::uint8_t buffer[net::kMaxDatagramSize];
    net::Address from;
    CHECK_FALSE(stray.receive(from, buffer, sizeof(buffer)).has_value());
}

TEST_CASE("Broadcast can be enabled on a UDP socket")
{
    net::UdpSocket socket;
    REQUIRE(socket.open());
    auto data = bytes({ 0 });
    // Only checks the socket accepts it: whether anything hears it depends on the network
    socket.broadcast(net::LanDiscovery::kDefaultPort, data.data(), data.size());
    CHECK(socket.isOpen());
}
#endif
