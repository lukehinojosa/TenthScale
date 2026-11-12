#include <doctest/doctest.h>

#include <net/Packet.h>
#include <net/Serialize.h>

namespace
{
    struct PlayerState
    {
        std::uint16_t x = 0;
        std::uint8_t health = 0;
        std::int32_t velocity = 0;
        bool alive = false;
        float heading = 0.0f;
        std::string name;
    };

    template<net::IsStream Stream>
    bool serializePlayer(Stream& stream, PlayerState& player)
    {
        bool ok = net::serializeInt(stream, player.x, 0, 1023);
        ok = ok && net::serializeInt(stream, player.health, 0, 100);
        ok = ok && net::serializeInt(stream, player.velocity, -500, 500);
        ok = ok && net::serializeBool(stream, player.alive);
        ok = ok && net::serializeFloat(stream, player.heading, 0.0f, 360.0f, 12);
        ok = ok && net::serializeString(stream, player.name);
        return ok;
    }
}

TEST_CASE("bitsRequired covers the whole range")
{
    CHECK(net::bitsRequired(0, 1) == 1);
    CHECK(net::bitsRequired(0, 7) == 3);
    CHECK(net::bitsRequired(0, 8) == 4);
    CHECK(net::bitsRequired(0, 1023) == 10);
    CHECK(net::bitsRequired(5, 5) == 1);
}

TEST_CASE("Varint and zigzag round trip")
{
    std::uint8_t buffer[10];
    std::uint32_t decoded = 0;
    CHECK(net::encodeVarint(300u, buffer) == 2);
    CHECK(buffer[0] == 0xAC);
    CHECK(buffer[1] == 0x02);
    CHECK(net::decodeVarint(buffer, decoded) == 2);
    CHECK(decoded == 300u);

    CHECK(net::zigzagEncode(0) == 0u);
    CHECK(net::zigzagEncode(-1) == 1u);
    CHECK(net::zigzagEncode(1) == 2u);
    CHECK(net::zigzagDecode(net::zigzagEncode(-123456)) == -123456);
}

TEST_CASE("BitWriter and BitReader agree bit for bit")
{
    net::BitWriter writer;
    writer.writeBits(5, 3);
    writer.writeBool(true);
    writer.writeBits(0xABCDEF, 24);
    writer.writeBits(0xFFFFFFFFu, 32);
    writer.flush();
    CHECK(writer.size() == 8);

    net::BitReader reader(writer.data(), writer.size());
    CHECK(reader.readBits(3) == 5u);
    CHECK(reader.readBool());
    CHECK(reader.readBits(24) == 0xABCDEFu);
    CHECK(reader.readBits(32) == 0xFFFFFFFFu);
    CHECK_FALSE(reader.hasOverflowed());

    reader.readBits(8);
    CHECK(reader.hasOverflowed());
}

TEST_CASE("One serialize function writes and reads a struct")
{
    PlayerState original{ 700, 85, -42, true, 123.4f, "Luke" };

    net::BitWriter writer;
    REQUIRE(serializePlayer(writer, original));
    writer.flush();

    PlayerState copy;
    net::BitReader reader(writer.data(), writer.size());
    REQUIRE(serializePlayer(reader, copy));
    CHECK(copy.x == original.x);
    CHECK(copy.health == original.health);
    CHECK(copy.velocity == original.velocity);
    CHECK(copy.alive == original.alive);
    CHECK(copy.heading == doctest::Approx(original.heading).epsilon(0.001));
    CHECK(copy.name == original.name);
}

TEST_CASE("Out of range values are rejected")
{
    PlayerState bad{ 2000, 0, 0, false, 0.0f, "" };
    net::BitWriter writer;
    CHECK_FALSE(serializePlayer(writer, bad));
}

TEST_CASE("PacketDispatcher routes each packet by type")
{
    std::vector<std::uint8_t> stream = net::makePacket(7, [](net::BitWriter& writer)
    {
        writer.writeBits(42, 8);
    });
    std::vector<std::uint8_t> second = net::makePacket(9, [](net::BitWriter&) {});
    stream.insert(stream.end(), second.begin(), second.end());

    int sevens = 0;
    int nines = 0;
    std::uint32_t payload = 0;
    net::PacketDispatcher dispatcher;
    dispatcher.setHandler(7, [&](net::BitReader& reader)
    {
        sevens++;
        payload = reader.readBits(8);
    });
    dispatcher.setHandler(9, [&](net::BitReader&) { nines++; });

    CHECK(dispatcher.dispatch(stream.data(), stream.size()));
    CHECK(sevens == 1);
    CHECK(nines == 1);
    CHECK(payload == 42u);

    // Truncated payloads or headers are refused instead of read past the end
    CHECK_FALSE(dispatcher.dispatch(stream.data(), 3));
    CHECK_FALSE(dispatcher.dispatch(stream.data(), 5));
}

#ifdef TEMPLATE_HAS_RELAY
#include <net/RelayClient.h>

#include <chrono>
#include <cstdlib>
#include <thread>

// Needs a running relay_server: set RELAY_TEST_URL (e.g. ws://localhost:9000) to enable
TEST_CASE("RelayClient host and guest exchange a message through relay_server")
{
    const char* url = std::getenv("RELAY_TEST_URL");
    if (url == nullptr)
    {
        MESSAGE("RELAY_TEST_URL not set, skipping");
        return;
    }

    net::RelayClient host;
    net::RelayClient guest;
    std::string code;
    bool hostConnected = false;
    std::vector<std::uint8_t> received;

    host.onLobbyCode = [&](const std::string& lobbyCode)
    {
        code = lobbyCode;
        guest.join(url, code);
    };
    host.onPeerConnected = [&]()
    {
        hostConnected = true;
    };
    guest.onPeerConnected = [&]()
    {
        const std::uint8_t hello[] = { 1, 2, 3 };
        guest.send(hello, sizeof(hello));
    };
    host.onMessage = [&](std::span<const std::uint8_t> data)
    {
        received.assign(data.begin(), data.end());
    };

    host.host(url, "TestHost");
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (received.empty() && std::chrono::steady_clock::now() < deadline)
    {
        host.poll();
        guest.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    CHECK(code.size() == 6);
    CHECK(hostConnected);
    REQUIRE(received.size() == 3);
    CHECK(received[2] == 3);
}
#endif
