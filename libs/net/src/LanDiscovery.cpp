#include "net/LanDiscovery.h"

#include <algorithm>
#include <cstring>

namespace net
{
    namespace
    {
        // Every datagram starts with these bytes, so stray traffic on the port is ignored
        constexpr std::uint8_t MAGIC[4] = { 'L', 'A', 'N', 'D' };
        constexpr std::uint8_t SEARCH = 1;
        constexpr std::uint8_t ANSWER = 2;
        constexpr std::size_t HEADER_SIZE = sizeof(MAGIC) + 1;

        bool hasHeader(const std::uint8_t* data, std::size_t size, std::uint8_t kind)
        {
            return size >= HEADER_SIZE && std::memcmp(data, MAGIC, sizeof(MAGIC)) == 0 && data[sizeof(MAGIC)] == kind;
        }

        std::vector<std::uint8_t> makeDatagram(std::uint8_t kind, const std::string& body)
        {
            std::vector<std::uint8_t> datagram(MAGIC, MAGIC + sizeof(MAGIC));
            datagram.push_back(kind);
            datagram.insert(datagram.end(), body.begin(), body.end());
            return datagram;
        }
    }

    bool LanDiscovery::host(const std::string& info, std::uint16_t port)
    {
        if (info.size() > kMaxDatagramSize - HEADER_SIZE)
        {
            return false;
        }
        mInfo = info;
        return mHostSocket.open(port);
    }

    void LanDiscovery::stopHosting()
    {
        mHostSocket.close();
    }

    bool LanDiscovery::ensureSearchSocket()
    {
        return mSearchSocket.isOpen() || mSearchSocket.open();
    }

    bool LanDiscovery::search(std::uint16_t port)
    {
        if (!ensureSearchSocket())
        {
            return false;
        }
        auto datagram = makeDatagram(SEARCH, "");
        return mSearchSocket.broadcast(port, datagram.data(), datagram.size());
    }

    bool LanDiscovery::query(const Address& to)
    {
        if (!ensureSearchSocket())
        {
            return false;
        }
        auto datagram = makeDatagram(SEARCH, "");
        return mSearchSocket.sendTo(to, datagram.data(), datagram.size());
    }

    void LanDiscovery::poll()
    {
        std::uint8_t buffer[kMaxDatagramSize];
        Address from;

        while (auto size = mHostSocket.receive(from, buffer, sizeof(buffer)))
        {
            if (hasHeader(buffer, *size, SEARCH))
            {
                auto answer = makeDatagram(ANSWER, mInfo);
                mHostSocket.sendTo(from, answer.data(), answer.size());
            }
        }

        while (auto size = mSearchSocket.receive(from, buffer, sizeof(buffer)))
        {
            if (!hasHeader(buffer, *size, ANSWER))
            {
                continue;
            }

            std::string info((const char*)buffer + HEADER_SIZE, *size - HEADER_SIZE);
            auto existing = std::find_if(mServers.begin(), mServers.end(), [&](const LanServer& server)
            {
                return server.address.host == from.host && server.address.port == from.port;
            });
            if (existing != mServers.end())
            {
                existing->info = info;
            }
            else
            {
                mServers.push_back(LanServer{ from, info });
            }
        }
    }
}
