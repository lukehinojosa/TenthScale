#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "UdpSocket.h"

namespace net
{
    struct LanServer
    {
        Address address; // Where the answer came from
        std::string info; // Whatever the host advertised, such as a name and a TCP port
    };

    // Finds games on the local network: hosts answer broadcast searches with a short description (desktop only)
    class LanDiscovery
    {
    public:
        static constexpr std::uint16_t kDefaultPort = 47777;

        // Answers searches on `port` with `info`; false if the port is taken
        bool host(const std::string& info, std::uint16_t port = kDefaultPort);
        void stopHosting();
        bool isHosting() const { return mHostSocket.isOpen(); }

        // Broadcasts one search; answers arrive through poll()
        bool search(std::uint16_t port = kDefaultPort);

        // Asks one known address directly, for networks that block broadcast
        bool query(const Address& to);

        // Answers searches while hosting and collects answers after searching. Call once per frame.
        void poll();

        const std::vector<LanServer>& getServers() const { return mServers; }
        void clearServers() { mServers.clear(); }

    private:
        bool ensureSearchSocket();

        UdpSocket mHostSocket;
        UdpSocket mSearchSocket;
        std::string mInfo;
        std::vector<LanServer> mServers;
    };
}
