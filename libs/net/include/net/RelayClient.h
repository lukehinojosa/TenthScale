#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "WebSocket.h"

namespace net
{
    struct LobbyInfo
    {
        std::string code;
        std::string hostName;
        int playerCount = 1;
    };

    // Two-player lobby client for server/relay: the host gets a code and the guest joins with it. Call poll() once
    // per frame; callbacks run inside it.
    class RelayClient
    {
    public:
        std::function<void(const std::string& code)> onLobbyCode; // Host only
        std::function<void()> onPeerConnected;
        std::function<void(std::span<const std::uint8_t> data)> onMessage;
        std::function<void(const std::string& reason)> onError;
        std::function<void()> onDisconnected;
        std::function<void(const std::vector<LobbyInfo>& lobbies)> onLobbyList;

        void host(const std::string& serverUrl, const std::string& playerName);
        void join(const std::string& serverUrl, const std::string& code);

        // Asks the server for open lobbies on a separate short-lived connection
        void requestLobbyList(const std::string& serverUrl);

        // Only reaches the peer once onPeerConnected has fired
        bool send(const std::uint8_t* data, std::size_t size);
        bool send(const std::vector<std::uint8_t>& data) { return send(data.data(), data.size()); }

        // Hosts waiting for a guest should call this every few seconds so proxies keep the line open
        void sendKeepalive();

        void close();
        void poll();

        bool isPeerConnected() const { return mPeerConnected; }

    private:
        void handleControlMessage(const std::string& text);
        void pollLobbyList();

        WebSocket mSocket;
        WebSocket mListSocket;
        std::string mPendingHello; // Sent as soon as the connection opens
        bool mPeerConnected = false;
    };
}
