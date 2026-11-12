#include "net/RelayClient.h"

#include <nlohmann/json.hpp>

namespace net
{
    using json = nlohmann::json;

    void RelayClient::host(const std::string& serverUrl, const std::string& playerName)
    {
        close();
        mPendingHello = json{ { "type", "host" }, { "username", playerName } }.dump();
        mSocket.open(serverUrl);
    }

    void RelayClient::join(const std::string& serverUrl, const std::string& code)
    {
        close();
        mPendingHello = json{ { "type", "join" }, { "code", code } }.dump();
        mSocket.open(serverUrl);
    }

    void RelayClient::requestLobbyList(const std::string& serverUrl)
    {
        mListSocket.open(serverUrl);
    }

    bool RelayClient::send(const std::uint8_t* data, std::size_t size)
    {
        if (!mPeerConnected)
        {
            return false;
        }
        return mSocket.sendBinary(data, size);
    }

    void RelayClient::sendKeepalive()
    {
        mSocket.sendText(R"({"type":"keepalive"})");
    }

    void RelayClient::close()
    {
        mSocket.close();
        mPendingHello.clear();
        mPeerConnected = false;
    }

    void RelayClient::poll()
    {
        SocketEvent event;
        while (mSocket.pollEvent(event))
        {
            switch (event.type)
            {
            case SocketEventType::OPENED:
                mSocket.sendText(mPendingHello);
                break;
            case SocketEventType::TEXT:
                handleControlMessage(event.text);
                break;
            case SocketEventType::BINARY:
                if (onMessage)
                {
                    onMessage(std::span<const std::uint8_t>(event.binary));
                }
                break;
            case SocketEventType::FAILED:
                if (onError)
                {
                    onError(event.text);
                }
                break;
            case SocketEventType::CLOSED:
                mPeerConnected = false;
                if (onDisconnected)
                {
                    onDisconnected();
                }
                break;
            }
        }

        pollLobbyList();
    }

    void RelayClient::handleControlMessage(const std::string& text)
    {
        json message = json::parse(text, nullptr, false);
        if (message.is_discarded())
        {
            return;
        }

        const std::string type = message.value("type", "");
        if (type == "code")
        {
            if (onLobbyCode)
            {
                onLobbyCode(message.value("code", ""));
            }
        }
        else if (type == "guest_ready" || type == "ready")
        {
            mPeerConnected = true;
            if (onPeerConnected)
            {
                onPeerConnected();
            }
        }
        else if (type == "error")
        {
            if (onError)
            {
                onError(message.value("msg", "unknown server error"));
            }
        }
    }

    void RelayClient::pollLobbyList()
    {
        SocketEvent event;
        while (mListSocket.pollEvent(event))
        {
            if (event.type == SocketEventType::OPENED)
            {
                mListSocket.sendText(R"({"type":"list"})");
            }
            else if (event.type == SocketEventType::TEXT)
            {
                json message = json::parse(event.text, nullptr, false);
                if (message.is_discarded() || message.value("type", "") != "list")
                {
                    continue;
                }

                std::vector<LobbyInfo> lobbies;
                for (const auto& lobby : message["lobbies"])
                {
                    lobbies.push_back({ lobby.value("code", ""), lobby.value("host", ""), lobby.value("players", 1) });
                }
                if (onLobbyList)
                {
                    onLobbyList(lobbies);
                }
                mListSocket.close();
                return;
            }
            else if (event.type == SocketEventType::FAILED && onError)
            {
                onError(event.text);
            }
        }
    }
}
