// Lobby and relay server for net::RelayClient: pairs a host with one guest, then forwards their messages
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <asio.hpp>
#include <nlohmann/json.hpp>
#include <rtc/rtc.hpp>

namespace
{
    using json = nlohmann::json;
    using SocketPtr = std::shared_ptr<rtc::WebSocket>;

    struct Lobby
    {
        SocketPtr host;
        std::string hostName;
        int playerCount = 1;
    };

    std::mutex gMutex;
    std::unordered_map<std::string, Lobby> gLobbies;
    std::unordered_map<rtc::WebSocket*, SocketPtr> gPartners; // Each paired socket maps to the other
    std::unordered_set<SocketPtr> gPending; // Keeps sockets alive before they host or join

    std::string makeCode()
    {
        static const char CHARS[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789"; // No 0/O or 1/I to misread
        static std::mt19937 rng{ std::random_device{}() };
        static std::uniform_int_distribution<int> dist(0, (int)sizeof(CHARS) - 2);

        std::string code(6, ' ');
        for (char& c : code)
        {
            c = CHARS[dist(rng)];
        }
        return code;
    }

    void handleHost(const SocketPtr& socket, const json& message)
    {
        std::string code;
        {
            std::lock_guard<std::mutex> lock(gMutex);
            do
            {
                code = makeCode();
            } while (gLobbies.count(code) > 0);

            gLobbies[code] = Lobby{ socket, message.value("username", "Unknown"), 1 };
            gPending.erase(socket);
        }
        socket->send(json{ { "type", "code" }, { "code", code } }.dump());
        std::cout << "[relay] lobby " << code << " opened\n";
    }

    void handleJoin(const SocketPtr& socket, const json& message)
    {
        const std::string code = message.value("code", "");
        SocketPtr host;
        {
            std::lock_guard<std::mutex> lock(gMutex);
            auto it = gLobbies.find(code);
            if (it == gLobbies.end() || it->second.playerCount >= 2)
            {
                std::string reason = (it == gLobbies.end()) ? "bad code" : "lobby full";
                socket->send(json{ { "type", "error" }, { "msg", reason } }.dump());
                socket->close();
                return;
            }

            host = it->second.host;
            it->second.playerCount = 2;
            gPartners[host.get()] = socket;
            gPartners[socket.get()] = host;
            gPending.erase(socket);
        }
        host->send(json{ { "type", "guest_ready" } }.dump());
        socket->send(json{ { "type", "ready" } }.dump());
        std::cout << "[relay] lobby " << code << " paired\n";
    }

    void handleList(const SocketPtr& socket)
    {
        json lobbies = json::array();
        {
            std::lock_guard<std::mutex> lock(gMutex);
            for (const auto& [code, lobby] : gLobbies)
            {
                lobbies.push_back({ { "code", code }, { "host", lobby.hostName }, { "players", lobby.playerCount } });
            }
        }
        socket->send(json{ { "type", "list" }, { "lobbies", lobbies } }.dump());
    }

    void handleClient(const SocketPtr& socket)
    {
        {
            std::lock_guard<std::mutex> lock(gMutex);
            gPending.insert(socket);
        }

        std::weak_ptr<rtc::WebSocket> weakSocket = socket;

        socket->onMessage([weakSocket](rtc::message_variant data)
        {
            SocketPtr socket = weakSocket.lock();
            if (!socket)
            {
                return;
            }

            // Paired sockets: forward everything untouched
            {
                std::lock_guard<std::mutex> lock(gMutex);
                auto it = gPartners.find(socket.get());
                if (it != gPartners.end())
                {
                    it->second->send(std::move(data));
                    return;
                }
            }

            if (!std::holds_alternative<std::string>(data))
            {
                return;
            }

            const std::string& raw = std::get<std::string>(data);
            json message = json::parse(raw, nullptr, false);
            if (message.is_discarded())
            {
                return;
            }

            const std::string type = message.value("type", "");
            if (type == "host")
            {
                handleHost(socket, message);
            }
            else if (type == "join")
            {
                handleJoin(socket, message);
            }
            else if (type == "list")
            {
                handleList(socket);
            }
            else if (type == "keepalive")
            {
                socket->send(raw);
            }
        });

        socket->onClosed([weakSocket]()
        {
            SocketPtr socket = weakSocket.lock();
            if (!socket)
            {
                return;
            }

            std::lock_guard<std::mutex> lock(gMutex);
            gPending.erase(socket);

            for (auto it = gLobbies.begin(); it != gLobbies.end();)
            {
                if (it->second.host == socket)
                {
                    it = gLobbies.erase(it);
                }
                else
                {
                    ++it;
                }
            }

            auto partner = gPartners.find(socket.get());
            if (partner != gPartners.end())
            {
                SocketPtr other = partner->second;
                gPartners.erase(other.get());
                gPartners.erase(partner);
                other->close(); // The other player can't keep playing alone
            }
        });
    }
}

int main(int argc, char** argv)
{
    std::cout << std::unitbuf; // Flush every line so logs show up live, even when piped
    rtc::WebSocketServer::Configuration config;
    config.port = 9000;
    if (argc > 1)
    {
        config.port = (uint16_t)std::atoi(argv[1]);
    }

    // Set TLS_CERT and TLS_KEY to serve wss://, which pages served over https require
    const char* certFile = std::getenv("TLS_CERT");
    const char* keyFile = std::getenv("TLS_KEY");
    if (certFile != nullptr && keyFile != nullptr)
    {
        config.enableTls = true;
        config.certificatePemFile = certFile;
        config.keyPemFile = keyFile;
    }

    auto server = std::make_shared<rtc::WebSocketServer>(config);
    server->onClient([](SocketPtr socket)
    {
        handleClient(socket);
    });

    std::cout << "[relay] listening on " << (config.enableTls ? "wss" : "ws") << "://0.0.0.0:" << config.port << "\n";

    // Block the main thread until Ctrl+C
    asio::io_context context;
    asio::signal_set signals(context, SIGINT, SIGTERM);
    signals.async_wait([&](const std::error_code& error, int)
    {
        if (!error)
        {
            std::cout << "[relay] shutting down\n";
            server->stop();
        }
    });
    context.run();
    return 0;
}
