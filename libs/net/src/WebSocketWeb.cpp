#ifdef __EMSCRIPTEN__

#include "net/WebSocket.h"

#include <deque>
#include <unordered_map>

#include <emscripten/emscripten.h>

namespace net
{
    struct WebSocket::Impl
    {
        int id = 0;
        bool open = false;
        std::deque<SocketEvent> events;
    };
}

namespace
{
    // JS refers to sockets by id, so a callback for a destroyed socket finds nothing instead of freed memory
    std::unordered_map<int, net::WebSocket::Impl*> gSockets;
    int gNextID = 1;

    net::WebSocket::Impl* findSocket(int id)
    {
        auto it = gSockets.find(id);
        return it == gSockets.end() ? nullptr : it->second;
    }
}

EM_JS(void, jsSocketOpen, (int id, const char* url), {
    if (!Module.templateSockets)
    {
        Module.templateSockets = {};
    }
    let ws;
    try
    {
        ws = new WebSocket(UTF8ToString(url));
    }
    catch (e)
    {
        Module.ccall('net_ws_on_error', null, ['number', 'string'], [id, String(e)]);
        return;
    }
    ws.binaryType = 'arraybuffer';
    Module.templateSockets[id] = ws;
    ws.onopen = () => Module.ccall('net_ws_on_open', null, ['number'], [id]);
    ws.onclose = () => Module.ccall('net_ws_on_close', null, ['number'], [id]);
    ws.onerror = () => Module.ccall('net_ws_on_error', null, ['number', 'string'], [id, 'WebSocket error']);
    ws.onmessage = (e) => {
        if (e.data instanceof ArrayBuffer)
        {
            const bytes = new Uint8Array(e.data);
            Module.ccall('net_ws_on_binary', null, ['number', 'array', 'number'], [id, bytes, bytes.length]);
        }
        else
        {
            Module.ccall('net_ws_on_text', null, ['number', 'string'], [id, e.data]);
        }
    };
});

EM_JS(int, jsSocketSendText, (int id, const char* text), {
    const ws = Module.templateSockets && Module.templateSockets[id];
    if (!ws || ws.readyState !== 1)
    {
        return 0;
    }
    ws.send(UTF8ToString(text));
    return 1;
});

EM_JS(int, jsSocketSendBinary, (int id, const unsigned char* data, int size), {
    const ws = Module.templateSockets && Module.templateSockets[id];
    if (!ws || ws.readyState !== 1)
    {
        return 0;
    }
    ws.send(HEAPU8.slice(data, data + size));
    return 1;
});

EM_JS(void, jsSocketClose, (int id), {
    const ws = Module.templateSockets && Module.templateSockets[id];
    if (!ws)
    {
        return;
    }
    ws.onopen = null;
    ws.onclose = null;
    ws.onerror = null;
    ws.onmessage = null;
    ws.close();
    delete Module.templateSockets[id];
});

extern "C"
{
    EMSCRIPTEN_KEEPALIVE void net_ws_on_open(int id)
    {
        if (auto* pImpl = findSocket(id))
        {
            pImpl->open = true;
            pImpl->events.push_back(net::SocketEvent{ net::SocketEventType::OPENED, {}, {} });
        }
    }

    EMSCRIPTEN_KEEPALIVE void net_ws_on_close(int id)
    {
        if (auto* pImpl = findSocket(id))
        {
            pImpl->open = false;
            pImpl->events.push_back(net::SocketEvent{ net::SocketEventType::CLOSED, {}, {} });
        }
    }

    EMSCRIPTEN_KEEPALIVE void net_ws_on_error(int id, const char* reason)
    {
        if (auto* pImpl = findSocket(id))
        {
            pImpl->events.push_back(net::SocketEvent{ net::SocketEventType::FAILED, reason, {} });
        }
    }

    EMSCRIPTEN_KEEPALIVE void net_ws_on_text(int id, const char* text)
    {
        if (auto* pImpl = findSocket(id))
        {
            pImpl->events.push_back(net::SocketEvent{ net::SocketEventType::TEXT, text, {} });
        }
    }

    EMSCRIPTEN_KEEPALIVE void net_ws_on_binary(int id, const std::uint8_t* data, int size)
    {
        if (auto* pImpl = findSocket(id))
        {
            net::SocketEvent event;
            event.type = net::SocketEventType::BINARY;
            event.binary.assign(data, data + size);
            pImpl->events.push_back(std::move(event));
        }
    }
}

namespace net
{
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
        mpImpl->id = gNextID++;
        gSockets[mpImpl->id] = mpImpl.get();
        jsSocketOpen(mpImpl->id, url.c_str());
    }

    void WebSocket::close()
    {
        if (mpImpl->id != 0)
        {
            jsSocketClose(mpImpl->id);
            gSockets.erase(mpImpl->id);
            mpImpl->id = 0;
        }
        mpImpl->open = false;
        mpImpl->events.clear();
    }

    bool WebSocket::isOpen() const
    {
        return mpImpl->open;
    }

    bool WebSocket::sendText(const std::string& text)
    {
        return mpImpl->id != 0 && jsSocketSendText(mpImpl->id, text.c_str()) != 0;
    }

    bool WebSocket::sendBinary(const std::uint8_t* data, std::size_t size)
    {
        return mpImpl->id != 0 && jsSocketSendBinary(mpImpl->id, data, static_cast<int>(size)) != 0;
    }

    bool WebSocket::pollEvent(SocketEvent& outEvent)
    {
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
