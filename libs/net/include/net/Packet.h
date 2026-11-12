#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "BitStream.h"

namespace net
{
    using MessageType = std::uint8_t;

    // Every packet starts with [1 byte type][2 byte payload length, big-endian]
    struct PacketHeader
    {
        static constexpr std::size_t SIZE = 3;

        MessageType type = 0;
        std::uint16_t payloadLength = 0;
    };

    inline void writePacket(std::vector<std::uint8_t>& out, MessageType type, const std::uint8_t* payload, std::size_t payloadLength)
    {
        std::uint16_t length = static_cast<std::uint16_t>(payloadLength);
        out.push_back(type);
        out.push_back(static_cast<std::uint8_t>((length >> 8) & 0xFF));
        out.push_back(static_cast<std::uint8_t>(length & 0xFF));
        if (payload != nullptr && payloadLength > 0)
        {
            out.insert(out.end(), payload, payload + payloadLength);
        }
    }

    // Returns false if the buffer is too short to hold the header
    inline bool readPacketHeader(const std::uint8_t* data, std::size_t size, PacketHeader& header)
    {
        if (size < PacketHeader::SIZE)
        {
            return false;
        }
        header.type = data[0];
        header.payloadLength = static_cast<std::uint16_t>((data[1] << 8) | data[2]);
        return true;
    }

    // Serializes a message with fn(BitWriter&) and frames it as one packet
    template<typename SerializeFn>
    std::vector<std::uint8_t> makePacket(MessageType type, SerializeFn fn)
    {
        BitWriter writer;
        fn(writer);
        writer.flush();

        std::vector<std::uint8_t> packet;
        writePacket(packet, type, writer.data(), writer.size());
        return packet;
    }

    // Routes each incoming packet to the handler registered for its type
    class PacketDispatcher
    {
    public:
        using Handler = std::function<void(BitReader&)>;

        void setHandler(MessageType type, Handler handler)
        {
            mHandlers[type] = std::move(handler);
        }

        // Handles every packet in the buffer; returns false on a malformed or unknown packet
        bool dispatch(const std::uint8_t* data, std::size_t size) const
        {
            std::size_t offset = 0;
            while (offset < size)
            {
                PacketHeader header;
                if (!readPacketHeader(data + offset, size - offset, header))
                {
                    return false;
                }
                offset += PacketHeader::SIZE;
                if (offset + header.payloadLength > size)
                {
                    return false;
                }

                auto it = mHandlers.find(header.type);
                if (it == mHandlers.end())
                {
                    return false;
                }

                BitReader reader(data + offset, header.payloadLength);
                it->second(reader);
                offset += header.payloadLength;
            }
            return true;
        }

    private:
        std::unordered_map<MessageType, Handler> mHandlers;
    };
}
