#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>

#include "BitStream.h"
#include "Varint.h"

// One serialize function per type handles both directions: write it once and the read can't drift
namespace net
{
    // Stores value - min in just enough bits for the range; fails on out-of-range values
    template<IsStream Stream, typename T>
        requires std::is_integral_v<T> || std::is_enum_v<T>
    bool serializeInt(Stream& stream, T& value, std::int64_t min, std::int64_t max)
    {
        const int bits = bitsRequired(0, static_cast<std::uint64_t>(max - min));
        if constexpr (IsWriter<Stream>)
        {
            std::int64_t asInt = static_cast<std::int64_t>(value);
            if (asInt < min || asInt > max)
            {
                return false;
            }
            stream.writeBits(static_cast<std::uint32_t>(asInt - min), bits);
            return true;
        }
        else
        {
            std::int64_t asInt = static_cast<std::int64_t>(stream.readBits(bits)) + min;
            if (asInt > max || stream.hasOverflowed())
            {
                return false;
            }
            value = static_cast<T>(asInt);
            return true;
        }
    }

    template<IsStream Stream>
    bool serializeBool(Stream& stream, bool& value)
    {
        if constexpr (IsWriter<Stream>)
        {
            stream.writeBool(value);
            return true;
        }
        else
        {
            value = stream.readBool();
            return !stream.hasOverflowed();
        }
    }

    // Lossy: snaps value to the nearest of evenly spaced steps between min and max, as many as the bits allow
    template<IsStream Stream>
    bool serializeFloat(Stream& stream, float& value, float min, float max, int bits)
    {
        const std::uint32_t steps = (bits == 32) ? 0xFFFFFFFFu : ((1u << bits) - 1);
        if constexpr (IsWriter<Stream>)
        {
            if (value < min || value > max)
            {
                return false;
            }
            float normalized = (value - min) / (max - min);
            stream.writeBits(static_cast<std::uint32_t>(std::lround(normalized * steps)), bits);
            return true;
        }
        else
        {
            std::uint32_t quantized = stream.readBits(bits);
            value = min + (max - min) * (static_cast<float>(quantized) / static_cast<float>(steps));
            return !stream.hasOverflowed();
        }
    }

    // Full 32-bit float, bit for bit
    template<IsStream Stream>
    bool serializeRawFloat(Stream& stream, float& value)
    {
        std::uint32_t bitsValue = 0;
        if constexpr (IsWriter<Stream>)
        {
            std::memcpy(&bitsValue, &value, sizeof(float));
            stream.writeBits(bitsValue, 32);
            return true;
        }
        else
        {
            bitsValue = stream.readBits(32);
            std::memcpy(&value, &bitsValue, sizeof(float));
            return !stream.hasOverflowed();
        }
    }

    // Varint length followed by raw bytes, byte-aligned
    template<IsStream Stream>
    bool serializeString(Stream& stream, std::string& str, std::uint32_t maxLength = 255)
    {
        if constexpr (IsWriter<Stream>)
        {
            if (str.size() > maxLength)
            {
                return false;
            }
            std::uint8_t lengthBytes[10];
            std::size_t lengthSize = encodeVarint(static_cast<std::uint32_t>(str.size()), lengthBytes);
            stream.writeBytes(lengthBytes, lengthSize);
            stream.writeBytes(reinterpret_cast<const std::uint8_t*>(str.data()), str.size());
            return true;
        }
        else
        {
            std::uint32_t length = 0;
            decodeVarintWith([&]() -> std::uint8_t
            {
                std::uint8_t byte = 0;
                stream.readBytes(&byte, 1);
                return byte;
            }, length);
            if (length > maxLength || stream.hasOverflowed())
            {
                return false;
            }
            str.resize(length);
            stream.readBytes(reinterpret_cast<std::uint8_t*>(str.data()), length);
            return !stream.hasOverflowed();
        }
    }
}
