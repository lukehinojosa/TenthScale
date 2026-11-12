#pragma once

#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace net
{
    // Fewest bits that can hold every value from min to max
    constexpr int bitsRequired(std::uint64_t min, std::uint64_t max)
    {
        if (min >= max)
        {
            return 1;
        }
        return static_cast<int>(std::bit_width(max - min));
    }

    // 7 data bits per byte, high bit set while more bytes follow; writeByte(uint8_t) receives each byte
    template<std::unsigned_integral T, typename WriteByteFn>
    std::size_t encodeVarintWith(T value, WriteByteFn writeByte)
    {
        std::size_t count = 0;
        do
        {
            std::uint8_t byte = static_cast<std::uint8_t>(value & 0x7F);
            value = static_cast<T>(value >> 7);
            if (value != 0)
            {
                byte |= 0x80;
            }
            writeByte(byte);
            count++;
        } while (value != 0);
        return count;
    }

    template<std::unsigned_integral T>
    std::size_t encodeVarint(T value, std::uint8_t* buffer)
    {
        std::size_t i = 0;
        return encodeVarintWith(value, [&](std::uint8_t byte)
        {
            buffer[i++] = byte;
        });
    }

    // readByte() returns the next byte; returns how many were consumed
    template<std::unsigned_integral T, typename ReadByteFn>
    std::size_t decodeVarintWith(ReadByteFn readByte, T& outValue)
    {
        T result = 0;
        std::size_t count = 0;
        int shift = 0;
        std::uint8_t byte = 0;
        do
        {
            byte = readByte();
            if (shift < static_cast<int>(sizeof(T) * 8))
            {
                result |= static_cast<T>(static_cast<T>(byte & 0x7F) << shift);
            }
            shift += 7;
            count++;
        } while ((byte & 0x80) != 0 && count < 10);
        outValue = result;
        return count;
    }

    template<std::unsigned_integral T>
    std::size_t decodeVarint(const std::uint8_t* buffer, T& outValue)
    {
        std::size_t i = 0;
        return decodeVarintWith([&]() -> std::uint8_t
        {
            return buffer[i++];
        }, outValue);
    }

    // Maps small negative and positive numbers to small unsigned ones: 0, -1, 1, -2 to 0, 1, 2, 3
    template<std::signed_integral T>
    std::make_unsigned_t<T> zigzagEncode(T value)
    {
        using U = std::make_unsigned_t<T>;
        constexpr int shift = static_cast<int>(sizeof(T) * 8 - 1);
        return static_cast<U>((static_cast<U>(value) << 1) ^ static_cast<U>(value >> shift));
    }

    template<std::unsigned_integral U>
    std::make_signed_t<U> zigzagDecode(U value)
    {
        return static_cast<std::make_signed_t<U>>((value >> 1) ^ (~(value & 1) + 1));
    }
}
