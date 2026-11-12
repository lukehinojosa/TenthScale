#pragma once

#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace net
{
    // Packs values bit by bit into a byte buffer through a 64-bit scratch register
    class BitWriter
    {
    public:
        // Writes the lowest `bits` bits of value (1 to 32)
        void writeBits(std::uint32_t value, int bits)
        {
            assert(bits >= 1 && bits <= 32);

            std::uint64_t mask = (bits == 32) ? 0xFFFFFFFFull : ((1ull << bits) - 1);
            mScratch |= (static_cast<std::uint64_t>(value) & mask) << mScratchBits;
            mScratchBits += bits;
            mTotalBits += bits;

            while (mScratchBits >= 8)
            {
                mBuffer.push_back(static_cast<std::uint8_t>(mScratch & 0xFF));
                mScratch >>= 8;
                mScratchBits -= 8;
            }
        }

        void writeBool(bool value)
        {
            writeBits(value ? 1u : 0u, 1);
        }

        // Pads the last partial byte with zeros; call before reading data() or size()
        void flush()
        {
            if (mScratchBits > 0)
            {
                mBuffer.push_back(static_cast<std::uint8_t>(mScratch & 0xFF));
                mTotalBits += 8 - mScratchBits;
                mScratch = 0;
                mScratchBits = 0;
            }
        }

        // Byte-aligns first, so raw data always starts on a byte boundary
        void writeBytes(const std::uint8_t* data, std::size_t length)
        {
            flush();
            mBuffer.insert(mBuffer.end(), data, data + length);
            mTotalBits += length * 8;
        }

        const std::uint8_t* data() const { return mBuffer.data(); }
        std::size_t size() const { return mBuffer.size(); }
        std::size_t bitsWritten() const { return mTotalBits; }
        const std::vector<std::uint8_t>& buffer() const { return mBuffer; }

    private:
        std::vector<std::uint8_t> mBuffer;
        std::uint64_t mScratch = 0;
        int mScratchBits = 0;
        std::size_t mTotalBits = 0;
    };

    // Reads values back out of a buffer written by BitWriter
    class BitReader
    {
    public:
        BitReader(const std::uint8_t* data, std::size_t size) : mpData(data), mSize(size) {}

        // Reading past the end returns zeros and sets hasOverflowed()
        std::uint32_t readBits(int bits)
        {
            assert(bits >= 1 && bits <= 32);

            while (mScratchBits < bits)
            {
                if (mByteOffset >= mSize)
                {
                    mOverflowed = true;
                    mScratchBits = bits; // Pretend the missing bits were zero
                    break;
                }
                mScratch |= static_cast<std::uint64_t>(mpData[mByteOffset++]) << mScratchBits;
                mScratchBits += 8;
            }

            std::uint64_t mask = (bits == 32) ? 0xFFFFFFFFull : ((1ull << bits) - 1);
            std::uint32_t result = static_cast<std::uint32_t>(mScratch & mask);
            mScratch >>= bits;
            mScratchBits -= bits;
            mTotalBitsRead += bits;
            return result;
        }

        bool readBool()
        {
            return readBits(1) != 0;
        }

        // Drops leftover bits of the current byte to match BitWriter::writeBytes
        void readBytes(std::uint8_t* out, std::size_t length)
        {
            mScratch = 0;
            mScratchBits = 0;
            for (std::size_t i = 0; i < length; i++)
            {
                if (mByteOffset >= mSize)
                {
                    mOverflowed = true;
                    out[i] = 0;
                    continue;
                }
                out[i] = mpData[mByteOffset++];
            }
            mTotalBitsRead += length * 8;
        }

        std::size_t bitsRead() const { return mTotalBitsRead; }
        bool hasOverflowed() const { return mOverflowed; }

    private:
        const std::uint8_t* mpData;
        std::size_t mSize;
        std::size_t mByteOffset = 0;
        std::uint64_t mScratch = 0;
        int mScratchBits = 0;
        std::size_t mTotalBitsRead = 0;
        bool mOverflowed = false;
    };

    template<typename T>
    concept IsWriter = requires(T& stream, std::uint32_t value, int bits, bool flag)
    {
        { stream.writeBits(value, bits) };
        { stream.writeBool(flag) };
        { stream.flush() };
    };

    template<typename T>
    concept IsReader = requires(T& stream, int bits)
    {
        { stream.readBits(bits) } -> std::convertible_to<std::uint32_t>;
        { stream.readBool() } -> std::convertible_to<bool>;
    };

    template<typename T>
    concept IsStream = IsWriter<T> || IsReader<T>;
}
