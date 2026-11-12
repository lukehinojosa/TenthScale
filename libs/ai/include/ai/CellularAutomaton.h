#pragma once

#include <cstdint>
#include <vector>

namespace ai
{
    // Life-like cellular automaton on a wrapping grid. Rules are bitmasks over live-neighbor counts:
    // bit n of birthMask means a dead cell with n neighbors comes alive. Conway's Life is B3/S23.
    class CellularAutomaton
    {
    public:
        CellularAutomaton(int width, int height, std::uint16_t birthMask = 1u << 3, std::uint16_t surviveMask = (1u << 2) | (1u << 3))
            : mWidth(width), mHeight(height), mBirthMask(birthMask), mSurviveMask(surviveMask), mCells((std::size_t)width * height, 0),
              mNext(mCells.size(), 0)
        {
        }

        int getWidth() const { return mWidth; }
        int getHeight() const { return mHeight; }

        bool isAlive(int x, int y) const { return mCells[wrapIndex(x, y)] != 0; }
        void setAlive(int x, int y, bool alive) { mCells[wrapIndex(x, y)] = alive ? 1 : 0; }

        int countLiveNeighbors(int x, int y) const
        {
            int count = 0;
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    if ((dx != 0 || dy != 0) && isAlive(x + dx, y + dy))
                    {
                        count++;
                    }
                }
            }
            return count;
        }

        // Every cell reads the old generation, so the update is order-independent
        void step()
        {
            for (int y = 0; y < mHeight; y++)
            {
                for (int x = 0; x < mWidth; x++)
                {
                    int neighbors = countLiveNeighbors(x, y);
                    std::uint16_t mask = isAlive(x, y) ? mSurviveMask : mBirthMask;
                    mNext[(std::size_t)y * mWidth + x] = ((mask >> neighbors) & 1u) ? 1 : 0;
                }
            }
            mCells.swap(mNext);
        }

        int countAlive() const
        {
            int count = 0;
            for (std::uint8_t cell : mCells)
            {
                count += cell;
            }
            return count;
        }

    private:
        std::size_t wrapIndex(int x, int y) const
        {
            x = ((x % mWidth) + mWidth) % mWidth;
            y = ((y % mHeight) + mHeight) % mHeight;
            return (std::size_t)y * mWidth + x;
        }

        int mWidth;
        int mHeight;
        std::uint16_t mBirthMask;
        std::uint16_t mSurviveMask;
        std::vector<std::uint8_t> mCells;
        std::vector<std::uint8_t> mNext;
    };
}
