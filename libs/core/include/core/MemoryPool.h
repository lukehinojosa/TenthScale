#pragma once

#include <cstddef>
#include <vector>

#include "Trackable.h"

namespace core
{
    using Byte = unsigned char;

    // A fixed number of equal-sized slots carved out of one block, handed out and returned in O(1)
    class MemoryPool : public Trackable
    {
    public:
        // objectSize is rounded up to a power of two so every slot stays aligned
        MemoryPool(unsigned int maxNumObjects, unsigned int objectSize);
        ~MemoryPool();

        MemoryPool(const MemoryPool&) = delete;
        MemoryPool& operator=(const MemoryPool&) = delete;

        // Returns nullptr when every slot is in use
        Byte* allocateObject();
        void freeObject(Byte* ptr);

        // Marks every slot free again without releasing the block
        void reset();

        bool contains(const Byte* ptr) const;

        unsigned int getMaxObjectSize() const { return mObjectSize; }
        unsigned int getMaxNumObjects() const { return mMaxNumObjects; }
        unsigned int getNumFreeObjects() const { return (unsigned int)mFreeList.size(); }
        unsigned int getTotalMemory() const { return mObjectSize * mMaxNumObjects; }

    private:
        Byte* mpMemory = nullptr;
        unsigned int mMaxNumObjects = 0;
        unsigned int mObjectSize = 0;
        std::vector<Byte*> mFreeList;
    };
}
