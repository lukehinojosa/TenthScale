#include "core/MemoryPool.h"

#include <bit>
#include <cassert>
#include <cstdlib>

namespace core
{
    MemoryPool::MemoryPool(unsigned int maxNumObjects, unsigned int objectSize)
        : mMaxNumObjects(maxNumObjects)
        , mObjectSize(std::bit_ceil(objectSize < 1u ? 1u : objectSize))
    {
        mpMemory = static_cast<Byte*>(std::malloc((std::size_t)mObjectSize * mMaxNumObjects));
        assert(mpMemory != nullptr);
        reset();
    }

    MemoryPool::~MemoryPool()
    {
        std::free(mpMemory);
    }

    Byte* MemoryPool::allocateObject()
    {
        if (mFreeList.empty())
        {
            return nullptr;
        }

        Byte* ptr = mFreeList.back();
        mFreeList.pop_back();
        return ptr;
    }

    void MemoryPool::freeObject(Byte* ptr)
    {
        if (ptr == nullptr)
        {
            return;
        }

        assert(contains(ptr));
        assert((ptr - mpMemory) % mObjectSize == 0); // Must be the start of a slot
        mFreeList.push_back(ptr);
    }

    void MemoryPool::reset()
    {
        mFreeList.clear();
        mFreeList.reserve(mMaxNumObjects);

        // Pushed in reverse so the first allocation gets the lowest address
        for (unsigned int i = mMaxNumObjects; i > 0; i--)
        {
            mFreeList.push_back(mpMemory + (std::size_t)(i - 1) * mObjectSize);
        }
    }

    bool MemoryPool::contains(const Byte* ptr) const
    {
        return ptr >= mpMemory && ptr < mpMemory + getTotalMemory();
    }
}
