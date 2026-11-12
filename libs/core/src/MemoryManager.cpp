#include "core/MemoryManager.h"

#include <algorithm>
#include <cassert>
#include <iomanip>

namespace core
{
    void MemoryManager::init(const std::vector<unsigned int>& sizes, const std::vector<unsigned int>& numSlots)
    {
        assert(sizes.size() == numSlots.size());
        if (sizes.size() != numSlots.size())
        {
            return;
        }

        cleanup();
        for (std::size_t i = 0; i < sizes.size(); i++)
        {
            mPools.push_back(new MemoryPool(numSlots[i], sizes[i]));
        }

        // Smallest first, so allocate() can take the first pool that fits
        std::sort(mPools.begin(), mPools.end(), [](const MemoryPool* a, const MemoryPool* b)
        {
            return a->getMaxObjectSize() < b->getMaxObjectSize();
        });
    }

    void MemoryManager::cleanup()
    {
        for (MemoryPool* pPool : mPools)
        {
            delete pPool;
        }
        mPools.clear();
        mRequestedSizes.clear();
        mPeakAllocated = 0;
        mNumFailedAllocations = 0;
    }

    void MemoryManager::reset()
    {
        for (MemoryPool* pPool : mPools)
        {
            pPool->reset();
        }
        mRequestedSizes.clear();
    }

    Byte* MemoryManager::allocate(unsigned int size)
    {
        for (MemoryPool* pPool : mPools)
        {
            if (pPool->getMaxObjectSize() < size || pPool->getNumFreeObjects() == 0)
            {
                continue;
            }

            Byte* ptr = pPool->allocateObject();
            if (ptr != nullptr)
            {
                mRequestedSizes[ptr] = size;
                mPeakAllocated = std::max(mPeakAllocated, getTotalAllocated());
                return ptr;
            }
        }

        mNumFailedAllocations++;
        return nullptr;
    }

    void MemoryManager::deallocate(Byte* ptr)
    {
        if (ptr == nullptr)
        {
            return;
        }

        for (MemoryPool* pPool : mPools)
        {
            if (pPool->contains(ptr))
            {
                pPool->freeObject(ptr);
                mRequestedSizes.erase(ptr);
                return;
            }
        }

        assert(false && "Pointer was not allocated by this MemoryManager");
    }

    unsigned int MemoryManager::getTotalAllocated() const
    {
        unsigned int total = 0;
        for (const MemoryPool* pPool : mPools)
        {
            unsigned int numInUse = pPool->getMaxNumObjects() - pPool->getNumFreeObjects();
            total += numInUse * pPool->getMaxObjectSize();
        }
        return total;
    }

    unsigned int MemoryManager::getTotalCapacity() const
    {
        unsigned int total = 0;
        for (const MemoryPool* pPool : mPools)
        {
            total += pPool->getTotalMemory();
        }
        return total;
    }

    unsigned int MemoryManager::getTotalRequested() const
    {
        unsigned int total = 0;
        for (const auto& entry : mRequestedSizes)
        {
            total += entry.second;
        }
        return total;
    }

    unsigned int MemoryManager::getTotalWaste() const
    {
        return getTotalAllocated() - getTotalRequested();
    }

    void MemoryManager::reportStats(std::ostream& stream) const
    {
        unsigned int capacity = getTotalCapacity();
        unsigned int allocated = getTotalAllocated();
        double percentUsed = capacity > 0 ? 100.0 * allocated / capacity : 0.0;

        stream << "Allocs: " << getNumAllocations()
            << " | Allocated: " << allocated << "/" << capacity << " bytes ("
            << std::fixed << std::setprecision(2) << percentUsed << "%)"
            << " | Requested: " << getTotalRequested()
            << " | Waste: " << getTotalWaste()
            << " | Peak: " << mPeakAllocated
            << " | Failed: " << mNumFailedAllocations
            << "\n";
    }
}
