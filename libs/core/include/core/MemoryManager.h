#pragma once

#include <ostream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "MemoryPool.h"
#include "Trackable.h"

namespace core
{
    // A set of MemoryPools of different slot sizes; each request goes to the smallest pool that fits
    class MemoryManager : public Trackable
    {
    public:
        MemoryManager() = default;
        ~MemoryManager() { cleanup(); }

        MemoryManager(const MemoryManager&) = delete;
        MemoryManager& operator=(const MemoryManager&) = delete;

        // sizes[i] is a slot size and numSlots[i] is how many slots of that size to make
        void init(const std::vector<unsigned int>& sizes, const std::vector<unsigned int>& numSlots);
        void cleanup();
        void reset();

        // Returns nullptr (and counts a failure) when no pool can fit the request
        Byte* allocate(unsigned int size);
        void deallocate(Byte* ptr);

        // Constructs a T in pooled memory, or returns nullptr when the pools are full
        template<class T, class... Args>
        T* create(Args&&... args)
        {
            Byte* ptr = allocate((unsigned int)sizeof(T));
            if (ptr == nullptr)
            {
                return nullptr;
            }
            return new (ptr) T(std::forward<Args>(args)...);
        }

        template<class T>
        void destroy(T* pObject)
        {
            if (pObject == nullptr)
            {
                return;
            }
            pObject->~T();
            deallocate(reinterpret_cast<Byte*>(pObject));
        }

        unsigned int getTotalAllocated() const; // Padded size of every slot in use, waste included
        unsigned int getTotalCapacity() const;
        unsigned int getTotalRequested() const; // What callers actually asked for
        unsigned int getTotalWaste() const;
        unsigned int getNumAllocations() const { return (unsigned int)mRequestedSizes.size(); }
        unsigned int getPeakAllocated() const { return mPeakAllocated; }
        unsigned int getNumFailedAllocations() const { return mNumFailedAllocations; }

        void reportStats(std::ostream& stream) const;

    private:
        std::vector<MemoryPool*> mPools; // Sorted by slot size, smallest first
        std::unordered_map<Byte*, unsigned int> mRequestedSizes;
        unsigned int mPeakAllocated = 0;
        unsigned int mNumFailedAllocations = 0;
    };
}
