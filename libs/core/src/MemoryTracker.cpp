#include "core/MemoryTracker.h"

#include <algorithm>
#include <vector>

namespace core
{
    MemoryTracker& MemoryTracker::getInstance()
    {
        static MemoryTracker instance;
        return instance;
    }

    void MemoryTracker::addAllocation(void* ptr, std::size_t size)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mAllocations[ptr] = AllocationRecord{ size, mNextOrder++ };
    }

    void MemoryTracker::removeAllocation(void* ptr)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mAllocations.erase(ptr);
    }

    std::size_t MemoryTracker::getNumAllocations() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        return mAllocations.size();
    }

    std::size_t MemoryTracker::getTotalBytes() const
    {
        std::lock_guard<std::mutex> lock(mMutex);
        std::size_t total = 0;
        for (const auto& entry : mAllocations)
        {
            total += entry.second.size;
        }
        return total;
    }

    void MemoryTracker::reportAllocations(std::ostream& stream) const
    {
        std::lock_guard<std::mutex> lock(mMutex);

        std::vector<std::pair<void*, AllocationRecord>> sorted(mAllocations.begin(), mAllocations.end());
        std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b)
        {
            return a.second.order < b.second.order;
        });

        stream << "Tracked allocations still alive: " << sorted.size() << "\n";
        for (const auto& entry : sorted)
        {
            stream << "  #" << entry.second.order << " at " << entry.first << ", " << entry.second.size << " bytes\n";
        }
    }
}
