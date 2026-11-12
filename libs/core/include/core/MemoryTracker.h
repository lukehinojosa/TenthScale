#pragma once

#include <cstddef>
#include <mutex>
#include <ostream>
#include <unordered_map>

namespace core
{
    // Records every live allocation made by a Trackable so leaks can be reported at shutdown
    class MemoryTracker
    {
    public:
        static MemoryTracker& getInstance();

        void addAllocation(void* ptr, std::size_t size);
        void removeAllocation(void* ptr);

        std::size_t getNumAllocations() const;
        std::size_t getTotalBytes() const;

        void reportAllocations(std::ostream& stream) const;

    private:
        MemoryTracker() = default;

        struct AllocationRecord
        {
            std::size_t size = 0;
            std::size_t order = 0; // Lets the report list allocations in the order they happened
        };

        mutable std::mutex mMutex;
        std::unordered_map<void*, AllocationRecord> mAllocations;
        std::size_t mNextOrder = 0;
    };
}
