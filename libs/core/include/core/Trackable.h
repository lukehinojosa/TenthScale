#pragma once

#include <cstddef>
#include <new>

#include "MemoryTracker.h"

namespace core
{
    // Inherit from this to have every heap allocation of the class recorded by MemoryTracker
    class Trackable
    {
    public:
        static void* operator new(std::size_t size)
        {
            void* ptr = ::operator new(size);
            MemoryTracker::getInstance().addAllocation(ptr, size);
            return ptr;
        }

        static void* operator new[](std::size_t size)
        {
            void* ptr = ::operator new[](size);
            MemoryTracker::getInstance().addAllocation(ptr, size);
            return ptr;
        }

        static void operator delete(void* ptr)
        {
            MemoryTracker::getInstance().removeAllocation(ptr);
            ::operator delete(ptr);
        }

        static void operator delete[](void* ptr)
        {
            MemoryTracker::getInstance().removeAllocation(ptr);
            ::operator delete[](ptr);
        }

        // Placement new is untracked, since the memory belongs to someone else (a MemoryManager, say)
        static void* operator new(std::size_t, void* where)
        {
            return where;
        }

        static void operator delete(void*, void*)
        {
        }
    };
}
