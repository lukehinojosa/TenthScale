#pragma once

#include <cassert>
#include <vector>

#include "Component.h"
#include "Trackable.h"

namespace core
{
    // Fixed-capacity pool of one component type, stored contiguously so updates walk memory in order
    template<class T>
    class ComponentPool : public Trackable
    {
    public:
        void init(unsigned int capacity)
        {
            // Sized once and never resized, so pointers into the array stay valid
            mComponents.clear();
            mComponents.resize(capacity);
            mInUse.assign(capacity, false);
            mFreeIndices.clear();
            for (unsigned int i = capacity; i > 0; i--)
            {
                mFreeIndices.push_back(i - 1);
            }
        }

        // Returns nullptr when the pool is full
        T* acquire(Entity* pOwner)
        {
            if (mFreeIndices.empty())
            {
                return nullptr;
            }

            unsigned int index = mFreeIndices.back();
            mFreeIndices.pop_back();
            mInUse[index] = true;

            T* pComponent = &mComponents[index];
            static_cast<Component*>(pComponent)->mpOwner = pOwner;
            return pComponent;
        }

        void release(T* pComponent)
        {
            if (pComponent == nullptr)
            {
                return;
            }

            unsigned int index = (unsigned int)(pComponent - mComponents.data());
            assert(index < mComponents.size() && mInUse[index]);

            mComponents[index] = T(); // Reset to a clean slot
            mInUse[index] = false;
            mFreeIndices.push_back(index);
        }

        void updateAll(double dt)
        {
            for (unsigned int i = 0; i < mComponents.size(); i++)
            {
                if (mInUse[i])
                {
                    mComponents[i].update(dt);
                }
            }
        }

        void drawAll()
        {
            for (unsigned int i = 0; i < mComponents.size(); i++)
            {
                if (mInUse[i])
                {
                    mComponents[i].draw();
                }
            }
        }

        unsigned int getCapacity() const { return (unsigned int)mComponents.size(); }
        unsigned int getNumInUse() const { return (unsigned int)(mComponents.size() - mFreeIndices.size()); }

    private:
        std::vector<T> mComponents;
        std::vector<bool> mInUse;
        std::vector<unsigned int> mFreeIndices;
    };
}
