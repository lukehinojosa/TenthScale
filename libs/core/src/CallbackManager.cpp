#include "core/CallbackManager.h"

#include <algorithm>

namespace core
{
    CallbackID CallbackManager::addTimed(double delaySeconds, std::function<void()> func, bool repeat)
    {
        auto pCallback = std::make_unique<Callback>();
        pCallback->id = mNextID++;
        pCallback->func = std::move(func);
        pCallback->delay = delaySeconds;
        pCallback->remaining = delaySeconds;
        pCallback->repeat = repeat;

        CallbackID id = pCallback->id;
        mCallbacks.push_back(std::move(pCallback));
        return id;
    }

    CallbackID CallbackManager::addConditional(std::function<bool()> condition, std::function<void()> func)
    {
        auto pCallback = std::make_unique<Callback>();
        pCallback->id = mNextID++;
        pCallback->func = std::move(func);
        pCallback->condition = std::move(condition);

        CallbackID id = pCallback->id;
        mCallbacks.push_back(std::move(pCallback));
        return id;
    }

    bool CallbackManager::cancel(CallbackID id)
    {
        for (auto& pCallback : mCallbacks)
        {
            if (pCallback->id == id && !pCallback->cancelled)
            {
                // Flagged rather than erased so cancelling from inside a callback is safe
                pCallback->cancelled = true;
                return true;
            }
        }
        return false;
    }

    void CallbackManager::clear()
    {
        mCallbacks.clear();
    }

    void CallbackManager::update(double dtSeconds)
    {
        // Index loop, since a callback can add new callbacks and grow the vector
        for (std::size_t i = 0; i < mCallbacks.size(); i++)
        {
            Callback* pCallback = mCallbacks[i].get();
            if (pCallback->cancelled)
            {
                continue;
            }

            bool shouldFire = false;
            if (pCallback->condition)
            {
                shouldFire = pCallback->condition();
            }
            else
            {
                pCallback->remaining -= dtSeconds;
                shouldFire = pCallback->remaining <= 0.0;
            }

            if (!shouldFire)
            {
                continue;
            }

            if (pCallback->repeat)
            {
                pCallback->remaining += pCallback->delay;
            }
            else
            {
                pCallback->cancelled = true;
            }
            mCallbacks[i]->func();
        }

        mCallbacks.erase(std::remove_if(mCallbacks.begin(), mCallbacks.end(), [](const auto& pCallback)
        {
            return pCallback->cancelled;
        }), mCallbacks.end());
    }
}
