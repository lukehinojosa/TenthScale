#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "Trackable.h"

namespace core
{
    using CallbackID = std::uint32_t;

    // Owns delayed and conditional callbacks; each one is removed once it has fired
    class CallbackManager : public Trackable
    {
    public:
        // Fires once after delaySeconds, or every delaySeconds when repeat is true
        CallbackID addTimed(double delaySeconds, std::function<void()> func, bool repeat = false);

        // Checks condition every update and fires once the first time it is true
        CallbackID addConditional(std::function<bool()> condition, std::function<void()> func);

        // Returns false if the callback already fired or never existed
        bool cancel(CallbackID id);
        void clear();

        void update(double dtSeconds);

        std::size_t getNumCallbacks() const { return mCallbacks.size(); }

    private:
        struct Callback
        {
            CallbackID id = 0;
            std::function<void()> func;
            std::function<bool()> condition; // Empty for timed callbacks
            double delay = 0.0;
            double remaining = 0.0;
            bool repeat = false;
            bool cancelled = false;
        };

        std::vector<std::unique_ptr<Callback>> mCallbacks;
        CallbackID mNextID = 1;
    };
}
