#pragma once

#include <chrono>
#include <thread>

namespace core
{
    // Wall-clock stopwatch for profiling or pacing a loop
    class Timer
    {
    public:
        void start()
        {
            mStart = Clock::now();
        }

        double getElapsedSeconds() const
        {
            return std::chrono::duration<double>(Clock::now() - mStart).count();
        }

        double getElapsedMS() const
        {
            return getElapsedSeconds() * 1000.0;
        }

        // Sleeps until this many milliseconds have passed since start()
        void sleepUntilElapsed(double ms) const
        {
            double remaining = ms - getElapsedMS();
            if (remaining > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(remaining));
            }
        }

    private:
        using Clock = std::chrono::steady_clock;
        Clock::time_point mStart = Clock::now();
    };
}
