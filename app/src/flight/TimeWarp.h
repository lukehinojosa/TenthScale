#pragma once

#include <array>

namespace tenth
{
    // Speeds up flight by running more of the normal fixed physics steps each tick, so nothing about the physics
    // changes but how many steps pass per second. A slow computer runs fewer than asked rather than freezing.
    class TimeWarp
    {
    public:
        static constexpr std::array<int, 7> RATES = { 1, 2, 5, 10, 50, 100, 1000 };
        static constexpr double SMOOTHING = 0.1; // share of each tick's rate the readout of the achieved rate takes in

        void faster()
        {
            setLevel(mLevel + 1);
        }

        void slower()
        {
            setLevel(mLevel - 1);
        }

        // Back to real time
        void stop()
        {
            setLevel(0);
        }

        void setLevel(int level)
        {
            mLevel = level < 0 ? 0 : (level > mMaxLevel ? mMaxLevel : level);
            mAchieved = rate();
        }

        // Holds warp to real time, such as while a tank is about to run dry, dropping out of any warp already on.
        // Lifting the hold leaves the rate where it is.
        void setHeld(bool held)
        {
            if (held == mHeld)
            {
                return;
            }
            mHeld = held;
            mMaxLevel = held ? 0 : (int)RATES.size() - 1;
            if (held)
            {
                setLevel(0);
            }
        }

        bool held() const { return mHeld; }
        int level() const { return mLevel; }
        int rate() const { return RATES[mLevel]; }

        void togglePause() { mPaused = !mPaused; }
        void setPaused(bool paused) { mPaused = paused; }
        bool paused() const { return mPaused; }

        // How many steps a tick has really been running lately; below the rate when the computer can't keep up
        double achieved() const { return mPaused ? 0.0 : mAchieved; }

        // Runs up to rate() physics steps for one tick, stopping early once budget seconds of real time have passed by
        // now(), or once a step puts on a hold. Always runs at least one step unless paused. Returns how many it ran.
        template <typename Step, typename Clock>
        int run(Step step, Clock now, double budget)
        {
            if (mPaused)
            {
                return 0;
            }
            const double start = now();
            int done = 0;
            while (done < rate())
            {
                step();
                done++;
                if (now() - start > budget)
                {
                    break;
                }
            }
            mAchieved += (done - mAchieved) * SMOOTHING;
            return done;
        }

    private:
        int mLevel = 0;
        int mMaxLevel = (int)RATES.size() - 1;
        bool mHeld = false;
        bool mPaused = false;
        double mAchieved = 1.0;
    };
}
