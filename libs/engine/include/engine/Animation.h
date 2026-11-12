#pragma once

#include <vector>

#include <core/Trackable.h>

#include "Sprite.h"

namespace engine
{
    // A sequence of sprites played at a fixed frame rate, looping or holding on the last frame
    class Animation : public core::Trackable
    {
    public:
        explicit Animation(bool loop = true, double framesPerSecond = 15.0) : mLoop(loop), mFramesPerSecond(framesPerSecond) {}

        // Cuts frameCount equal frames from a sheet, left to right then top to bottom
        static Animation fromSheet(const GraphicsBuffer* pBuffer, int frameWidth, int frameHeight, int frameCount, bool loop = true, double framesPerSecond = 15.0);

        void addSprite(const Sprite& sprite) { mSprites.push_back(sprite); }
        void update(double dt);
        void reset();

        // Returns an empty sprite if the animation has no frames
        const Sprite& getCurrentSprite() const;
        int getCurrentFrame() const { return mCurrentFrame; }
        int getFrameCount() const { return (int)mSprites.size(); }
        bool isFinished() const { return !mLoop && !mSprites.empty() && mCurrentFrame == (int)mSprites.size() - 1; }

        void setSpeed(double framesPerSecond) { mFramesPerSecond = framesPerSecond; }
        double getSpeed() const { return mFramesPerSecond; }
        void setLoop(bool loop) { mLoop = loop; }

    private:
        std::vector<Sprite> mSprites;
        bool mLoop;
        double mFramesPerSecond;
        int mCurrentFrame = 0;
        double mElapsed = 0.0;
    };
}
