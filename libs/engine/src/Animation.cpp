#include "engine/Animation.h"

namespace engine
{
    Animation Animation::fromSheet(const GraphicsBuffer* pBuffer, int frameWidth, int frameHeight, int frameCount, bool loop, double framesPerSecond)
    {
        Animation animation(loop, framesPerSecond);
        if (pBuffer == nullptr || frameWidth <= 0 || frameHeight <= 0)
        {
            return animation;
        }

        int columns = pBuffer->getWidth() / frameWidth;
        if (columns <= 0)
        {
            return animation;
        }
        for (int i = 0; i < frameCount; i++)
        {
            Rectangle source{ (float)((i % columns) * frameWidth), (float)((i / columns) * frameHeight), (float)frameWidth, (float)frameHeight };
            animation.addSprite(Sprite(pBuffer, source));
        }
        return animation;
    }

    void Animation::update(double dt)
    {
        if (mSprites.empty() || mFramesPerSecond <= 0.0)
        {
            return;
        }

        mElapsed += dt;
        const double secondsPerFrame = 1.0 / mFramesPerSecond;
        while (mElapsed >= secondsPerFrame)
        {
            mElapsed -= secondsPerFrame;
            mCurrentFrame++;
            if (mCurrentFrame >= (int)mSprites.size())
            {
                mCurrentFrame = mLoop ? 0 : (int)mSprites.size() - 1;
            }
        }
    }

    void Animation::reset()
    {
        mCurrentFrame = 0;
        mElapsed = 0.0;
    }

    const Sprite& Animation::getCurrentSprite() const
    {
        static const Sprite EMPTY;
        if (mSprites.empty())
        {
            return EMPTY;
        }
        return mSprites[mCurrentFrame];
    }
}
