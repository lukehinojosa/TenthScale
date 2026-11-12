#pragma once

#include <raylib.h>

#include "GraphicsBuffer.h"

namespace engine
{
    // A rectangle of a GraphicsBuffer; cheap to copy, and doesn't own the buffer
    class Sprite
    {
    public:
        Sprite() = default;
        Sprite(const GraphicsBuffer* pBuffer, Rectangle source) : mpBuffer(pBuffer), mSource(source) {}

        const GraphicsBuffer* getBuffer() const { return mpBuffer; }
        Rectangle getSource() const { return mSource; }
        float getWidth() const { return mSource.width; }
        float getHeight() const { return mSource.height; }

        // position is the sprite's center; rotation is in degrees
        void draw(Vector2 position, float scale = 1.0f, float rotation = 0.0f, Color tint = WHITE) const
        {
            if (mpBuffer == nullptr || !mpBuffer->isValid())
            {
                return;
            }
            Rectangle dest{ position.x, position.y, mSource.width * scale, mSource.height * scale };
            Vector2 origin{ dest.width * 0.5f, dest.height * 0.5f };
            DrawTexturePro(mpBuffer->getTexture(), mSource, dest, origin, rotation, tint);
        }

    private:
        const GraphicsBuffer* mpBuffer = nullptr;
        Rectangle mSource{};
    };
}
