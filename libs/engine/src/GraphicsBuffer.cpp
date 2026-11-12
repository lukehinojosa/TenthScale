#include "engine/GraphicsBuffer.h"

namespace engine
{
    GraphicsBuffer::GraphicsBuffer(const std::string& filename)
    {
        mTexture = LoadTexture(filename.c_str());
    }

    GraphicsBuffer::GraphicsBuffer(const Image& image)
    {
        mTexture = LoadTextureFromImage(image);
    }

    GraphicsBuffer::GraphicsBuffer(int width, int height, Color fill) : mIsTarget(true)
    {
        mTarget = LoadRenderTexture(width, height);
        BeginTextureMode(mTarget);
        ClearBackground(fill);
        EndTextureMode();
    }

    GraphicsBuffer::~GraphicsBuffer()
    {
        if (mIsTarget)
        {
            UnloadRenderTexture(mTarget);
        }
        else if (mTexture.id != 0)
        {
            UnloadTexture(mTexture);
        }
    }

    void GraphicsBuffer::beginDrawingInto()
    {
        if (mIsTarget)
        {
            BeginTextureMode(mTarget);
        }
    }

    void GraphicsBuffer::endDrawingInto()
    {
        if (mIsTarget)
        {
            EndTextureMode();
        }
    }

    void GraphicsBuffer::draw(Vector2 position, Color tint) const
    {
        const Texture2D& texture = getTexture();
        if (mIsTarget)
        {
            // Render textures are stored upside down, so flip the source rectangle
            DrawTextureRec(texture, Rectangle{ 0, 0, (float)texture.width, -(float)texture.height }, position, tint);
        }
        else
        {
            DrawTextureV(texture, position, tint);
        }
    }
}
