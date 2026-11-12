#pragma once

#include <string>

#include <raylib.h>

#include <core/Trackable.h>

namespace engine
{
    // An image on the GPU: loaded from a file or an Image, or a blank target you can draw into
    class GraphicsBuffer : public core::Trackable
    {
    public:
        explicit GraphicsBuffer(const std::string& filename);
        explicit GraphicsBuffer(const Image& image);
        GraphicsBuffer(int width, int height, Color fill = BLANK); // Drawable target
        ~GraphicsBuffer();

        GraphicsBuffer(const GraphicsBuffer&) = delete;
        GraphicsBuffer& operator=(const GraphicsBuffer&) = delete;

        bool isValid() const { return getTexture().id != 0; }
        int getWidth() const { return getTexture().width; }
        int getHeight() const { return getTexture().height; }
        const Texture2D& getTexture() const { return mIsTarget ? mTarget.texture : mTexture; }

        // Draw calls between these land in this buffer instead of the screen (targets only)
        void beginDrawingInto();
        void endDrawingInto();

        void draw(Vector2 position, Color tint = WHITE) const;

    private:
        Texture2D mTexture{};
        RenderTexture2D mTarget{};
        bool mIsTarget = false;
    };
}
