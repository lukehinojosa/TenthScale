#include "engine/GraphicsBufferManager.h"

namespace engine
{
    GraphicsBuffer* GraphicsBufferManager::loadBuffer(const std::string& key, const std::string& filename)
    {
        auto pBuffer = std::make_unique<GraphicsBuffer>(filename);
        if (!pBuffer->isValid())
        {
            return nullptr;
        }
        return addBuffer(key, std::move(pBuffer));
    }

    GraphicsBuffer* GraphicsBufferManager::addBuffer(const std::string& key, std::unique_ptr<GraphicsBuffer> pBuffer)
    {
        GraphicsBuffer* pRaw = pBuffer.get();
        mBuffers[key] = std::move(pBuffer);
        return pRaw;
    }

    GraphicsBuffer* GraphicsBufferManager::getBuffer(const std::string& key) const
    {
        auto it = mBuffers.find(key);
        return it == mBuffers.end() ? nullptr : it->second.get();
    }

    void GraphicsBufferManager::deleteBuffer(const std::string& key)
    {
        mBuffers.erase(key);
    }

    void GraphicsBufferManager::clear()
    {
        mBuffers.clear();
    }
}
