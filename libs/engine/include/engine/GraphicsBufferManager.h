#pragma once

#include <map>
#include <memory>
#include <string>

#include <core/Trackable.h>

#include "GraphicsBuffer.h"

namespace engine
{
    // Owns every GraphicsBuffer, looked up by name, so sprites can share one sheet
    class GraphicsBufferManager : public core::Trackable
    {
    public:
        // Returns the buffer, or nullptr if the file couldn't be loaded; replaces any buffer with that key
        GraphicsBuffer* loadBuffer(const std::string& key, const std::string& filename);

        // Takes ownership; replaces any buffer with that key
        GraphicsBuffer* addBuffer(const std::string& key, std::unique_ptr<GraphicsBuffer> pBuffer);

        GraphicsBuffer* getBuffer(const std::string& key) const;
        void deleteBuffer(const std::string& key);
        void clear();

        std::size_t getNumBuffers() const { return mBuffers.size(); }

    private:
        std::map<std::string, std::unique_ptr<GraphicsBuffer>> mBuffers;
    };
}
