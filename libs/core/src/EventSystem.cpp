#include "core/EventSystem.h"
#include "core/EventListener.h"

#include <algorithm>

namespace core
{
    EventSystem* EventSystem::mspInstance = nullptr;

    void EventSystem::createInstance()
    {
        if (mspInstance == nullptr)
        {
            mspInstance = new EventSystem;
        }
    }

    void EventSystem::destroyInstance()
    {
        delete mspInstance;
        mspInstance = nullptr;
    }

    void EventSystem::init()
    {
        cleanup();
        mIsInitted = true;
    }

    void EventSystem::cleanup()
    {
        mListeners.clear();
        mIsInitted = false;
    }

    void EventSystem::addListener(EventType type, EventListener* pListener)
    {
        auto range = mListeners.equal_range(type);
        for (auto it = range.first; it != range.second; ++it)
        {
            if (it->second == pListener)
            {
                return;
            }
        }
        mListeners.emplace(type, pListener);
    }

    void EventSystem::removeListener(EventType type, EventListener* pListener)
    {
        auto range = mListeners.equal_range(type);
        for (auto it = range.first; it != range.second; ++it)
        {
            if (it->second == pListener)
            {
                mListeners.erase(it);
                return;
            }
        }
    }

    void EventSystem::removeListenerFromAllEvents(EventListener* pListener)
    {
        for (auto it = mListeners.begin(); it != mListeners.end();)
        {
            if (it->second == pListener)
            {
                it = mListeners.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    void EventSystem::fireEvent(const Event& theEvent)
    {
        // Snapshot first, since a handler can change the listener map
        std::vector<EventListener*> targets;
        auto range = mListeners.equal_range(theEvent.getType());
        for (auto it = range.first; it != range.second; ++it)
        {
            targets.push_back(it->second);
        }

        for (EventListener* pListener : targets)
        {
            // Skip anyone removed by an earlier handler in this same dispatch
            bool stillListening = false;
            auto current = mListeners.equal_range(theEvent.getType());
            for (auto it = current.first; it != current.second; ++it)
            {
                if (it->second == pListener)
                {
                    stillListening = true;
                    break;
                }
            }

            if (stillListening)
            {
                pListener->handleEvent(theEvent);
            }
        }
    }
}
