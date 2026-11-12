#pragma once

#include <map>
#include <vector>

#include "Event.h"
#include "Trackable.h"

namespace core
{
    class EventListener;

    // Global publish/subscribe hub: listeners register for event types and fireEvent calls them
    class EventSystem : public Trackable
    {
    public:
        static void createInstance();
        static void destroyInstance();
        static EventSystem* getInstance() { return mspInstance; }

        void init();
        void cleanup();

        void addListener(EventType type, EventListener* pListener);
        void removeListener(EventType type, EventListener* pListener);
        void removeListenerFromAllEvents(EventListener* pListener);

        // Listeners may add or remove listeners while handling the event
        void fireEvent(const Event& theEvent);

        std::size_t getNumListeners() const { return mListeners.size(); }

    private:
        EventSystem() = default;
        ~EventSystem() { cleanup(); }

        static EventSystem* mspInstance;
        std::multimap<EventType, EventListener*> mListeners;
        bool mIsInitted = false;
    };
}
