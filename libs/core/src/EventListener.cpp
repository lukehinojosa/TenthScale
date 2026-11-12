#include "core/EventListener.h"
#include "core/EventSystem.h"

namespace core
{
    EventListener::~EventListener()
    {
        if (EventSystem* pEventSystem = EventSystem::getInstance())
        {
            pEventSystem->removeListenerFromAllEvents(this);
        }
    }
}
