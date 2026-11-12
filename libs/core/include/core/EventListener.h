#pragma once

#include "Event.h"
#include "Trackable.h"

namespace core
{
    // Removes itself from the EventSystem when destroyed, so a dead listener is never called
    class EventListener : public Trackable
    {
    public:
        EventListener() = default;
        virtual ~EventListener();

        EventListener(const EventListener&) = delete;
        EventListener& operator=(const EventListener&) = delete;

        virtual void handleEvent(const Event& theEvent) = 0;
    };
}
