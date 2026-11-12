#pragma once

#include "Trackable.h"

namespace core
{
    using EventType = int;

    // Each layer continues numbering from the one below it (see engine::InterfaceEventType)
    enum BaseEventType : EventType
    {
        INVALID_EVENT_TYPE = -1,
        NUM_BASE_EVENT_TYPES = 0
    };

    class Event : public Trackable
    {
    public:
        explicit Event(EventType type) : mType(type) {}
        virtual ~Event() = default;

        EventType getType() const { return mType; }

    private:
        EventType mType;
    };
}
