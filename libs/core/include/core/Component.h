#pragma once

#include <cstdint>

#include "Trackable.h"

namespace core
{
    using EntityID = std::uint32_t;

    // Anything that owns components; subclass it for your game objects
    class Entity : public Trackable
    {
    public:
        explicit Entity(EntityID id) : mID(id) {}
        virtual ~Entity() = default;

        EntityID getID() const { return mID; }

    private:
        EntityID mID;
    };

    template<class T> class ComponentPool;

    // One piece of an Entity's behavior; siblings talk to each other through the owner
    class Component : public Trackable
    {
    public:
        template<class T> friend class ComponentPool;

        virtual ~Component() = default;

        virtual void update(double /*dt*/) {}
        virtual void draw() {}

        Entity* getOwner() const { return mpOwner; }

        template<class T>
        T* getOwnerAs() const
        {
            return static_cast<T*>(mpOwner);
        }

    protected:
        Component() = default;

        Entity* mpOwner = nullptr; // Not owned
    };
}
