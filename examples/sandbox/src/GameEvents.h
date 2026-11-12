#pragma once

#include <raylib.h>

#include <engine/InterfaceEvent.h>

// What the game reacts to, decoupled from which key or button caused it
enum GameEventType : core::EventType
{
    QUIT_GAME = engine::NUM_INTERFACE_EVENT_TYPES,
    RESET_SCENE,
    TOGGLE_PAUSE,
    ADD_UNIT,
    DELETE_UNIT_AT,
    DELETE_ALL_UNITS,
    TOGGLE_LAST_UNIT_ANIMATION,
    FREEZE_UNITS,
    NUM_GAME_EVENT_TYPES
};

// Game events that need a screen position (ADD_UNIT, DELETE_UNIT_AT) carry it here
class UnitPositionEvent : public core::Event
{
public:
    UnitPositionEvent(GameEventType type, Vector2 position) : Event(type), mPosition(position) {}

    Vector2 getPosition() const { return mPosition; }

private:
    Vector2 mPosition;
};
