#pragma once

#include <raylib.h>

#include <core/Event.h>

namespace engine
{
    // Game event enums should start at NUM_INTERFACE_EVENT_TYPES
    enum InterfaceEventType : core::EventType
    {
        KEYBOARD_EVENT = core::NUM_BASE_EVENT_TYPES,
        MOUSE_EVENT,
        NUM_INTERFACE_EVENT_TYPES
    };

    enum class InputAction
    {
        PRESSED,
        RELEASED
    };

    class KeyboardEvent : public core::Event
    {
    public:
        KeyboardEvent(KeyboardKey key, InputAction action) : Event(KEYBOARD_EVENT), mKey(key), mAction(action) {}

        KeyboardKey getKey() const { return mKey; }
        InputAction getAction() const { return mAction; }

    private:
        KeyboardKey mKey;
        InputAction mAction;
    };

    class MouseEvent : public core::Event
    {
    public:
        MouseEvent(Vector2 position, MouseButton button, InputAction action)
            : Event(MOUSE_EVENT), mPosition(position), mButton(button), mAction(action)
        {
        }

        Vector2 getPosition() const { return mPosition; }
        MouseButton getButton() const { return mButton; }
        InputAction getAction() const { return mAction; }

    private:
        Vector2 mPosition;
        MouseButton mButton;
        InputAction mAction;
    };
}
