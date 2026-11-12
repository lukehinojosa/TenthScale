#include "InputTranslator.h"
#include "GameEvents.h"

#include <core/EventSystem.h>

InputTranslator::InputTranslator()
{
    core::EventSystem* pEventSystem = core::EventSystem::getInstance();
    pEventSystem->addListener(engine::KEYBOARD_EVENT, this);
    pEventSystem->addListener(engine::MOUSE_EVENT, this);
}

void InputTranslator::handleEvent(const core::Event& theEvent)
{
    core::EventSystem* pEventSystem = core::EventSystem::getInstance();

    if (theEvent.getType() == engine::MOUSE_EVENT)
    {
        const auto& mouseEvent = static_cast<const engine::MouseEvent&>(theEvent);
        if (mouseEvent.getAction() != engine::InputAction::PRESSED)
        {
            return;
        }
        if (mouseEvent.getButton() == MOUSE_BUTTON_LEFT)
        {
            pEventSystem->fireEvent(UnitPositionEvent(ADD_UNIT, mouseEvent.getPosition()));
        }
        else if (mouseEvent.getButton() == MOUSE_BUTTON_MIDDLE)
        {
            pEventSystem->fireEvent(UnitPositionEvent(DELETE_UNIT_AT, mouseEvent.getPosition()));
        }
        return;
    }

    if (theEvent.getType() != engine::KEYBOARD_EVENT)
    {
        return;
    }

    const auto& keyEvent = static_cast<const engine::KeyboardEvent&>(theEvent);
    if (keyEvent.getAction() != engine::InputAction::PRESSED)
    {
        return;
    }

    switch (keyEvent.getKey())
    {
    case KEY_ESCAPE:
        pEventSystem->fireEvent(core::Event(QUIT_GAME));
        break;
    case KEY_R:
        pEventSystem->fireEvent(core::Event(RESET_SCENE));
        break;
    case KEY_SPACE:
        pEventSystem->fireEvent(core::Event(TOGGLE_PAUSE));
        break;
    case KEY_ENTER:
        pEventSystem->fireEvent(core::Event(TOGGLE_LAST_UNIT_ANIMATION));
        break;
    case KEY_F:
        pEventSystem->fireEvent(core::Event(FREEZE_UNITS));
        break;
    case KEY_BACKSPACE:
        pEventSystem->fireEvent(core::Event(DELETE_ALL_UNITS));
        break;
    default:
        break;
    }
}
