#pragma once

#include <core/EventListener.h>

// Turns raw keyboard and mouse events into game events; rebinding controls only touches this class
class InputTranslator : public core::EventListener
{
public:
    InputTranslator();

private:
    void handleEvent(const core::Event& theEvent) override;
};
