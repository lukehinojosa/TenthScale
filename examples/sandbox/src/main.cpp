#include <iostream>

#include <core/MemoryTracker.h>
#ifdef TEMPLATE_HAS_CONFIG
#include <core/Config.h>
#endif

#include "SandboxGame.h"

int main()
{
    SandboxSettings settings;
#ifdef TEMPLATE_HAS_CONFIG
    core::Config config;
    config.load(core::Config::findFile("sandbox.yaml", "SANDBOX_CONFIG"));
    settings.game.title = config.get<std::string>("window.title", settings.game.title);
    settings.game.width = config.get("window.width", settings.game.width);
    settings.game.height = config.get("window.height", settings.game.height);
    settings.game.fixedHz = config.get("physics.fixed_hz", settings.game.fixedHz);
    settings.restitution = config.get("physics.restitution", settings.restitution);
    settings.game.masterVolume = config.get("audio.master_volume", settings.game.masterVolume);
    settings.relayUrl = config.get<std::string>("net.relay_url", settings.relayUrl);
#endif

#ifdef __EMSCRIPTEN__
    // Heap allocated because run() never returns on the web and main's stack is unwound
    SandboxGame* pGame = new SandboxGame(settings);
    pGame->run();
#else
    {
        SandboxGame game(settings);
        game.run();
    }
    core::MemoryTracker::getInstance().reportAllocations(std::cout);
#endif
    return 0;
}
