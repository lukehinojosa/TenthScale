#ifdef TEMPLATE_HAS_CONFIG
#include <core/Config.h>
#endif

#include "Game.h"

int main()
{
    // Window and timing settings come from config/game.yaml; missing keys keep these defaults
    engine::GameConfig settings;
    settings.title = "Tenth Scale";
#ifdef TEMPLATE_HAS_CONFIG
    core::Config config;
    config.load(core::Config::findFile("game.yaml", "GAME_CONFIG"));
    settings.title = config.get<std::string>("window.title", settings.title);
    settings.width = config.get("window.width", settings.width);
    settings.height = config.get("window.height", settings.height);
    settings.fixedHz = config.get("physics.fixed_hz", settings.fixedHz);
    settings.masterVolume = config.get("audio.master_volume", settings.masterVolume);
#endif

#ifdef __EMSCRIPTEN__
    // Heap allocated because run() never returns on the web and main's stack is unwound
    Game* pGame = new Game(settings);
    pGame->run();
#else
    Game game(settings);
    game.run();
#endif
    return 0;
}
