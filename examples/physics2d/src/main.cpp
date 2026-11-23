#ifdef TEMPLATE_HAS_CONFIG
#include <core/Config.h>
#endif

#include "Physics2DGame.h"

int main()
{
    // Window and timing settings come from config/physics2d.yaml; missing keys keep these defaults
    engine::GameConfig settings;
    settings.title = "CppTemplate 2D Physics";
    settings.fixedHz = 60.0;
    Physics2DSettings physics;
#ifdef TEMPLATE_HAS_CONFIG
    core::Config config;
    config.load(core::Config::findFile("physics2d.yaml", "PHYSICS2D_CONFIG"));
    settings.title = config.get<std::string>("window.title", settings.title);
    settings.width = config.get("window.width", settings.width);
    settings.height = config.get("window.height", settings.height);
    settings.fixedHz = config.get("physics.fixed_hz", settings.fixedHz);
    physics.friction = config.get("physics.friction", physics.friction);
    physics.restitution = config.get("physics.restitution", physics.restitution);
#endif

#ifdef __EMSCRIPTEN__
    // Heap allocated because run() never returns on the web and main's stack is unwound
    Physics2DGame* pGame = new Physics2DGame(settings, physics);
    pGame->run();
#else
    Physics2DGame game(settings, physics);
    game.run();
#endif
    return 0;
}
