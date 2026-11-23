#include <doctest/doctest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <engine/Game.h>
#include <engine/SceneManager.h>

namespace
{
    // Writes every hook it receives into a shared log, as "name:hook"
    class RecordingScene : public engine::Scene
    {
    public:
        RecordingScene(std::vector<std::string>& log, bool overlay = false) : mLog(log), mOverlay(overlay) {}

        void onLoad() override { record("load"); }
        void onUnload() override { record("unload"); }
        void onEnter() override { record("enter"); }
        void onExit() override { record("exit"); }
        void onPause() override { record("pause"); }
        void onResume() override { record("resume"); }
        void onFixedUpdate(double) override { record("fixed"); }
        void onDraw2D() override { record("draw"); }
        void onGui() override { record("gui"); }

        void onUpdate(double, double) override
        {
            record("update");
            if (!mNext.empty())
            {
                switchTo(mNext);
                mNext.clear();
                record("asked");
            }
            if (!mPushNext.empty())
            {
                push(mPushNext);
                mPushNext.clear();
            }
            if (mPopNext)
            {
                pop();
                mPopNext = false;
            }
        }

        bool isOverlay() const override { return mOverlay; }

        // Asks for a switch from inside its own update, the way a menu button would
        void switchOnNextUpdate(const std::string& name) { mNext = name; }
        void pushOnNextUpdate(const std::string& name) { mPushNext = name; }
        void popOnNextUpdate() { mPopNext = true; }

        // Reaches the game through the scene's helpers
        float cameraFov() const { return getCamera().fovy; }

    private:
        void record(const char* hook) { mLog.push_back(getName() + ":" + hook); }

        std::vector<std::string>& mLog;
        bool mOverlay;
        std::string mNext;
        std::string mPushNext;
        bool mPopNext = false;
    };

    struct Fixture
    {
        engine::Game game;
        engine::SceneManager scenes{ game };
        std::vector<std::string> log;

        RecordingScene& add(const std::string& name, bool overlay = false) { return scenes.add<RecordingScene>(name, log, overlay); }

        void step(double dt)
        {
            scenes.beginFrame(dt);
            scenes.update(dt, 0.0);
        }

        void frame()
        {
            scenes.beginFrame(0.02);
            scenes.fixedUpdate(0.02);
            scenes.update(0.02, 0.0);
            scenes.draw2D();
            scenes.gui();
        }

        std::vector<std::string> take()
        {
            std::vector<std::string> taken = log;
            log.clear();
            return taken;
        }
    };

    using Log = std::vector<std::string>;
}

TEST_CASE("A switch waits for the next update, loads the scene once, and exits the old one")
{
    Fixture f;
    RecordingScene& menu = f.add("menu");
    f.add("level");

    f.scenes.switchTo("menu");
    CHECK(f.scenes.getCurrent() == nullptr);
    CHECK(f.scenes.isTransitioning());
    f.step(0.02);
    CHECK(f.scenes.getCurrent() == &menu);
    CHECK(f.take() == Log{ "menu:load", "menu:enter", "menu:update" });
    CHECK(menu.cameraFov() == doctest::Approx(45.0f));

    f.scenes.switchTo("level");
    f.step(0.02);
    f.scenes.switchTo("menu");
    f.step(0.02);
    CHECK(f.take() == Log{ "menu:exit", "level:load", "level:enter", "level:update", "level:exit", "menu:enter", "menu:update" });
}

TEST_CASE("A scene can put another on top of itself and take it off again through its own shortcuts")
{
    Fixture f;
    RecordingScene& level = f.add("level");
    RecordingScene& pause = f.add("pause", true);
    f.scenes.switchTo("level");
    f.step(0.02);

    level.pushOnNextUpdate("pause");
    f.step(0.02);
    f.step(0.02);
    CHECK(f.scenes.getCurrent() == &pause);
    CHECK(f.scenes.getDepth() == 2);

    pause.popOnNextUpdate();
    f.step(0.02);
    f.step(0.02);
    CHECK(f.scenes.getCurrent() == &level);
    CHECK(f.scenes.getDepth() == 1);
}

TEST_CASE("A scene that asks for a switch inside its update finishes the frame before it goes")
{
    Fixture f;
    RecordingScene& menu = f.add("menu");
    f.add("level");
    f.scenes.switchTo("menu");
    f.frame();
    f.take();

    menu.switchOnNextUpdate("level");
    f.frame();
    CHECK(f.take() == Log{ "menu:fixed", "menu:update", "menu:asked", "menu:draw", "menu:gui" });
    f.frame();
    CHECK(f.take() == Log{ "menu:exit", "level:load", "level:enter", "level:fixed", "level:update", "level:draw", "level:gui" });
    CHECK(f.scenes.getCurrent()->getName() == "level");
}

TEST_CASE("A pushed overlay pauses the scene under it, which still draws but doesn't update")
{
    Fixture f;
    f.add("level");
    f.add("pause", true);
    f.add("options");
    f.scenes.switchTo("level");
    f.step(0.02);
    f.take();

    f.scenes.push("pause");
    f.step(0.02);
    CHECK(f.take() == Log{ "level:pause", "pause:load", "pause:enter", "pause:update" });
    CHECK(f.scenes.getDepth() == 2);

    f.scenes.fixedUpdate(0.02);
    f.scenes.draw2D();
    f.scenes.gui();
    CHECK(f.take() == Log{ "pause:fixed", "level:draw", "pause:draw", "pause:gui" });

    // A full scene on top hides everything under it
    f.scenes.push("options");
    f.step(0.02);
    f.take();
    f.scenes.draw2D();
    CHECK(f.take() == Log{ "options:draw" });

    f.scenes.pop();
    f.scenes.pop();
    f.step(0.02);
    CHECK(f.take() == Log{ "options:exit", "pause:resume", "pause:exit", "level:resume", "level:update" });
    CHECK(f.scenes.getDepth() == 1);
}

TEST_CASE("A fading switch keeps the old scene until the screen is covered, then fades the new one in")
{
    Fixture f;
    f.add("menu");
    f.add("level");
    f.scenes.switchTo("menu");
    f.step(0.02);

    f.scenes.switchTo("level", engine::SceneTransition::fade(1.0f));
    f.step(0.25);
    CHECK(f.scenes.getCurrent()->getName() == "menu");
    CHECK(f.scenes.getFadeAmount() == doctest::Approx(0.5f));

    f.step(0.3);
    CHECK(f.scenes.getCurrent()->getName() == "level");
    CHECK(f.scenes.getFadeAmount() == doctest::Approx(1.0f));

    f.step(0.25);
    CHECK(f.scenes.getFadeAmount() == doctest::Approx(0.5f));
    f.step(0.3);
    CHECK(f.scenes.getFadeAmount() == doctest::Approx(0.0f));
    CHECK_FALSE(f.scenes.isTransitioning());
}

TEST_CASE("Unloading exits the stack and unloads only scenes that were loaded")
{
    Fixture f;
    f.add("menu");
    f.add("never");
    f.scenes.switchTo("menu");
    f.step(0.02);
    f.take();

    f.scenes.unloadAll();
    CHECK(f.take() == Log{ "menu:exit", "menu:unload" });
    CHECK(f.scenes.getCurrent() == nullptr);
}

TEST_CASE("Unknown names are ignored, and adding a name twice throws")
{
    Fixture f;
    f.add("menu");
    f.scenes.switchTo("missing");
    f.scenes.push("missing");
    f.scenes.pop();
    f.step(0.02);
    CHECK(f.scenes.getCurrent() == nullptr);
    CHECK_FALSE(f.scenes.isTransitioning());
    CHECK(f.scenes.find("missing") == nullptr);
    CHECK_THROWS_AS(f.add("menu"), std::logic_error);
}

TEST_CASE("A value given to switchTo or push reaches the scene before it loads and enters")
{
    // Reads its value in onLoad and onEnter, the way a level would read which map to open
    class LevelScene : public engine::Scene
    {
    public:
        void onLoad() override { loadedWith = getParameter(-1); }
        void onEnter() override { enteredWith = getParameter(-1); }

        int loadedWith = 0;
        int enteredWith = 0;
    };

    struct Settings
    {
        std::string map;
        int lives = 0;
    };

    Fixture f;
    LevelScene& level = f.scenes.add<LevelScene>("level");
    RecordingScene& menu = f.add("menu");

    f.scenes.switchTo("level", 3);
    f.step(0.02);
    CHECK(level.loadedWith == 3);
    CHECK(level.enteredWith == 3);

    // A new entry replaces the value, and one with no value leaves the scene with none
    f.scenes.switchTo("level", 7, engine::SceneTransition::fade(0.2f));
    f.step(0.15);
    f.step(0.15);
    CHECK(level.enteredWith == 7);
    CHECK(level.loadedWith == 3);
    f.scenes.switchTo("level");
    f.step(0.02);
    CHECK_FALSE(level.hasParameter());
    CHECK(level.enteredWith == -1);

    // Any type works; a string literal reads back as std::string, and the wrong type gives the fallback
    f.scenes.push("menu", "from level");
    f.step(0.02);
    CHECK(menu.getParameter(std::string("none")) == "from level");
    CHECK(menu.getParameter(0) == 0);
    f.scenes.pop();
    f.scenes.switchTo("menu", Settings{ "harbour", 3 });
    f.step(0.02);
    REQUIRE(menu.findParameter<Settings>() != nullptr);
    CHECK(menu.findParameter<Settings>()->map == "harbour");
    CHECK(menu.findParameter<int>() == nullptr);
}

TEST_CASE("A scene resumed after a pop keeps the value it was entered with")
{
    Fixture f;
    RecordingScene& level = f.add("level");
    f.add("pause", true);
    f.scenes.switchTo("level", 2.5);
    f.step(0.02);
    f.scenes.push("pause");
    f.step(0.02);
    f.scenes.pop();
    f.step(0.02);
    CHECK(level.getParameter(0.0) == doctest::Approx(2.5));
}
