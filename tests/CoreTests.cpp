#include <doctest/doctest.h>

#include <core/CallbackManager.h>
#include <core/ComponentPool.h>
#include <core/EventListener.h>
#include <core/EventSystem.h>
#include <core/MemoryManager.h>
#include <core/MemoryTracker.h>

namespace
{
    struct TrackedThing : public core::Trackable
    {
        int value = 0;
    };

    enum TestEventType : core::EventType
    {
        PING_EVENT = core::NUM_BASE_EVENT_TYPES,
        OTHER_EVENT
    };

    struct CountingListener : public core::EventListener
    {
        int count = 0;
        void handleEvent(const core::Event&) override
        {
            count++;
        }
    };

    struct EventSystemFixture
    {
        EventSystemFixture()
        {
            core::EventSystem::createInstance();
            core::EventSystem::getInstance()->init();
        }

        ~EventSystemFixture()
        {
            core::EventSystem::destroyInstance();
        }
    };

    struct CounterComponent : public core::Component
    {
        int updates = 0;
        void update(double) override
        {
            updates++;
        }
    };
}

TEST_CASE("Trackable allocations are recorded and released")
{
    auto& tracker = core::MemoryTracker::getInstance();
    std::size_t before = tracker.getNumAllocations();

    TrackedThing* pThing = new TrackedThing;
    CHECK(tracker.getNumAllocations() == before + 1);

    delete pThing;
    CHECK(tracker.getNumAllocations() == before);
}

TEST_CASE("MemoryPool hands out every slot once, then nullptr")
{
    core::MemoryPool pool(3, 24);
    CHECK(pool.getMaxObjectSize() == 32); // Rounded up to a power of two

    core::Byte* a = pool.allocateObject();
    core::Byte* b = pool.allocateObject();
    core::Byte* c = pool.allocateObject();
    CHECK(a != nullptr);
    CHECK(b != nullptr);
    CHECK(c != nullptr);
    CHECK(pool.allocateObject() == nullptr);

    pool.freeObject(b);
    CHECK(pool.getNumFreeObjects() == 1);
    CHECK(pool.allocateObject() == b);
}

TEST_CASE("MemoryManager picks the smallest pool that fits")
{
    core::MemoryManager manager;
    manager.init({ 64, 16 }, { 2, 2 });

    core::Byte* small = manager.allocate(10);
    core::Byte* large = manager.allocate(40);
    REQUIRE(small != nullptr);
    REQUIRE(large != nullptr);
    CHECK(manager.getTotalRequested() == 50);
    CHECK(manager.getTotalAllocated() == 16 + 64);
    CHECK(manager.getTotalWaste() == 30);

    CHECK(manager.allocate(1000) == nullptr);
    CHECK(manager.getNumFailedAllocations() == 1);

    manager.deallocate(small);
    manager.deallocate(large);
    CHECK(manager.getNumAllocations() == 0);
}

TEST_CASE("MemoryManager create and destroy run constructors and destructors")
{
    core::MemoryManager manager;
    manager.init({ 64 }, { 1 });

    TrackedThing* pThing = manager.create<TrackedThing>();
    REQUIRE(pThing != nullptr);
    pThing->value = 7;
    CHECK(manager.create<TrackedThing>() == nullptr);

    manager.destroy(pThing);
    CHECK(manager.create<TrackedThing>() != nullptr);
}

TEST_CASE_FIXTURE(EventSystemFixture, "EventSystem delivers only to listeners of that type")
{
    CountingListener pingListener;
    CountingListener otherListener;
    auto* pEvents = core::EventSystem::getInstance();
    pEvents->addListener(PING_EVENT, &pingListener);
    pEvents->addListener(OTHER_EVENT, &otherListener);

    pEvents->fireEvent(core::Event(PING_EVENT));
    pEvents->fireEvent(core::Event(PING_EVENT));
    CHECK(pingListener.count == 2);
    CHECK(otherListener.count == 0);
}

TEST_CASE_FIXTURE(EventSystemFixture, "A destroyed listener unregisters itself")
{
    auto* pEvents = core::EventSystem::getInstance();
    {
        CountingListener listener;
        pEvents->addListener(PING_EVENT, &listener);
        CHECK(pEvents->getNumListeners() == 1);
    }
    CHECK(pEvents->getNumListeners() == 0);
    pEvents->fireEvent(core::Event(PING_EVENT)); // Must not touch the dead listener
}

TEST_CASE("CallbackManager fires timed callbacks once, or repeatedly")
{
    core::CallbackManager callbacks;
    int once = 0;
    int repeated = 0;
    callbacks.addTimed(1.0, [&]() { once++; });
    callbacks.addTimed(0.5, [&]() { repeated++; }, true);

    callbacks.update(0.6);
    CHECK(once == 0);
    CHECK(repeated == 1);

    callbacks.update(0.6);
    CHECK(once == 1);
    CHECK(repeated == 2);
    CHECK(callbacks.getNumCallbacks() == 1); // The one-shot is gone
}

TEST_CASE("CallbackManager conditional callbacks and cancel")
{
    core::CallbackManager callbacks;
    bool ready = false;
    int fired = 0;
    callbacks.addConditional([&]() { return ready; }, [&]() { fired++; });
    core::CallbackID cancelled = callbacks.addTimed(0.1, [&]() { fired += 100; });

    CHECK(callbacks.cancel(cancelled));
    callbacks.update(1.0);
    CHECK(fired == 0);

    ready = true;
    callbacks.update(0.0);
    callbacks.update(0.0);
    CHECK(fired == 1);
}

TEST_CASE("ComponentPool reuses slots and updates only live components")
{
    core::Entity owner(1);
    core::ComponentPool<CounterComponent> pool;
    pool.init(2);

    CounterComponent* a = pool.acquire(&owner);
    CounterComponent* b = pool.acquire(&owner);
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    CHECK(a->getOwner() == &owner);
    CHECK(pool.acquire(&owner) == nullptr);

    pool.release(b);
    pool.updateAll(0.016);
    CHECK(a->updates == 1);
    CHECK(pool.getNumInUse() == 1);
}

#ifdef TEMPLATE_HAS_CONFIG
#include <core/Config.h>

#include <filesystem>
#include <fstream>

TEST_CASE("Config reads dotted keys and falls back on anything missing or malformed")
{
    std::filesystem::path path = std::filesystem::temp_directory_path() / "cpptemplate_test.yaml";
    {
        std::ofstream file(path);
        file << "physics:\n  fixed_hz: 120\n  gravity: not-a-number\nname: \"Tenth Scale\"\n";
    }

    core::Config config;
    REQUIRE(config.load(path.string()));
    CHECK(config.get("physics.fixed_hz", 50) == 120);
    CHECK(config.get("physics.gravity", 9.81) == doctest::Approx(9.81));
    CHECK(config.get("physics.missing", 7) == 7);
    CHECK(config.get<std::string>("name", "") == "Tenth Scale");
    CHECK(config.get("name.too.deep", 3) == 3);

    core::Config missing;
    CHECK_FALSE(missing.load((std::filesystem::temp_directory_path() / "does_not_exist.yaml").string()));
    CHECK(missing.get("anything", 1.5f) == doctest::Approx(1.5f));
    std::filesystem::remove(path);
}
#endif
