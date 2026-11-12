#include <doctest/doctest.h>

#include <algorithm>
#include <cstdlib>

#include <engine/Animation.h>
#include <engine/Audio.h>
#include <engine/Unit.h>

namespace
{
    // Sprites without a buffer: enough for timing and hit tests, no GPU needed
    engine::Animation makeAnimation(int frames, bool loop, double fps, float size = 10.0f)
    {
        engine::Animation animation(loop, fps);
        for (int i = 0; i < frames; i++)
        {
            animation.addSprite(engine::Sprite(nullptr, Rectangle{ i * size, 0.0f, size, size }));
        }
        return animation;
    }
}

TEST_CASE("Animation advances at its frame rate and loops")
{
    engine::Animation animation = makeAnimation(4, true, 10.0);
    animation.update(0.05);
    CHECK(animation.getCurrentFrame() == 0);
    animation.update(0.06);
    CHECK(animation.getCurrentFrame() == 1);
    animation.update(0.3);
    CHECK(animation.getCurrentFrame() == 0); // Frames 2, 3, then back to 0

    animation.reset();
    CHECK(animation.getCurrentFrame() == 0);
}

TEST_CASE("A non-looping animation holds its last frame")
{
    engine::Animation animation = makeAnimation(3, false, 10.0);
    animation.update(5.0);
    CHECK(animation.getCurrentFrame() == 2);
    CHECK(animation.isFinished());
    CHECK(animation.getCurrentSprite().getSource().x == doctest::Approx(20.0f));
}

TEST_CASE("An empty animation is safe to update and draw from")
{
    engine::Animation animation;
    animation.update(1.0);
    CHECK(animation.getFrameCount() == 0);
    CHECK(animation.getCurrentSprite().getBuffer() == nullptr);
}

TEST_CASE("UnitManager hit-tests the top unit and toggles animations")
{
    engine::UnitManager units;
    engine::Animation primary = makeAnimation(2, true, 10.0, 20.0f);
    engine::Animation alternate = makeAnimation(2, true, 10.0, 40.0f);

    engine::Unit* pFirst = units.addUnit({ 100, 100 }, primary, alternate);
    engine::Unit* pSecond = units.addUnit({ 105, 100 }, primary, alternate);
    CHECK(pFirst->getID() != pSecond->getID());

    CHECK(units.getUnitAt({ 104, 100 }) == pSecond); // Overlap: the newer unit is on top
    CHECK(units.getUnitAt({ 91, 100 }) == pFirst);
    CHECK(units.getUnitAt({ 200, 200 }) == nullptr);

    pFirst->toggleAnimation();
    CHECK(pFirst->isShowingAlternate());
    CHECK(pFirst->containsPoint({ 81, 100 })); // The alternate frames are 40 wide

    units.setAllActive(false);
    pSecond->velocity = { 50.0f, 0.0f };
    units.update(1.0);
    CHECK(pSecond->getPosition().x == doctest::Approx(105.0f)); // Frozen

    CHECK(units.deleteUnit(pFirst->getID()));
    CHECK_FALSE(units.deleteUnit(999));
    CHECK(units.getUnitCount() == 1);
}

TEST_CASE("makeTone builds a mono 16-bit wave of the requested length")
{
    engine::ToneSettings settings;
    settings.seconds = 0.5f;
    settings.sampleRate = 8000;
    settings.volume = 1.0f;
    Wave wave = engine::makeTone(settings);

    REQUIRE(wave.data != nullptr);
    CHECK(wave.frameCount == 4000u);
    CHECK(wave.channels == 1u);
    CHECK(wave.sampleSize == 16u);

    // Fades in from silence and out toward it, so playback never clicks
    const auto* pSamples = static_cast<const short*>(wave.data);
    CHECK(pSamples[0] == 0);
    short loudest = 0;
    for (unsigned int i = 0; i < wave.frameCount; i++)
    {
        loudest = std::max<short>(loudest, (short)std::abs(pSamples[i]));
    }
    CHECK(loudest > 20000);
    CHECK(std::abs(pSamples[wave.frameCount - 1]) < 100);
    UnloadWave(wave);

    settings.seconds = 0.0f;
    CHECK(engine::makeTone(settings).data == nullptr);
}

TEST_CASE("AudioManager without a device ignores every call")
{
    engine::AudioManager audio;
    CHECK_FALSE(audio.isReady());
    CHECK_FALSE(audio.addSound("blip", Wave{}));
    audio.playSound("blip");
    audio.playMusic("missing");
    audio.update();
    CHECK(audio.getCurrentMusic().empty());
}
