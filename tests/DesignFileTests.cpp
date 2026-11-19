#include <doctest/doctest.h>

#include <optional>
#include <string>

#include <flight/DesignFile.h>
#include <flight/DesignStats.h>
#include <flight/PartCatalog.h>

using namespace tenth;

namespace
{
    const PartCatalog& catalog()
    {
        static const PartCatalog standard = PartCatalog::standard();
        return standard;
    }

    int countOf(const RocketDesign& design, const char* id)
    {
        int count = 0;
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            if (design.part(i).pDefinition->id == id)
            {
                count++;
            }
        }
        return count;
    }
}

TEST_CASE("The default rocket is built like Saturn V carrying Apollo")
{
    const RocketDesign design = defaultRocket(catalog());
    CHECK(design.part(0).pDefinition->id == "pod");
    CHECK(countOf(design, "nose-cone") == 1);
    CHECK(countOf(design, "engine-heavy") == 1);
    CHECK(countOf(design, "engine-lifter") == 1);
    CHECK(countOf(design, "engine-vacuum") == 2);
    CHECK(countOf(design, "decoupler") == 4);
    CHECK(countOf(design, "landing-leg") == 4);
    CHECK(countOf(design, "fin-large") == 4);

    // Three stages to the Moon, the lander, then the pod dropping the lander and opening its parachute
    REQUIRE(design.stageCount() == 6);
    const std::vector<PartIndex> podRelease = design.partsInStage(4);
    REQUIRE(podRelease.size() == 1);
    CHECK(design.part(podRelease.front()).pDefinition->separates);
    CHECK(design.part(podRelease.front()).parent == 0);
    const std::vector<PartIndex> parachute = design.partsInStage(5);
    REQUIRE(parachute.size() == 1);
    CHECK(design.part(parachute.front()).pDefinition->hasParachute);

    // Enough push to leave the pad briskly, enough speed to reach orbit and head for the Moon, and enough in the
    // lander to set down, lift off again, and come home
    const DesignStats stats = computeStats(design, 9.82);
    CHECK(stats.liftoffTwr > 1.2);
    CHECK(stats.liftoffTwr < 1.6);
    CHECK(stats.stages[0].deltaVVacuum + stats.stages[1].deltaVVacuum + stats.stages[2].deltaVVacuum > 4000.0);
    CHECK(stats.stages[3].deltaVVacuum > 1500.0);
    CHECK(stats.stable);

    // Every engine but the bottom one sits on a decoupler, so only the first stage's bell shows
    for (PartIndex i = 0; i < design.partCount(); i++)
    {
        if (design.part(i).pDefinition->isEngine())
        {
            CHECK(design.isShrouded(i) == (design.part(i).pDefinition->id != "engine-heavy"));
        }
    }
}

TEST_CASE("A saved design reads back the same, staging and symmetry included")
{
    RocketDesign design = defaultRocket(catalog());
    design.insertStage(0);
    const std::string text = writeDesign(design);

    const std::optional<RocketDesign> read = readDesign(text, catalog());
    REQUIRE(read.has_value());
    REQUIRE(read->partCount() == design.partCount());
    CHECK(read->stageCount() == design.stageCount());
    CHECK(read->customStaging());
    for (PartIndex i = 0; i < design.partCount(); i++)
    {
        const PlacedPart& a = design.part(i);
        const PlacedPart& b = read->part(i);
        CHECK(a.pDefinition == b.pDefinition);
        CHECK(a.parent == b.parent);
        CHECK(a.stage == b.stage);
        CHECK(a.symmetryGroup == b.symmetryGroup);
        CHECK(a.position.x == doctest::Approx(b.position.x));
        CHECK(a.position.y == doctest::Approx(b.position.y));
        CHECK(a.position.z == doctest::Approx(b.position.z));
    }
    CHECK(writeDesign(*read) == text);
}

TEST_CASE("Reading rejects text that isn't a design or names unknown parts")
{
    CHECK_FALSE(readDesign("", catalog()).has_value());
    CHECK_FALSE(readDesign("something else 1\n", catalog()).has_value());
    CHECK_FALSE(readDesign("tenth-rocket 1\nstages 0 0\npart warp-drive root 0 -1\n", catalog()).has_value());

    // A part can't hang off a parent that comes after it
    CHECK_FALSE(readDesign("tenth-rocket 1\nstages 0 0\npart pod root 0 -1\npart tank-small stack 5 bottom 0 -1\n", catalog()).has_value());

    const std::optional<RocketDesign> empty = readDesign("tenth-rocket 1\nstages 0 0\n", catalog());
    REQUIRE(empty.has_value());
    CHECK(empty->empty());
}

TEST_CASE("Rocket names keep only characters safe in a file name")
{
    CHECK(cleanRocketName("  Moon Lander 2 ") == "Moon Lander 2");
    CHECK(cleanRocketName("../up/and_away!") == "upand_away");
    CHECK(cleanRocketName("???").empty());
}
