#pragma once

#include <optional>
#include <string>
#include <vector>

#include "flight/PartCatalog.h"
#include "flight/RocketDesign.h"

namespace tenth
{
    // A design as text, one line per part with its stage, so it can be saved and read back
    std::string writeDesign(const RocketDesign& design);

    // Nothing if the text isn't a design, or names a part the catalog doesn't have
    std::optional<RocketDesign> readDesign(const std::string& text, const PartCatalog& catalog);

    // Nose cone, pod, tank, and a vacuum engine on four legs, over a decoupler, two tanks, and a finned lifter
    RocketDesign defaultRocket(const PartCatalog& catalog);

    // Saved rockets live in a rockets folder next to the game; in the browser they're kept in its storage
    void initRocketStorage();
    std::vector<std::string> savedRocketNames();
    bool saveRocket(const std::string& name, const RocketDesign& design);
    std::optional<RocketDesign> loadRocket(const std::string& name, const PartCatalog& catalog);

    // Keeps letters, digits, spaces, dashes, and underscores, so a name is always a safe file name
    std::string cleanRocketName(const std::string& name);
}
