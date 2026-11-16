#pragma once

#include <string_view>
#include <vector>

#include "flight/PartDefinition.h"

namespace tenth
{
    // Every part the build screen offers. Designs point at these definitions, so the catalog must outlive them.
    class PartCatalog
    {
    public:
        // The parts the game ships with
        static PartCatalog standard();

        void add(PartDefinition definition);

        // nullptr if no part has that id
        const PartDefinition* find(std::string_view id) const;

        const std::vector<PartDefinition>& parts() const { return mParts; }

    private:
        std::vector<PartDefinition> mParts;
    };
}
