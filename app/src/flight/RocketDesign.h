#pragma once

#include <vector>

#include "flight/PartDefinition.h"
#include "shared/Vec3d.h"

namespace tenth
{
    using PartIndex = int;
    constexpr PartIndex NO_PART = -1;
    constexpr int NO_STAGE = -1;

    enum class Node
    {
        Top,
        Bottom,
    };

    enum class Attachment
    {
        Root,
        Stack, // on one of the parent's top or bottom nodes
        Radial, // flat against the parent's side
    };

    // One part placed in a design. Positions are part centres in metres, with the root part at the origin and +Y up.
    struct PlacedPart
    {
        const PartDefinition* pDefinition = nullptr;
        PartIndex parent = NO_PART;
        Attachment attachment = Attachment::Root;
        Node parentNode = Node::Bottom; // stack parts: which of the parent's nodes it's on
        double angle = 0.0; // radial parts: radians around the parent's axis, 0 facing +Z
        Vec3d position;
        PartShape shape; // the definition's, or resized to fit what it sits on, as nose cones are
        int symmetryGroup = 0; // copies placed together in symmetry share a number; 0 for a part on its own
        int stage = NO_STAGE; // stageable parts: which press of the staging key fires it, 0 first

        // Resized parts weigh more or less with their surface, as nose cones do
        double dryMass() const;
        double wetMass() const { return dryMass() + pDefinition->fuelMass; }
    };

    // A rocket as built on the build screen: a tree of parts hanging off one root. A parent always comes before its
    // children, so walking the list in order visits the tree from the root out.
    class RocketDesign
    {
    public:
        void clear();
        bool empty() const { return mParts.empty(); }
        int partCount() const { return (int)mParts.size(); }
        const PlacedPart& part(PartIndex index) const { return mParts.at(index); }

        // Starts a new design with this part alone at the origin
        PartIndex setRoot(const PartDefinition& definition);

        bool canAttachStack(PartIndex parent, Node parentNode, const PartDefinition& definition) const;

        // NO_PART if it can't go there
        PartIndex attachStack(PartIndex parent, Node parentNode, const PartDefinition& definition);

        // The part's shape as it would be on a stack node, resized there if it fits its parent's diameter
        PartShape shapeOnNode(PartIndex parent, Node parentNode, const PartDefinition& definition) const;

        // Where a part's centre would go on a stack node
        Vec3d stackPosition(PartIndex parent, Node parentNode, const PartDefinition& definition) const;

        // A whole subassembly on a stack node: its root must have the matching node free
        bool canAttachDesign(PartIndex parent, Node parentNode, const RocketDesign& subassembly) const;

        // Returns where the subassembly's root ended up, or NO_PART if it can't go there
        PartIndex attachDesign(PartIndex parent, Node parentNode, const RocketDesign& subassembly);

        // Takes the part and everything attached through it out as its own design, with that part as the root at the
        // origin. Indices held from before are stale afterwards, as with remove().
        RocketDesign extract(PartIndex index);

        bool canAttachRadial(PartIndex parent, const PartDefinition& definition) const;

        // Where a part's centre would go against the parent's side; height is kept within the parent's height
        Vec3d radialPosition(PartIndex parent, double angle, double height, const PartDefinition& definition) const;

        bool canAttachDesignRadial(PartIndex parent, const RocketDesign& subassembly) const;

        // Places copies of the subassembly evenly around the parent, starting at angle, each turned to face outward.
        // Returns where each copy's root ended up, or nothing if it can't go there.
        std::vector<PartIndex> attachDesignRadial(PartIndex parent, double angle, double height, const RocketDesign& subassembly,
                                                  int copies);

        // The part and every copy placed in symmetry with it
        std::vector<PartIndex> symmetryGroup(PartIndex index) const;

        // As remove() and extract(), but the part's symmetry copies go too. The extracted copy is no longer grouped.
        int removeWithSymmetry(PartIndex index);
        RocketDesign extractWithSymmetry(PartIndex index);

        // height is measured from the parent's centre and kept within the parent's height. NO_PART if it can't go there.
        PartIndex attachRadial(PartIndex parent, double angle, double height, const PartDefinition& definition);

        // Removes the part and everything attached through it, and returns how many parts went. Later parts move up
        // to fill the gaps, so indices held from before are stale.
        int remove(PartIndex index);

        // Whether nothing is attached to this node yet, including the part's own link to its parent
        bool isNodeFree(PartIndex index, Node node) const;

        std::vector<PartIndex> children(PartIndex index) const;

        double totalMass() const;
        double dryMass() const;

        // With full tanks. The origin for an empty design.
        Vec3d centerOfMass() const;

        // The height of the lowest point of any part, for setting the rocket down on the pad
        double lowestPoint() const;

        // The height of the highest point of any part. 0 for an empty design.
        double highestPoint() const;

        // An engine with something on its bottom node wears a cylinder over its bell, flush with the parts around it
        bool isShrouded(PartIndex index) const;

        // Stages fire in order, stage 0 first. Each stageable part sits in one; a stage can be empty.
        int stageCount() const { return mStageCount; }
        std::vector<PartIndex> partsInStage(int stage) const;

        // Moving parts or adding and removing stages marks the staging as the player's own, so edits keep it
        void moveToStage(const std::vector<PartIndex>& parts, int stage);

        // Puts an empty stage at this index, moving the stages from there on one later
        void insertStage(int stage);

        // Only an empty stage can be removed. Returns whether it was.
        bool removeStage(int stage);

        // Lays the stages out from the bottom of the rocket up: each section's engines, then the decoupler that drops
        // it along with the engines above, and the parachute last
        void autoStage();

        // Gives stageable parts without a stage one. Until the player changes the staging, it's laid out afresh.
        void updateStaging();

        bool customStaging() const { return mCustomStaging; }

        // For loading a saved design
        void setSymmetryGroup(PartIndex index, int group) { mParts.at(index).symmetryGroup = group; }
        void setStage(PartIndex index, int stage) { mParts.at(index).stage = stage; }
        void setStaging(int stageCount, bool custom);

    private:
        // Where autoStage would put each part, and how many stages that makes
        std::vector<int> autoStages(int& stageCount) const;

        std::vector<PlacedPart> mParts;
        int mStageCount = 0;
        bool mCustomStaging = false;
    };
}
