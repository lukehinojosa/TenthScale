#pragma once

#include <vector>

#include "flight/PartDefinition.h"
#include "shared/Vec3d.h"

namespace tenth
{
    using PartIndex = int;
    constexpr PartIndex NO_PART = -1;

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
        int symmetryGroup = 0; // copies placed together in symmetry share a number; 0 for a part on its own
    };

    // A rocket as built on the build screen: a tree of parts hanging off one root. A parent always comes before its
    // children, so walking the list in order visits the tree from the root out.
    class RocketDesign
    {
    public:
        void clear() { mParts.clear(); }
        bool empty() const { return mParts.empty(); }
        int partCount() const { return (int)mParts.size(); }
        const PlacedPart& part(PartIndex index) const { return mParts.at(index); }

        // Starts a new design with this part alone at the origin
        PartIndex setRoot(const PartDefinition& definition);

        bool canAttachStack(PartIndex parent, Node parentNode, const PartDefinition& definition) const;

        // NO_PART if it can't go there
        PartIndex attachStack(PartIndex parent, Node parentNode, const PartDefinition& definition);

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

    private:
        std::vector<PlacedPart> mParts;
    };
}
