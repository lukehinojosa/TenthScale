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

        bool canAttachRadial(PartIndex parent, const PartDefinition& definition) const;

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
