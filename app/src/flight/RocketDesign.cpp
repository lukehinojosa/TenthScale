#include "flight/RocketDesign.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tenth
{
    namespace
    {
        bool hasNode(const PartDefinition& definition, Node node)
        {
            return node == Node::Top ? definition.hasTopNode : definition.hasBottomNode;
        }

        Node opposite(Node node)
        {
            return node == Node::Top ? Node::Bottom : Node::Top;
        }

        // The radius of a round part at a height measured from its centre
        double radiusAt(const PartShape& shape, double height)
        {
            const double t = std::clamp(height / shape.height + 0.5, 0.0, 1.0);
            return (shape.bottomDiameter + (shape.topDiameter - shape.bottomDiameter) * t) * 0.5;
        }
    }

    PartIndex RocketDesign::setRoot(const PartDefinition& definition)
    {
        mParts.clear();
        PlacedPart root;
        root.pDefinition = &definition;
        mParts.push_back(root);
        return 0;
    }

    bool RocketDesign::isNodeFree(PartIndex index, Node node) const
    {
        const PlacedPart& placed = part(index);
        if (placed.attachment == Attachment::Stack && opposite(placed.parentNode) == node)
        {
            return false;
        }
        for (const PlacedPart& other : mParts)
        {
            if (other.parent == index && other.attachment == Attachment::Stack && other.parentNode == node)
            {
                return false;
            }
        }
        return true;
    }

    bool RocketDesign::canAttachStack(PartIndex parent, Node parentNode, const PartDefinition& definition) const
    {
        if (parent < 0 || parent >= partCount())
        {
            return false;
        }
        const PartDefinition& parentDefinition = *part(parent).pDefinition;
        return hasNode(parentDefinition, parentNode) && hasNode(definition, opposite(parentNode)) && isNodeFree(parent, parentNode);
    }

    PartIndex RocketDesign::attachStack(PartIndex parent, Node parentNode, const PartDefinition& definition)
    {
        if (!canAttachStack(parent, parentNode, definition))
        {
            return NO_PART;
        }
        const PlacedPart& parentPart = part(parent);
        const double gap = (parentPart.pDefinition->shape.height + definition.shape.height) * 0.5;

        PlacedPart placed;
        placed.pDefinition = &definition;
        placed.parent = parent;
        placed.attachment = Attachment::Stack;
        placed.parentNode = parentNode;
        placed.position = parentPart.position + Vec3d{ 0.0, parentNode == Node::Top ? gap : -gap, 0.0 };
        mParts.push_back(placed);
        return partCount() - 1;
    }

    bool RocketDesign::canAttachRadial(PartIndex parent, const PartDefinition& definition) const
    {
        if (parent < 0 || parent >= partCount())
        {
            return false;
        }
        const PartDefinition& parentDefinition = *part(parent).pDefinition;
        return definition.mountsRadially && parentDefinition.acceptsRadial && parentDefinition.shape.kind == ShapeKind::Round;
    }

    PartIndex RocketDesign::attachRadial(PartIndex parent, double angle, double height, const PartDefinition& definition)
    {
        if (!canAttachRadial(parent, definition))
        {
            return NO_PART;
        }
        const PlacedPart& parentPart = part(parent);
        const PartShape& parentShape = parentPart.pDefinition->shape;
        const double halfHeight = parentShape.height * 0.5;
        const double clampedHeight = std::clamp(height, -halfHeight, halfHeight);

        // Round parts sit against the surface by their own radius, box parts by half their reach
        const PartShape& shape = definition.shape;
        const double ownDepth = shape.kind == ShapeKind::Box ? shape.span * 0.5 : std::max(shape.topDiameter, shape.bottomDiameter) * 0.5;
        const double out = radiusAt(parentShape, clampedHeight) + ownDepth;

        PlacedPart placed;
        placed.pDefinition = &definition;
        placed.parent = parent;
        placed.attachment = Attachment::Radial;
        placed.angle = angle;
        placed.position = parentPart.position + Vec3d{ std::sin(angle) * out, clampedHeight, std::cos(angle) * out };
        mParts.push_back(placed);
        return partCount() - 1;
    }

    int RocketDesign::remove(PartIndex index)
    {
        if (index < 0 || index >= partCount())
        {
            return 0;
        }

        // Parents come first, so one pass in order catches every descendant
        std::vector<bool> removed(mParts.size(), false);
        for (PartIndex i = index; i < partCount(); i++)
        {
            removed[i] = i == index || (mParts[i].parent != NO_PART && removed[mParts[i].parent]);
        }

        std::vector<PartIndex> newIndex(mParts.size(), NO_PART);
        std::vector<PlacedPart> kept;
        for (PartIndex i = 0; i < partCount(); i++)
        {
            if (removed[i])
            {
                continue;
            }
            newIndex[i] = (PartIndex)kept.size();
            kept.push_back(mParts[i]);
            if (kept.back().parent != NO_PART)
            {
                kept.back().parent = newIndex[kept.back().parent];
            }
        }
        const int removedCount = partCount() - (int)kept.size();
        mParts = std::move(kept);
        return removedCount;
    }

    std::vector<PartIndex> RocketDesign::children(PartIndex index) const
    {
        std::vector<PartIndex> result;
        for (PartIndex i = 0; i < partCount(); i++)
        {
            if (mParts[i].parent == index)
            {
                result.push_back(i);
            }
        }
        return result;
    }

    double RocketDesign::totalMass() const
    {
        double mass = 0.0;
        for (const PlacedPart& placed : mParts)
        {
            mass += placed.pDefinition->wetMass();
        }
        return mass;
    }

    double RocketDesign::dryMass() const
    {
        double mass = 0.0;
        for (const PlacedPart& placed : mParts)
        {
            mass += placed.pDefinition->dryMass;
        }
        return mass;
    }

    Vec3d RocketDesign::centerOfMass() const
    {
        const double mass = totalMass();
        if (mass <= 0.0)
        {
            return {};
        }
        Vec3d weighted;
        for (const PlacedPart& placed : mParts)
        {
            weighted += placed.position * placed.pDefinition->wetMass();
        }
        return weighted / mass;
    }

    double RocketDesign::lowestPoint() const
    {
        if (mParts.empty())
        {
            return 0.0;
        }
        double lowest = std::numeric_limits<double>::infinity();
        for (const PlacedPart& placed : mParts)
        {
            lowest = std::min(lowest, placed.position.y - placed.pDefinition->shape.height * 0.5);
        }
        return lowest;
    }
}
