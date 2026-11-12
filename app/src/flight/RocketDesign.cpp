#include "flight/RocketDesign.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

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

    Vec3d RocketDesign::stackPosition(PartIndex parent, Node parentNode, const PartDefinition& definition) const
    {
        const PlacedPart& parentPart = part(parent);
        const double gap = (parentPart.pDefinition->shape.height + definition.shape.height) * 0.5;
        return parentPart.position + Vec3d{ 0.0, parentNode == Node::Top ? gap : -gap, 0.0 };
    }

    PartIndex RocketDesign::attachStack(PartIndex parent, Node parentNode, const PartDefinition& definition)
    {
        if (!canAttachStack(parent, parentNode, definition))
        {
            return NO_PART;
        }
        PlacedPart placed;
        placed.pDefinition = &definition;
        placed.parent = parent;
        placed.attachment = Attachment::Stack;
        placed.parentNode = parentNode;
        placed.position = stackPosition(parent, parentNode, definition);
        mParts.push_back(placed);
        return partCount() - 1;
    }

    bool RocketDesign::canAttachDesign(PartIndex parent, Node parentNode, const RocketDesign& subassembly) const
    {
        return !subassembly.empty() && canAttachStack(parent, parentNode, *subassembly.part(0).pDefinition) &&
               subassembly.isNodeFree(0, opposite(parentNode));
    }

    PartIndex RocketDesign::attachDesign(PartIndex parent, Node parentNode, const RocketDesign& subassembly)
    {
        if (!canAttachDesign(parent, parentNode, subassembly))
        {
            return NO_PART;
        }

        // Every part moves by the same amount, so radial parts and lower stages keep their places around the root
        const PartIndex first = partCount();
        const Vec3d shift = stackPosition(parent, parentNode, *subassembly.part(0).pDefinition) - subassembly.part(0).position;
        for (PartIndex i = 0; i < subassembly.partCount(); i++)
        {
            PlacedPart placed = subassembly.part(i);
            placed.position += shift;
            if (i == 0)
            {
                placed.parent = parent;
                placed.attachment = Attachment::Stack;
                placed.parentNode = parentNode;
            }
            else
            {
                placed.parent += first;
            }
            mParts.push_back(placed);
        }
        return first;
    }

    RocketDesign RocketDesign::extract(PartIndex index)
    {
        RocketDesign subassembly;
        if (index < 0 || index >= partCount())
        {
            return subassembly;
        }

        // Parents come first, so one pass in order collects the branch and can map each parent to its new index
        std::vector<PartIndex> newIndex(mParts.size(), NO_PART);
        const Vec3d origin = mParts[index].position;
        for (PartIndex i = index; i < partCount(); i++)
        {
            const bool inBranch = i == index || (mParts[i].parent != NO_PART && newIndex[mParts[i].parent] != NO_PART);
            if (!inBranch)
            {
                continue;
            }
            PlacedPart placed = mParts[i];
            placed.position -= origin;
            if (i == index)
            {
                placed.parent = NO_PART;
                placed.attachment = Attachment::Root;
            }
            else
            {
                placed.parent = newIndex[placed.parent];
            }
            newIndex[i] = subassembly.partCount();
            subassembly.mParts.push_back(placed);
        }
        remove(index);
        return subassembly;
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

    Vec3d RocketDesign::radialPosition(PartIndex parent, double angle, double height, const PartDefinition& definition) const
    {
        const PlacedPart& parentPart = part(parent);
        const PartShape& parentShape = parentPart.pDefinition->shape;
        const double halfHeight = parentShape.height * 0.5;
        const double clampedHeight = std::clamp(height, -halfHeight, halfHeight);

        // Round parts sit against the surface by their own radius, box parts by half their reach
        const PartShape& shape = definition.shape;
        const double ownDepth = shape.kind == ShapeKind::Box ? shape.span * 0.5 : std::max(shape.topDiameter, shape.bottomDiameter) * 0.5;
        const double out = radiusAt(parentShape, clampedHeight) + ownDepth;
        return parentPart.position + Vec3d{ std::sin(angle) * out, clampedHeight, std::cos(angle) * out };
    }

    PartIndex RocketDesign::attachRadial(PartIndex parent, double angle, double height, const PartDefinition& definition)
    {
        if (!canAttachRadial(parent, definition))
        {
            return NO_PART;
        }
        PlacedPart placed;
        placed.pDefinition = &definition;
        placed.parent = parent;
        placed.attachment = Attachment::Radial;
        placed.angle = angle;
        placed.position = radialPosition(parent, angle, height, definition);
        mParts.push_back(placed);
        return partCount() - 1;
    }

    bool RocketDesign::canAttachDesignRadial(PartIndex parent, const RocketDesign& subassembly) const
    {
        return !subassembly.empty() && canAttachRadial(parent, *subassembly.part(0).pDefinition);
    }

    std::vector<PartIndex> RocketDesign::attachDesignRadial(PartIndex parent, double angle, double height, const RocketDesign& subassembly,
                                                            int copies)
    {
        std::vector<PartIndex> roots;
        if (!canAttachDesignRadial(parent, subassembly) || copies < 1)
        {
            return roots;
        }

        int group = 0;
        if (copies > 1)
        {
            for (const PlacedPart& placed : mParts)
            {
                group = std::max(group, placed.symmetryGroup);
            }
            group++;
        }

        const PlacedPart& subRoot = subassembly.part(0);
        for (int copy = 0; copy < copies; copy++)
        {
            // Each copy turns about the vertical axis, so whatever hangs off the root swings around with it
            const double copyAngle = angle + 2.0 * std::numbers::pi * copy / copies;
            const double turn = copyAngle - subRoot.angle;
            const double c = std::cos(turn);
            const double s = std::sin(turn);
            const Vec3d rootPosition = radialPosition(parent, copyAngle, height, *subRoot.pDefinition);
            const PartIndex first = partCount();
            for (PartIndex i = 0; i < subassembly.partCount(); i++)
            {
                PlacedPart placed = subassembly.part(i);
                const Vec3d local = placed.position - subRoot.position;
                placed.position = rootPosition + Vec3d{ local.x * c + local.z * s, local.y, -local.x * s + local.z * c };
                placed.angle += turn;
                if (i == 0)
                {
                    placed.parent = parent;
                    placed.attachment = Attachment::Radial;
                    placed.angle = copyAngle;
                    placed.symmetryGroup = group;
                }
                else
                {
                    placed.parent += first;
                }
                mParts.push_back(placed);
            }
            roots.push_back(first);
        }
        return roots;
    }

    std::vector<PartIndex> RocketDesign::symmetryGroup(PartIndex index) const
    {
        const int group = part(index).symmetryGroup;
        if (group == 0)
        {
            return { index };
        }
        std::vector<PartIndex> members;
        for (PartIndex i = 0; i < partCount(); i++)
        {
            if (mParts[i].symmetryGroup == group)
            {
                members.push_back(i);
            }
        }
        return members;
    }

    // Removing from the highest index down leaves the lower ones in place, since a part's branch only holds later parts
    int RocketDesign::removeWithSymmetry(PartIndex index)
    {
        if (index < 0 || index >= partCount())
        {
            return 0;
        }
        const std::vector<PartIndex> members = symmetryGroup(index);
        int removed = 0;
        for (auto it = members.rbegin(); it != members.rend(); ++it)
        {
            removed += remove(*it);
        }
        return removed;
    }

    RocketDesign RocketDesign::extractWithSymmetry(PartIndex index)
    {
        if (index < 0 || index >= partCount())
        {
            return {};
        }
        const int group = part(index).symmetryGroup;
        RocketDesign subassembly = extract(index);
        if (group != 0)
        {
            subassembly.mParts[0].symmetryGroup = 0;
            for (PartIndex i = partCount() - 1; i >= 0; i--)
            {
                if (i < partCount() && mParts[i].symmetryGroup == group)
                {
                    remove(i);
                }
            }
        }
        return subassembly;
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
