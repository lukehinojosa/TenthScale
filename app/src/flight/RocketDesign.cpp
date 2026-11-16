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

    double PlacedPart::dryMass() const
    {
        const double baseHeight = pDefinition->shape.height;
        if (!pDefinition->fitsParentDiameter || baseHeight <= 0.0)
        {
            return pDefinition->dryMass;
        }
        const double scale = shape.height / baseHeight;
        return pDefinition->dryMass * scale * scale;
    }

    void RocketDesign::clear()
    {
        mParts.clear();
        mStageCount = 0;
        mCustomStaging = false;
    }

    PartIndex RocketDesign::setRoot(const PartDefinition& definition)
    {
        clear();
        PlacedPart root;
        root.pDefinition = &definition;
        root.shape = definition.shape;
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

    PartShape RocketDesign::shapeOnNode(PartIndex parent, Node parentNode, const PartDefinition& definition) const
    {
        const PartShape& parentShape = part(parent).shape;
        const double nodeDiameter = parentNode == Node::Top ? parentShape.topDiameter : parentShape.bottomDiameter;
        const double ownDiameter = std::max(definition.shape.topDiameter, definition.shape.bottomDiameter);
        if (!definition.fitsParentDiameter || nodeDiameter <= 0.0 || ownDiameter <= 0.0)
        {
            return definition.shape;
        }
        const double scale = nodeDiameter / ownDiameter;
        PartShape shape = definition.shape;
        shape.height *= scale;
        shape.topDiameter *= scale;
        shape.bottomDiameter *= scale;
        shape.span *= scale;
        shape.thickness *= scale;
        return shape;
    }

    Vec3d RocketDesign::stackPosition(PartIndex parent, Node parentNode, const PartDefinition& definition) const
    {
        const PlacedPart& parentPart = part(parent);
        const double gap = (parentPart.shape.height + shapeOnNode(parent, parentNode, definition).height) * 0.5;
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
        placed.shape = shapeOnNode(parent, parentNode, definition);
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
                placed.shape = shapeOnNode(parent, parentNode, *placed.pDefinition);
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
        return definition.mountsRadially && parentDefinition.acceptsRadial && part(parent).shape.kind == ShapeKind::Round;
    }

    Vec3d RocketDesign::radialPosition(PartIndex parent, double angle, double height, const PartDefinition& definition) const
    {
        const PlacedPart& parentPart = part(parent);
        const PartShape& parentShape = parentPart.shape;
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
        placed.shape = definition.shape;
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
            mass += placed.wetMass();
        }
        return mass;
    }

    double RocketDesign::dryMass() const
    {
        double mass = 0.0;
        for (const PlacedPart& placed : mParts)
        {
            mass += placed.dryMass();
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
            weighted += placed.position * placed.wetMass();
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
            lowest = std::min(lowest, placed.position.y - placed.shape.height * 0.5);
        }
        return lowest;
    }

    double RocketDesign::highestPoint() const
    {
        if (mParts.empty())
        {
            return 0.0;
        }
        double highest = -std::numeric_limits<double>::infinity();
        for (const PlacedPart& placed : mParts)
        {
            highest = std::max(highest, placed.position.y + placed.shape.height * 0.5);
        }
        return highest;
    }

    bool RocketDesign::isShrouded(PartIndex index) const
    {
        return part(index).pDefinition->isEngine() && !isNodeFree(index, Node::Bottom);
    }

    std::vector<PartIndex> RocketDesign::partsInStage(int stage) const
    {
        std::vector<PartIndex> result;
        for (PartIndex i = 0; i < partCount(); i++)
        {
            if (mParts[i].pDefinition->isStageable() && mParts[i].stage == stage)
            {
                result.push_back(i);
            }
        }
        return result;
    }

    void RocketDesign::moveToStage(const std::vector<PartIndex>& parts, int stage)
    {
        if (stage < 0 || stage >= mStageCount)
        {
            return;
        }
        for (PartIndex index : parts)
        {
            if (index >= 0 && index < partCount() && mParts[index].pDefinition->isStageable())
            {
                mParts[index].stage = stage;
            }
        }
        mCustomStaging = true;
    }

    void RocketDesign::insertStage(int stage)
    {
        stage = std::clamp(stage, 0, mStageCount);
        for (PlacedPart& placed : mParts)
        {
            if (placed.stage >= stage)
            {
                placed.stage++;
            }
        }
        mStageCount++;
        mCustomStaging = true;
    }

    bool RocketDesign::removeStage(int stage)
    {
        if (stage < 0 || stage >= mStageCount || !partsInStage(stage).empty())
        {
            return false;
        }
        for (PlacedPart& placed : mParts)
        {
            if (placed.stage > stage)
            {
                placed.stage--;
            }
        }
        mStageCount--;
        mCustomStaging = true;
        return true;
    }

    std::vector<int> RocketDesign::autoStages(int& stageCount) const
    {
        // Parents come before children, so each part's count of decouplers builds on its parent's
        std::vector<int> depths(mParts.size(), 0);
        int deepest = 0;
        for (PartIndex i = 0; i < partCount(); i++)
        {
            const PlacedPart& placed = mParts[i];
            const int parentDepth = placed.parent == NO_PART ? 0 : depths[placed.parent];
            depths[i] = parentDepth + (placed.pDefinition->separates ? 1 : 0);
            deepest = std::max(deepest, depths[i]);
        }

        // The deepest section burns first; its decoupler fires together with the engines of the section above
        std::vector<int> stages(mParts.size(), NO_STAGE);
        for (PartIndex i = 0; i < partCount(); i++)
        {
            const PartDefinition& definition = *mParts[i].pDefinition;
            if (definition.separates)
            {
                stages[i] = deepest - depths[i] + 1;
            }
            else if (definition.isEngine())
            {
                stages[i] = deepest - depths[i];
            }
            else if (definition.hasParachute)
            {
                stages[i] = deepest + 2;
            }
        }

        // Stages nothing landed in are dropped, keeping the rest in order
        std::vector<int> renumber(deepest + 3, NO_STAGE);
        for (int stage : stages)
        {
            if (stage != NO_STAGE)
            {
                renumber[stage] = 0;
            }
        }
        stageCount = 0;
        for (int& stage : renumber)
        {
            if (stage == 0)
            {
                stage = stageCount++;
            }
        }
        for (int& stage : stages)
        {
            if (stage != NO_STAGE)
            {
                stage = renumber[stage];
            }
        }
        return stages;
    }

    void RocketDesign::autoStage()
    {
        const std::vector<int> stages = autoStages(mStageCount);
        for (PartIndex i = 0; i < partCount(); i++)
        {
            mParts[i].stage = stages[i];
        }
        mCustomStaging = false;
    }

    void RocketDesign::updateStaging()
    {
        if (mParts.empty())
        {
            mStageCount = 0;
            mCustomStaging = false;
            return;
        }
        if (!mCustomStaging)
        {
            autoStage();
            return;
        }

        // New parts go where autoStage would have put them, within the stages the player already has
        int autoCount = 0;
        const std::vector<int> stages = autoStages(autoCount);
        for (PartIndex i = 0; i < partCount(); i++)
        {
            PlacedPart& placed = mParts[i];
            const bool unstaged = placed.stage < 0 || placed.stage >= mStageCount;
            if (!placed.pDefinition->isStageable() || !unstaged)
            {
                continue;
            }
            if (mStageCount == 0)
            {
                mStageCount = 1;
            }
            placed.stage = std::min(stages[i], mStageCount - 1);
        }
    }

    void RocketDesign::setStaging(int stageCount, bool custom)
    {
        mStageCount = std::max(stageCount, 0);
        mCustomStaging = custom;
    }
}
