#include "broadPhase.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

using namespace cyclone;

namespace
{
    bool spheresOverlap(const Vector3& ca, real ra, const Vector3& cb, real rb)
    {
        const real reach = ra + rb;
        return (ca - cb).squareMagnitude() <= reach * reach;
    }

    PotentialContact makePair(int a, int b)
    {
        return a < b ? PotentialContact{ a, b } : PotentialContact{ b, a };
    }

    std::uint64_t pairKey(int a, int b)
    {
        if (a > b)
        {
            std::swap(a, b);
        }
        return ((std::uint64_t)(std::uint32_t)a << 32) | (std::uint32_t)b;
    }

    real axisOf(const Vector3& v, int axis)
    {
        return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
    }
}

BoundingSphere::BoundingSphere(const BoundingSphere& one, const BoundingSphere& two)
{
    const Vector3 centreOffset = two.center - one.center;
    real distance = centreOffset.squareMagnitude();
    const real radiusDiff = two.radius - one.radius;

    // One already holds the other
    if (radiusDiff * radiusDiff >= distance)
    {
        if (one.radius > two.radius)
        {
            center = one.center;
            radius = one.radius;
        }
        else
        {
            center = two.center;
            radius = two.radius;
        }
        return;
    }

    distance = real_sqrt(distance);
    radius = (distance + one.radius + two.radius) * (real)0.5;
    center = one.center;
    if (distance > 0)
    {
        center += centreOffset * ((radius - one.radius) / distance);
    }
}

bool BoundingSphere::overlaps(const BoundingSphere& other) const
{
    return spheresOverlap(center, radius, other.center, other.radius);
}

real BoundingSphere::getGrowth(const BoundingSphere& other) const
{
    const BoundingSphere combined(*this, other);
    return combined.radius * combined.radius - radius * radius;
}

real BoundingSphere::getSize() const
{
    return (real)1.333333 * (real)R_PI * radius * radius * radius;
}

int BoundingVolumeHierarchy::allocNode()
{
    if (!freeNodes.empty())
    {
        const int index = freeNodes.back();
        freeNodes.pop_back();
        nodes[index] = Node();
        return index;
    }
    nodes.emplace_back();
    return (int)nodes.size() - 1;
}

void BoundingVolumeHierarchy::freeNode(int index)
{
    freeNodes.push_back(index);
}

void BoundingVolumeHierarchy::refit(int index)
{
    while (index != NONE)
    {
        Node& node = nodes[index];
        if (!node.isLeaf())
        {
            node.volume = BoundingSphere(nodes[node.children[0]].volume, nodes[node.children[1]].volume);
        }
        index = node.parent;
    }
}

void BoundingVolumeHierarchy::insert(int id, const BoundingSphere& volume)
{
    if (leafOf.count(id) != 0)
    {
        update(id, volume);
        return;
    }
    const int leaf = allocNode();
    nodes[leaf].volume = volume;
    nodes[leaf].id = id;
    leafOf[id] = leaf;
    if (root == NONE)
    {
        root = leaf;
        return;
    }

    // Down whichever side would have to grow least to take it
    int index = root;
    while (!nodes[index].isLeaf())
    {
        const int c0 = nodes[index].children[0];
        const int c1 = nodes[index].children[1];
        index = nodes[c0].volume.getGrowth(volume) < nodes[c1].volume.getGrowth(volume) ? c0 : c1;
    }

    // That leaf becomes an inner node holding the old leaf and the new one
    const int parent = nodes[index].parent;
    const int inner = allocNode();
    nodes[inner].parent = parent;
    nodes[inner].children[0] = index;
    nodes[inner].children[1] = leaf;
    nodes[index].parent = inner;
    nodes[leaf].parent = inner;
    if (parent == NONE)
    {
        root = inner;
    }
    else
    {
        int* children = nodes[parent].children;
        children[children[0] == index ? 0 : 1] = inner;
    }
    refit(inner);
}

bool BoundingVolumeHierarchy::remove(int id)
{
    auto found = leafOf.find(id);
    if (found == leafOf.end())
    {
        return false;
    }
    const int leaf = found->second;
    leafOf.erase(found);
    const int parent = nodes[leaf].parent;
    freeNode(leaf);
    if (parent == NONE)
    {
        root = NONE;
        return true;
    }

    // The sibling takes the parent's place
    const int sibling = nodes[parent].children[0] == leaf ? nodes[parent].children[1] : nodes[parent].children[0];
    const int grand = nodes[parent].parent;
    nodes[sibling].parent = grand;
    if (grand == NONE)
    {
        root = sibling;
    }
    else
    {
        int* children = nodes[grand].children;
        children[children[0] == parent ? 0 : 1] = sibling;
    }
    freeNode(parent);
    refit(grand);
    return true;
}

void BoundingVolumeHierarchy::update(int id, const BoundingSphere& volume)
{
    remove(id);
    insert(id, volume);
}

void BoundingVolumeHierarchy::clear()
{
    nodes.clear();
    freeNodes.clear();
    leafOf.clear();
    root = NONE;
}

void BoundingVolumeHierarchy::getPotentialContacts(std::vector<PotentialContact>& out) const
{
    if (root != NONE)
    {
        pairsWithin(root, out);
    }
}

void BoundingVolumeHierarchy::pairsWithin(int index, std::vector<PotentialContact>& out) const
{
    const Node& node = nodes[index];
    if (node.isLeaf())
    {
        return;
    }
    pairsBetween(node.children[0], node.children[1], out);
    pairsWithin(node.children[0], out);
    pairsWithin(node.children[1], out);
}

void BoundingVolumeHierarchy::pairsBetween(int a, int b, std::vector<PotentialContact>& out) const
{
    const Node& na = nodes[a];
    const Node& nb = nodes[b];
    if (!na.volume.overlaps(nb.volume))
    {
        return;
    }
    if (na.isLeaf() && nb.isLeaf())
    {
        out.push_back(makePair(na.id, nb.id));
        return;
    }

    // Open up the bigger of the two, unless it's a leaf
    if (na.isLeaf() || (!nb.isLeaf() && nb.volume.getSize() > na.volume.getSize()))
    {
        pairsBetween(a, nb.children[0], out);
        pairsBetween(a, nb.children[1], out);
    }
    else
    {
        pairsBetween(na.children[0], b, out);
        pairsBetween(na.children[1], b, out);
    }
}

void BoundingVolumeHierarchy::query(const BoundingSphere& volume, std::vector<int>& out) const
{
    if (root == NONE)
    {
        return;
    }
    std::vector<int> stack = { root };
    while (!stack.empty())
    {
        const Node& node = nodes[stack.back()];
        stack.pop_back();
        if (!node.volume.overlaps(volume))
        {
            continue;
        }
        if (node.isLeaf())
        {
            out.push_back(node.id);
        }
        else
        {
            stack.push_back(node.children[0]);
            stack.push_back(node.children[1]);
        }
    }
}

int BoundingVolumeHierarchy::depthOf(int index) const
{
    if (index == NONE || nodes[index].isLeaf())
    {
        return index == NONE ? 0 : 1;
    }
    return 1 + std::max(depthOf(nodes[index].children[0]), depthOf(nodes[index].children[1]));
}

int BoundingVolumeHierarchy::getDepth() const
{
    return depthOf(root);
}

void BspTree::build(const std::vector<BroadPhaseItem>& newItems, const std::vector<BspPlane>& planes, int newMaxLeafSize, int newMaxDepth)
{
    items = newItems;
    handPlanes = planes;
    maxLeafSize = std::max(newMaxLeafSize, 1);
    maxDepth = newMaxDepth;
    nodes.clear();
    std::vector<int> indices(items.size());
    for (int i = 0; i < (int)items.size(); i++)
    {
        indices[i] = i;
    }
    buildNode(indices, 0, 0);
}

bool BspTree::choosePlane(const std::vector<int>& indices, std::size_t& nextPlane, BspPlane& plane) const
{
    // A hand-placed plane only counts if it has objects wholly on each side
    while (nextPlane < handPlanes.size())
    {
        const BspPlane& candidate = handPlanes[nextPlane++];
        int frontOnly = 0;
        int backOnly = 0;
        for (int i : indices)
        {
            const real distance = candidate.normal * items[i].center - candidate.offset;
            if (distance > items[i].radius)
            {
                frontOnly++;
            }
            else if (distance < -items[i].radius)
            {
                backOnly++;
            }
        }
        if (frontOnly > 0 && backOnly > 0)
        {
            plane = candidate;
            return true;
        }
    }

    // Otherwise the median of the centres along the axis they spread furthest on
    Vector3 low(REAL_MAX, REAL_MAX, REAL_MAX);
    Vector3 high(-REAL_MAX, -REAL_MAX, -REAL_MAX);
    for (int i : indices)
    {
        const Vector3& c = items[i].center;
        low = Vector3(std::min(low.x, c.x), std::min(low.y, c.y), std::min(low.z, c.z));
        high = Vector3(std::max(high.x, c.x), std::max(high.y, c.y), std::max(high.z, c.z));
    }
    const Vector3 extent = high - low;
    int axis = 0;
    if (extent.y > axisOf(extent, axis))
    {
        axis = 1;
    }
    if (extent.z > axisOf(extent, axis))
    {
        axis = 2;
    }
    if (axisOf(extent, axis) <= 0)
    {
        return false;
    }
    std::vector<real> values;
    values.reserve(indices.size());
    for (int i : indices)
    {
        values.push_back(axisOf(items[i].center, axis));
    }
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    plane.normal = Vector3(axis == 0 ? (real)1 : 0, axis == 1 ? (real)1 : 0, axis == 2 ? (real)1 : 0);
    plane.offset = values[values.size() / 2];
    return true;
}

int BspTree::buildNode(std::vector<int>& indices, std::size_t nextPlane, int depth)
{
    const int index = (int)nodes.size();
    nodes.emplace_back();
    BspPlane plane;
    std::size_t cursor = nextPlane;
    if ((int)indices.size() <= maxLeafSize || depth >= maxDepth || !choosePlane(indices, cursor, plane))
    {
        nodes[index].items = indices;
        return index;
    }

    // Straddling objects go both ways
    std::vector<int> front;
    std::vector<int> back;
    for (int i : indices)
    {
        const real distance = plane.normal * items[i].center - plane.offset;
        if (distance > -items[i].radius)
        {
            front.push_back(i);
        }
        if (distance < items[i].radius)
        {
            back.push_back(i);
        }
    }
    // A cut that doesn't separate anything would recurse forever
    if (front.size() == indices.size() && back.size() == indices.size())
    {
        nodes[index].items = indices;
        return index;
    }
    nodes[index].plane = plane;
    const int frontIndex = buildNode(front, cursor, depth + 1);
    const int backIndex = buildNode(back, cursor, depth + 1);
    nodes[index].front = frontIndex;
    nodes[index].back = backIndex;
    return index;
}

void BspTree::getPotentialContacts(std::vector<PotentialContact>& out) const
{
    std::unordered_set<std::uint64_t> seen;
    for (const Node& node : nodes)
    {
        if (node.front != NONE)
        {
            continue;
        }
        for (std::size_t i = 0; i < node.items.size(); i++)
        {
            for (std::size_t j = i + 1; j < node.items.size(); j++)
            {
                const BroadPhaseItem& a = items[node.items[i]];
                const BroadPhaseItem& b = items[node.items[j]];
                if (spheresOverlap(a.center, a.radius, b.center, b.radius) && seen.insert(pairKey(a.id, b.id)).second)
                {
                    out.push_back(makePair(a.id, b.id));
                }
            }
        }
    }
}

void BspTree::queryNode(int index, const BoundingSphere& volume, std::vector<int>& out) const
{
    const Node& node = nodes[index];
    if (node.front == NONE)
    {
        for (int i : node.items)
        {
            if (spheresOverlap(items[i].center, items[i].radius, volume.center, volume.radius))
            {
                out.push_back(items[i].id);
            }
        }
        return;
    }
    const real distance = node.plane.normal * volume.center - node.plane.offset;
    if (distance > -volume.radius)
    {
        queryNode(node.front, volume, out);
    }
    if (distance < volume.radius)
    {
        queryNode(node.back, volume, out);
    }
}

void BspTree::query(const BoundingSphere& volume, std::vector<int>& out) const
{
    if (nodes.empty())
    {
        return;
    }
    const std::size_t start = out.size();
    queryNode(0, volume, out);
    std::sort(out.begin() + (std::ptrdiff_t)start, out.end());
    out.erase(std::unique(out.begin() + (std::ptrdiff_t)start, out.end()), out.end());
}

int BspTree::getLeafCount() const
{
    int count = 0;
    for (const Node& node : nodes)
    {
        if (node.front == NONE)
        {
            count++;
        }
    }
    return count;
}

MultiResolutionMap::MultiResolutionMap(real smallestCellSize, int levelCount)
{
    real size = smallestCellSize > 0 ? smallestCellSize : 1;
    levels.resize(std::max(levelCount, 1));
    for (Level& level : levels)
    {
        level.cellSize = size;
        size *= 2;
    }
}

void MultiResolutionMap::clear()
{
    for (Level& level : levels)
    {
        level.cells.clear();
    }
    items.clear();
    itemLevel.clear();
}

std::uint64_t MultiResolutionMap::key(int x, int y, int z)
{
    const std::uint64_t bias = 1u << 20;
    return (((std::uint64_t)(x + (int)bias)) & 0x1FFFFF) | ((((std::uint64_t)(y + (int)bias)) & 0x1FFFFF) << 21) |
           ((((std::uint64_t)(z + (int)bias)) & 0x1FFFFF) << 42);
}

void MultiResolutionMap::cellOf(const Level& level, const Vector3& point, int& x, int& y, int& z) const
{
    x = (int)std::floor(point.x / level.cellSize);
    y = (int)std::floor(point.y / level.cellSize);
    z = (int)std::floor(point.z / level.cellSize);
}

int MultiResolutionMap::getLevelOf(real radius) const
{
    for (int i = 0; i < (int)levels.size(); i++)
    {
        if (levels[i].cellSize >= radius * 2)
        {
            return i;
        }
    }
    return (int)levels.size(); // Too big for any grid: checked against everything
}

void MultiResolutionMap::insert(const BroadPhaseItem& item)
{
    const int index = (int)items.size();
    items.push_back(item);
    const int level = getLevelOf(item.radius);
    itemLevel.push_back(level);
    if (level < (int)levels.size())
    {
        int x;
        int y;
        int z;
        cellOf(levels[level], item.center, x, y, z);
        levels[level].cells[key(x, y, z)].push_back(index);
    }
}

void MultiResolutionMap::getPotentialContacts(std::vector<PotentialContact>& out) const
{
    const int levelCount = (int)levels.size();
    for (int i = 0; i < (int)items.size(); i++)
    {
        const BroadPhaseItem& a = items[i];
        const int own = itemLevel[i];
        if (own >= levelCount)
        {
            // Oversized: against every item after it, and every smaller one
            for (int j = 0; j < (int)items.size(); j++)
            {
                if (j == i || (itemLevel[j] >= levelCount && j < i))
                {
                    continue;
                }
                if (spheresOverlap(a.center, a.radius, items[j].center, items[j].radius))
                {
                    out.push_back(makePair(a.id, items[j].id));
                }
            }
            continue;
        }

        // Its own level and every coarser one; anything that can touch it is in a neighbouring cell there
        for (int l = own; l < levelCount; l++)
        {
            const Level& level = levels[l];
            int cx;
            int cy;
            int cz;
            cellOf(level, a.center, cx, cy, cz);
            for (int dz = -1; dz <= 1; dz++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        auto cell = level.cells.find(key(cx + dx, cy + dy, cz + dz));
                        if (cell == level.cells.end())
                        {
                            continue;
                        }
                        for (int j : cell->second)
                        {
                            if (l == own && j <= i)
                            {
                                continue;
                            }
                            if (spheresOverlap(a.center, a.radius, items[j].center, items[j].radius))
                            {
                                out.push_back(makePair(a.id, items[j].id));
                            }
                        }
                    }
                }
            }
        }
    }
}

void MultiResolutionMap::query(const BoundingSphere& volume, std::vector<int>& out) const
{
    for (const Level& level : levels)
    {
        // Items here are at most half a cell in radius, so widen the search by that much
        const real reach = volume.radius + level.cellSize * (real)0.5;
        int x0;
        int y0;
        int z0;
        int x1;
        int y1;
        int z1;
        cellOf(level, volume.center - Vector3(reach, reach, reach), x0, y0, z0);
        cellOf(level, volume.center + Vector3(reach, reach, reach), x1, y1, z1);
        for (int z = z0; z <= z1; z++)
        {
            for (int y = y0; y <= y1; y++)
            {
                for (int x = x0; x <= x1; x++)
                {
                    auto cell = level.cells.find(key(x, y, z));
                    if (cell == level.cells.end())
                    {
                        continue;
                    }
                    for (int j : cell->second)
                    {
                        if (spheresOverlap(items[j].center, items[j].radius, volume.center, volume.radius))
                        {
                            out.push_back(items[j].id);
                        }
                    }
                }
            }
        }
    }
    for (int j = 0; j < (int)items.size(); j++)
    {
        if (itemLevel[j] >= (int)levels.size() && spheresOverlap(items[j].center, items[j].radius, volume.center, volume.radius))
        {
            out.push_back(items[j].id);
        }
    }
}
