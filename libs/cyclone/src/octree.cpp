#include "octree.h"

using namespace cyclone;

Octree::Octree(const BoundingBox& bounds, size_t capacity, int maxDepth)
    : m_capacity(capacity == 0 ? 1 : capacity), m_maxDepth(maxDepth < 0 ? 0 : maxDepth)
{
    allocNode(bounds); // The root, at index 0
}

int Octree::allocNode(const BoundingBox& bounds)
{
    // Only grow the pool when every node in it is in use
    if (m_nodeCount == m_nodes.size())
    {
        m_nodes.emplace_back();
    }

    const int index = (int)m_nodeCount++;
    Node& node = m_nodes[index];
    node.bounds = bounds;
    node.items.clear(); // Keeps the vector's capacity for reuse
    for (int i = 0; i < 8; i++)
    {
        node.children[i] = kNone;
    }
    node.leaf = true;
    return index;
}

bool Octree::insert(const OctreeItem& item)
{
    if (!m_nodes[0].bounds.contains(item.position))
    {
        return false;
    }

    insertInto(0, item, 0);
    m_itemCount++;
    return true;
}

void Octree::clear()
{
    clear(m_nodes[0].bounds);
}

void Octree::clear(const BoundingBox& bounds)
{
    // Copy the bounds first, since they might belong to the root that's about
    // to be reset. Every node stays allocated to be reused.
    const BoundingBox rootBounds = bounds;
    m_nodeCount = 0;
    allocNode(rootBounds);
    m_itemCount = 0;
}

void Octree::insertInto(int nodeIndex, const OctreeItem& item, int depth)
{
    // Not a leaf, so pass it down to the child it's in
    if (!m_nodes[nodeIndex].leaf)
    {
        const int octant = octantOf(m_nodes[nodeIndex], item.position);
        insertInto(m_nodes[nodeIndex].children[octant], item, depth + 1);
        return;
    }

    // A leaf with room, or one at the max depth that has to take it anyway
    if (m_nodes[nodeIndex].items.size() < m_capacity || depth >= m_maxDepth)
    {
        m_nodes[nodeIndex].items.push_back(item);
        return;
    }

    // The leaf is full but can still split: subdivide and push its items down. That can move m_nodes, so the
    // node is looked up again by index afterwards.
    subdivide(nodeIndex);
    {
        Node& node = m_nodes[nodeIndex];
        for (const OctreeItem& existing : node.items)
        {
            const int octant = octantOf(node, existing.position);
            // Adding to a child's item vector never resizes m_nodes, so node
            // (and the items being looped over) stay valid
            m_nodes[node.children[octant]].items.push_back(existing);
        }
        node.items.clear(); // Keeps the vector's capacity for reuse
    }

    const int octant = octantOf(m_nodes[nodeIndex], item.position);
    insertInto(m_nodes[nodeIndex].children[octant], item, depth + 1);
}

void Octree::subdivide(int nodeIndex)
{
    // Copy the parent's bounds first, since allocating the children can move
    // m_nodes in memory
    const BoundingBox parent = m_nodes[nodeIndex].bounds;
    const Vector3 c = parent.center();

    // Each bit of a child's index picks the high half of one axis:
    // bit 0 = x, bit 1 = y, bit 2 = z. octantOf() uses the same order.
    int children[8];
    for (int i = 0; i < 8; i++)
    {
        const Vector3 min((i & 1) ? c.x : parent.min.x, (i & 2) ? c.y : parent.min.y, (i & 4) ? c.z : parent.min.z);
        const Vector3 max((i & 1) ? parent.max.x : c.x, (i & 2) ? parent.max.y : c.y, (i & 4) ? parent.max.z : c.z);
        children[i] = allocNode(BoundingBox(min, max));
    }

    // Only link the children once every allocation is done
    Node& node = m_nodes[nodeIndex];
    for (int i = 0; i < 8; i++)
    {
        node.children[i] = children[i];
    }
    node.leaf = false;
}

int Octree::octantOf(const Node& node, Vector3 p)
{
    const Vector3 c = node.bounds.center();
    int octant = 0;
    if (p.x >= c.x)
    {
        octant |= 1;
    }
    if (p.y >= c.y)
    {
        octant |= 2;
    }
    if (p.z >= c.z)
    {
        octant |= 4;
    }
    return octant;
}

void Octree::queryRange(const BoundingBox& range, std::vector<OctreeItem>& out) const
{
    queryNode(0, range, out);
}

void Octree::queryNode(int nodeIndex, const BoundingBox& range, std::vector<OctreeItem>& out) const
{
    const Node& node = m_nodes[nodeIndex];

    // Skip the whole subtree if its space misses the range
    if (!node.bounds.intersects(range))
    {
        return;
    }

    if (node.leaf)
    {
        for (const OctreeItem& item : node.items)
        {
            if (range.contains(item.position))
            {
                out.push_back(item);
            }
        }
        return;
    }

    for (int child : node.children)
    {
        queryNode(child, range, out);
    }
}
