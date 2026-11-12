#pragma once
#include <cstddef>
#include <vector>
#include "core.h"

namespace cyclone
{
    // An axis-aligned box from its min and max corners. A point on a max face counts as outside, so a point on a
    // dividing plane lands in only one child.
    struct BoundingBox
    {
        Vector3 min;
        Vector3 max;

        BoundingBox() = default;
        BoundingBox(Vector3 min, Vector3 max) : min(min), max(max) {}

        // Builds a cube around a center point, reaching half out on every axis.
        static BoundingBox fromCenterHalf(Vector3 center, real half)
        {
            const Vector3 extent(half, half, half);
            return BoundingBox(center - extent, center + extent);
        }

        Vector3 center() const { return (min + max) * (real)0.5; }
        Vector3 size() const { return max - min; }

        // True if p is inside. A point on a min face is inside, but a point
        // on a max face is not.
        bool contains(Vector3 p) const { return p.x >= min.x && p.x < max.x && p.y >= min.y && p.y < max.y && p.z >= min.z && p.z < max.z; }

        // True if the two boxes overlap. Touching faces count.
        bool intersects(const BoundingBox& other) const
        {
            return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y && max.y >= other.min.y && min.z <= other.max.z &&
                   max.z >= other.min.z;
        }
    };

    // A point stored in the octree, with the caller's id for finding the object again.
    struct OctreeItem
    {
        int id = 0;
        Vector3 position;
    };

    // Points in a tree that splits space into eight at each level, for finding what's near a spot. Full nodes
    // split, except at the max depth, where points stacked together could never be separated.
    class Octree
    {
    public:
        // How many items a leaf holds before it subdivides
        static constexpr size_t kDefaultCapacity = 16;
        // Limits the depth, so clustered points can't use unlimited memory
        static constexpr int kDefaultMaxDepth = 12;

        explicit Octree(const BoundingBox& bounds, size_t capacity = kDefaultCapacity, int maxDepth = kDefaultMaxDepth);

        // Adds a point. Returns false (and adds nothing) if it's outside the tree's bounds.
        bool insert(const OctreeItem& item);

        // Removes every item, leaving a single empty root with the same bounds.
        void clear();

        // Removes every item and changes the bounds, e.g. to fit where the
        // objects are this frame.
        void clear(const BoundingBox& bounds);

        // Appends every item whose position is inside range to out. out is not cleared first.
        void queryRange(const BoundingBox& range, std::vector<OctreeItem>& out) const;

        const BoundingBox& getBounds() const { return m_nodes[0].bounds; }
        size_t getItemCount() const { return m_itemCount; }

        // Number of nodes in the tree right now
        size_t getNodeCount() const { return m_nodeCount; }

    private:
        // Child index for a leaf, which has no children
        static constexpr int kNone = -1;

        struct Node
        {
            BoundingBox bounds;
            std::vector<OctreeItem> items; // Only used while the node is a leaf
            int children[8]; // Indices into m_nodes
            bool leaf = true;
        };

        // Takes an empty leaf from the pool and returns its index. It can move m_nodes in memory, so nodes are always
        // found again by index afterwards.
        int allocNode(const BoundingBox& bounds);

        void insertInto(int nodeIndex, const OctreeItem& item, int depth);
        void subdivide(int nodeIndex);
        static int octantOf(const Node& node, Vector3 p);
        void queryNode(int nodeIndex, const BoundingBox& range, std::vector<OctreeItem>& out) const;

        std::vector<Node> m_nodes; // The node pool. The root is index 0.
        size_t m_nodeCount = 0; // Nodes in use this frame (at most m_nodes.size())
        size_t m_capacity;
        int m_maxDepth;
        size_t m_itemCount = 0;
    };
}
