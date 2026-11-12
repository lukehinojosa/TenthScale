#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "core.h"

namespace cyclone
{
    // An object as the broad phase sees it: a bounding sphere and the caller's id for it
    struct BroadPhaseItem
    {
        int id = 0;
        Vector3 center;
        real radius = 0;
    };

    // Two objects whose bounding volumes overlap, so the narrow phase should test them; the lower id comes first
    struct PotentialContact
    {
        int a = 0;
        int b = 0;
    };

    struct BoundingSphere
    {
        Vector3 center;
        real radius = 0;

        BoundingSphere() = default;
        BoundingSphere(const Vector3& center, real radius) : center(center), radius(radius) {}

        // The smallest sphere holding both
        BoundingSphere(const BoundingSphere& one, const BoundingSphere& two);

        bool overlaps(const BoundingSphere& other) const;

        // How much bigger (in squared radius) this sphere would have to grow to hold other
        real getGrowth(const BoundingSphere& other) const;
        real getSize() const;
    };

    // A tree of bounding spheres (chapter 12.3). New objects go down the side that would grow least, so it stays
    // compact while objects come and go.
    class BoundingVolumeHierarchy
    {
    public:
        void insert(int id, const BoundingSphere& volume);
        bool remove(int id);

        // For moving objects; the same as remove then insert
        void update(int id, const BoundingSphere& volume);
        void clear();

        void getPotentialContacts(std::vector<PotentialContact>& out) const;
        void query(const BoundingSphere& volume, std::vector<int>& out) const;

        int getLeafCount() const { return (int)leafOf.size(); }
        int getDepth() const;

    private:
        static constexpr int NONE = -1;

        struct Node
        {
            BoundingSphere volume;
            int parent = NONE;
            int children[2] = { NONE, NONE };
            int id = 0;
            bool isLeaf() const { return children[0] == NONE; }
        };

        int allocNode();
        void freeNode(int index);
        void refit(int index);
        void pairsWithin(int index, std::vector<PotentialContact>& out) const;
        void pairsBetween(int a, int b, std::vector<PotentialContact>& out) const;
        int depthOf(int index) const;

        std::vector<Node> nodes;
        std::vector<int> freeNodes;
        std::unordered_map<int, int> leafOf;
        int root = NONE;
    };

    struct BspPlane
    {
        Vector3 normal; // Unit length
        real offset = 0; // How far the plane is from the origin along its normal
    };

    // A binary space partitioning tree (chapter 12.4.1). Hand-placed planes such as walls are used first, then it
    // splits each group across its widest direction.
    class BspTree
    {
    public:
        void build(const std::vector<BroadPhaseItem>& items, const std::vector<BspPlane>& planes = {}, int maxLeafSize = 8,
                   int maxDepth = 20);

        void getPotentialContacts(std::vector<PotentialContact>& out) const;
        void query(const BoundingSphere& volume, std::vector<int>& out) const;

        int getNodeCount() const { return (int)nodes.size(); }
        int getLeafCount() const;

    private:
        static constexpr int NONE = -1;

        struct Node
        {
            BspPlane plane;
            int front = NONE;
            int back = NONE;
            std::vector<int> items; // Indices into the item list; leaves only
        };

        int buildNode(std::vector<int>& indices, std::size_t nextPlane, int depth);
        bool choosePlane(const std::vector<int>& indices, std::size_t& nextPlane, BspPlane& plane) const;
        void queryNode(int index, const BoundingSphere& volume, std::vector<int>& out) const;

        std::vector<BroadPhaseItem> items;
        std::vector<BspPlane> handPlanes;
        std::vector<Node> nodes;
        int maxLeafSize = 8;
        int maxDepth = 20;
    };

    // Grids of several cell sizes (chapter 12.4.4). Each object goes in the finest grid its size fits, so a few big
    // objects don't force big cells on many small ones. Rebuild it each frame.
    class MultiResolutionMap
    {
    public:
        explicit MultiResolutionMap(real smallestCellSize = 1, int levels = 8);

        void clear();
        void insert(const BroadPhaseItem& item);

        void getPotentialContacts(std::vector<PotentialContact>& out) const;
        void query(const BoundingSphere& volume, std::vector<int>& out) const;

        int getLevelOf(real radius) const;
        int getLevelCount() const { return (int)levels.size(); }

    private:
        struct Level
        {
            real cellSize = 1;
            std::unordered_map<std::uint64_t, std::vector<int>> cells; // Indices into items
        };

        static std::uint64_t key(int x, int y, int z);
        void cellOf(const Level& level, const Vector3& point, int& x, int& y, int& z) const;

        std::vector<Level> levels;
        std::vector<BroadPhaseItem> items;
        std::vector<int> itemLevel;
    };
}
