#pragma once

#include <memory>
#include <vector>

#include "Vec2.h"

namespace ai
{
    struct Boid
    {
        Vec2 position;
        Vec2 velocity;
    };

    // One steering behavior; the flock sums every enabled rule's force times its weight
    class FlockingRule
    {
    public:
        explicit FlockingRule(float weight) : weight(weight) {}
        virtual ~FlockingRule() = default;

        // neighbors excludes self and only holds boids inside the flock's neighbor radius
        virtual Vec2 computeForce(const std::vector<const Boid*>& neighbors, const Boid& self) = 0;
        virtual const char* getName() const = 0;

        float weight;
        bool enabled = true;
    };

    // Pushes away from close neighbors, harder the closer they are
    class SeparationRule : public FlockingRule
    {
    public:
        explicit SeparationRule(float weight = 1.5f, float desiredDistance = 2.0f) : FlockingRule(weight), desiredDistance(desiredDistance) {}
        Vec2 computeForce(const std::vector<const Boid*>& neighbors, const Boid& self) override;
        const char* getName() const override { return "Separation"; }

        float desiredDistance;
    };

    // Steers toward the neighbors' average velocity
    class AlignmentRule : public FlockingRule
    {
    public:
        explicit AlignmentRule(float weight = 1.0f) : FlockingRule(weight) {}
        Vec2 computeForce(const std::vector<const Boid*>& neighbors, const Boid& self) override;
        const char* getName() const override { return "Alignment"; }
    };

    // Steers toward the neighbors' center of mass
    class CohesionRule : public FlockingRule
    {
    public:
        explicit CohesionRule(float weight = 1.0f) : FlockingRule(weight) {}
        Vec2 computeForce(const std::vector<const Boid*>& neighbors, const Boid& self) override;
        const char* getName() const override { return "Cohesion"; }
    };

    // Pushes back from the edges of a rectangle once a boid is within margin of one
    class BoundedAreaRule : public FlockingRule
    {
    public:
        BoundedAreaRule(Vec2 min, Vec2 max, float margin, float weight = 2.0f) : FlockingRule(weight), min(min), max(max), margin(margin) {}
        Vec2 computeForce(const std::vector<const Boid*>& neighbors, const Boid& self) override;
        const char* getName() const override { return "Bounded Area"; }

        Vec2 min;
        Vec2 max;
        float margin;
    };

    // Constant push in one direction
    class WindRule : public FlockingRule
    {
    public:
        explicit WindRule(float angleRadians = 0.0f, float weight = 0.2f) : FlockingRule(weight), angleRadians(angleRadians) {}
        Vec2 computeForce(const std::vector<const Boid*>& neighbors, const Boid& self) override;
        const char* getName() const override { return "Wind"; }

        float angleRadians;
    };

    class Flock
    {
    public:
        float neighborRadius = 5.0f;
        float maxSpeed = 6.0f;
        float maxForce = 12.0f;

        // Separation, alignment, and cohesion with default weights
        void addDefaultRules();

        template<class Rule, class... Args>
        Rule* addRule(Args&&... args)
        {
            auto pRule = std::make_unique<Rule>(std::forward<Args>(args)...);
            Rule* pRaw = pRule.get();
            mRules.push_back(std::move(pRule));
            return pRaw;
        }

        Boid& addBoid(Vec2 position, Vec2 velocity = {});

        void update(float dt);

        std::vector<Boid>& getBoids() { return mBoids; }
        const std::vector<Boid>& getBoids() const { return mBoids; }
        std::vector<std::unique_ptr<FlockingRule>>& getRules() { return mRules; }

    private:
        std::vector<Boid> mBoids;
        std::vector<std::unique_ptr<FlockingRule>> mRules;
        std::vector<Vec2> mForces;
        std::vector<const Boid*> mNeighbors;
    };
}
