#include "ai/Flocking.h"

namespace ai
{
    Vec2 SeparationRule::computeForce(const std::vector<const Boid*>& neighbors, const Boid& self)
    {
        Vec2 force;
        for (const Boid* pNeighbor : neighbors)
        {
            Vec2 away = self.position - pNeighbor->position;
            float distance = away.length();
            if (distance > 1e-4f && distance < desiredDistance)
            {
                force += away.normalized() * (desiredDistance / distance);
            }
        }
        return force;
    }

    Vec2 AlignmentRule::computeForce(const std::vector<const Boid*>& neighbors, const Boid& self)
    {
        if (neighbors.empty())
        {
            return {};
        }

        Vec2 average;
        for (const Boid* pNeighbor : neighbors)
        {
            average += pNeighbor->velocity;
        }
        average = average / (float)neighbors.size();
        return average - self.velocity; // Steer toward the average, not just add it
    }

    Vec2 CohesionRule::computeForce(const std::vector<const Boid*>& neighbors, const Boid& self)
    {
        if (neighbors.empty())
        {
            return {};
        }

        Vec2 center;
        for (const Boid* pNeighbor : neighbors)
        {
            center += pNeighbor->position;
        }
        center = center / (float)neighbors.size();
        return (center - self.position).normalized();
    }

    Vec2 BoundedAreaRule::computeForce(const std::vector<const Boid*>&, const Boid& self)
    {
        Vec2 force;
        float toLeft = self.position.x - min.x;
        float toRight = max.x - self.position.x;
        float toBottom = self.position.y - min.y;
        float toTop = max.y - self.position.y;

        // Inverse distance, so the push grows sharply near the wall; outside the box it pushes at full strength
        if (toLeft < margin)
        {
            force.x += margin / std::fmax(toLeft, 1e-2f);
        }
        if (toRight < margin)
        {
            force.x -= margin / std::fmax(toRight, 1e-2f);
        }
        if (toBottom < margin)
        {
            force.y += margin / std::fmax(toBottom, 1e-2f);
        }
        if (toTop < margin)
        {
            force.y -= margin / std::fmax(toTop, 1e-2f);
        }
        return force;
    }

    Vec2 WindRule::computeForce(const std::vector<const Boid*>&, const Boid&)
    {
        return { std::cos(angleRadians), std::sin(angleRadians) };
    }

    void Flock::addDefaultRules()
    {
        addRule<SeparationRule>();
        addRule<AlignmentRule>();
        addRule<CohesionRule>();
    }

    Boid& Flock::addBoid(Vec2 position, Vec2 velocity)
    {
        mBoids.push_back(Boid{ position, velocity });
        return mBoids.back();
    }

    void Flock::update(float dt)
    {
        // All forces come from the same snapshot, so boid order doesn't matter
        const float radiusSquared = neighborRadius * neighborRadius;
        mForces.assign(mBoids.size(), Vec2{});

        for (std::size_t i = 0; i < mBoids.size(); i++)
        {
            mNeighbors.clear();
            for (std::size_t j = 0; j < mBoids.size(); j++)
            {
                if (i != j && (mBoids[j].position - mBoids[i].position).lengthSquared() <= radiusSquared)
                {
                    mNeighbors.push_back(&mBoids[j]);
                }
            }

            Vec2 total;
            for (const auto& pRule : mRules)
            {
                if (pRule->enabled)
                {
                    total += pRule->computeForce(mNeighbors, mBoids[i]) * pRule->weight;
                }
            }
            mForces[i] = total.clamped(maxForce);
        }

        for (std::size_t i = 0; i < mBoids.size(); i++)
        {
            Boid& boid = mBoids[i];
            boid.velocity = (boid.velocity + mForces[i] * dt).clamped(maxSpeed);
            boid.position += boid.velocity * dt;
        }
    }
}
