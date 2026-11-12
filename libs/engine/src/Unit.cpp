#include "engine/Unit.h"

#include <algorithm>

namespace engine
{
    Unit::Unit(UnitID id, Vector2 position, const Animation& primary, const Animation& alternate)
        : mID(id), mPosition(position), mPrimary(primary), mAlternate(alternate)
    {
    }

    void Unit::update(double dt)
    {
        if (!mIsActive)
        {
            return;
        }
        mPosition.x += velocity.x * (float)dt;
        mPosition.y += velocity.y * (float)dt;
        getCurrentAnimation().update(dt);
    }

    void Unit::draw() const
    {
        getCurrentAnimation().getCurrentSprite().draw(mPosition, scale);
    }

    void Unit::toggleAnimation()
    {
        mShowingAlternate = !mShowingAlternate;
        getCurrentAnimation().reset();
    }

    bool Unit::containsPoint(Vector2 point) const
    {
        const Sprite& sprite = getCurrentAnimation().getCurrentSprite();
        float halfWidth = sprite.getWidth() * scale * 0.5f;
        float halfHeight = sprite.getHeight() * scale * 0.5f;
        return point.x >= mPosition.x - halfWidth && point.x <= mPosition.x + halfWidth && point.y >= mPosition.y - halfHeight &&
            point.y <= mPosition.y + halfHeight;
    }

    Unit* UnitManager::addUnit(Vector2 position, const Animation& primary, const Animation& alternate)
    {
        mUnits.push_back(std::make_unique<Unit>(mNextID++, position, primary, alternate));
        return mUnits.back().get();
    }

    bool UnitManager::deleteUnit(UnitID id)
    {
        auto it = std::find_if(mUnits.begin(), mUnits.end(), [id](const auto& pUnit)
        {
            return pUnit->getID() == id;
        });
        if (it == mUnits.end())
        {
            return false;
        }
        mUnits.erase(it);
        return true;
    }

    void UnitManager::deleteAllUnits()
    {
        mUnits.clear();
    }

    Unit* UnitManager::getUnitAt(Vector2 point) const
    {
        for (auto it = mUnits.rbegin(); it != mUnits.rend(); ++it)
        {
            if ((*it)->containsPoint(point))
            {
                return it->get();
            }
        }
        return nullptr;
    }

    Unit* UnitManager::getUnit(UnitID id) const
    {
        for (const auto& pUnit : mUnits)
        {
            if (pUnit->getID() == id)
            {
                return pUnit.get();
            }
        }
        return nullptr;
    }

    void UnitManager::setAllActive(bool active)
    {
        for (const auto& pUnit : mUnits)
        {
            pUnit->setActive(active);
        }
    }

    void UnitManager::update(double dt)
    {
        for (const auto& pUnit : mUnits)
        {
            pUnit->update(dt);
        }
    }

    void UnitManager::draw() const
    {
        for (const auto& pUnit : mUnits)
        {
            pUnit->draw();
        }
    }
}
