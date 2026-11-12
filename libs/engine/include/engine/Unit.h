#pragma once

#include <memory>
#include <vector>

#include <raylib.h>

#include <core/Trackable.h>

#include "Animation.h"

namespace engine
{
    using UnitID = unsigned int;

    // A 2D object that plays one of two animations; the simple alternative to core's component pattern
    class Unit : public core::Trackable
    {
    public:
        Unit(UnitID id, Vector2 position, const Animation& primary, const Animation& alternate);

        void update(double dt);
        void draw() const;

        void toggleAnimation();
        bool isShowingAlternate() const { return mShowingAlternate; }
        Animation& getCurrentAnimation() { return mShowingAlternate ? mAlternate : mPrimary; }
        const Animation& getCurrentAnimation() const { return mShowingAlternate ? mAlternate : mPrimary; }

        // Inactive units still draw but freeze on their current frame
        void setActive(bool active) { mIsActive = active; }
        bool isActive() const { return mIsActive; }

        UnitID getID() const { return mID; }
        Vector2 getPosition() const { return mPosition; }
        void setPosition(Vector2 position) { mPosition = position; }
        Vector2 velocity{};
        float scale = 1.0f;

        // Hit test against the current frame's rectangle, centered on the position
        bool containsPoint(Vector2 point) const;

    private:
        UnitID mID;
        Vector2 mPosition;
        Animation mPrimary;
        Animation mAlternate;
        bool mShowingAlternate = false;
        bool mIsActive = true;
    };

    // Owns every Unit; ids stay unique for the manager's lifetime
    class UnitManager : public core::Trackable
    {
    public:
        Unit* addUnit(Vector2 position, const Animation& primary, const Animation& alternate);
        bool deleteUnit(UnitID id);
        void deleteAllUnits();

        // The last-added unit under the point, since it draws on top
        Unit* getUnitAt(Vector2 point) const;
        Unit* getUnit(UnitID id) const;
        Unit* getLastUnit() const { return mUnits.empty() ? nullptr : mUnits.back().get(); }
        std::size_t getUnitCount() const { return mUnits.size(); }

        void setAllActive(bool active);
        void update(double dt);
        void draw() const;

    private:
        std::vector<std::unique_ptr<Unit>> mUnits;
        UnitID mNextID = 1;
    };
}
