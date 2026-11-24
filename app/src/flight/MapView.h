#pragma once

#include <vector>

#include <imgui.h>
#include <raylib.h>

#include "flight/OrbitPath.h"
#include "shared/OrbitCamera.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    class BodyGlobes;
    class PlanetSurface;
    class SpaceSky;

    // The map: every planet and moon where it is, with its orbit, and the rocket's path ahead marked with its high
    // and low points and where it will come down or leave. Clicking a body targets it or centres the view on it.
    // It's drawn to scale in metres, measured from whatever the view is centred on so floats stay precise there.
    class MapView
    {
    public:
        // The flight as the map sees it each frame
        struct Flight
        {
            const SolarSystem* pSystem = nullptr;
            double time = 0.0;
            BodyId reference = NO_BODY;
            Vec3d position; // the rocket's, from the reference body's centre
            Vec3d velocity;
            std::vector<Vec3d> debris; // dropped pieces, from the same centre
        };

        // Picks each body's dot colour; the surface maps are the flight's, handed to draw
        void init(const SolarSystem& system);

        // Centres on the body the rocket is around, far enough out to see its whole orbit
        void reset(const Flight& flight);

        // Right-drag turns the view, the wheel zooms, and Tab moves the centre on to the next body
        void update(Camera3D& camera, const Flight& flight, float dt);

        // The stars and the bodies; Earth is drawn by the flight's own planet and air
        void draw(const Camera3D& camera, const Flight& flight, const PlanetSurface& earth, const SpaceSky& sky,
                  const BodyGlobes& globes) const;

        // The orbits, the path, the markers, and the menu a body opens when clicked, which can change the target
        void drawOverlay(const Camera3D& camera, const Flight& flight, BodyId& target);

    private:
        // Where the view is centred: on a body, or on the rocket with NO_BODY
        Vec3d focusPosition(const Flight& flight, BodyId focus) const;
        Vec3d bodyPosition(const Flight& flight, BodyId id) const;
        double minimumDistance(const Flight& flight, BodyId focus) const;
        double framingDistance(const Flight& flight, BodyId focus) const;
        void setFocus(const Flight& flight, BodyId focus);

        // The direction to the star at the root of the Solar System, from a body
        Vec3d sunFrom(const Flight& flight, BodyId id) const;

        std::vector<ImU32> mDots; // each body's colour for the dot it shrinks to far away

        OrbitCamera mCamera;
        BodyId mFocus = NO_BODY;
        Vec3d mOrigin; // the focus's position from the Sun, where the drawing is measured from
        Vec3d mEase; // how far the view still has to slide after the focus changed, from the new focus
        float mDistanceTarget = 0.0f; // 0 while not easing the zoom

        // The menu a clicked body or the rocket opens
        struct Menu
        {
            bool open = false;
            bool fresh = false; // opened this frame, so the click that opened it doesn't also close it
            ImVec2 at;
            BodyId body = NO_BODY; // NO_BODY for the rocket
        };
        Menu mMenu;
    };
}
