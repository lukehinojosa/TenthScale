#pragma once

#include <raylib.h>

#include "flight/PartCatalog.h"
#include "flight/RocketDesign.h"
#include "shared/OrbitCamera.h"
#include "shared/Scene.h"

namespace tenth
{
    // The build screen: drag parts from the panel on the left onto a build plate the camera orbits
    class BuildScene : public Scene
    {
    public:
        BuildScene(Camera3D& camera, const PartCatalog& catalog, RocketDesign& design);

        void onEnter() override;
        void onUpdate(double frameDt, double alpha) override;
        void onDraw3D() override;
        void onGui() override;

    private:
        void updateDrag(Ray ray);
        void updateSelection(Ray ray);
        void drawPartList();
        void drawFooter();

        bool mouseOverViewport() const;
        bool canDrop(const PartDefinition& definition) const;

        // Lifts the rocket so its lowest point rests on the plate
        Vector3 designOffset() const;

        PartIndex pickPart(Ray ray) const;

        Camera3D& mCamera;
        const PartCatalog& mCatalog;
        RocketDesign& mDesign;
        OrbitCamera mOrbit;

        const PartDefinition* mpDragged = nullptr; // the part being dragged out of the panel
        bool mDragOverPlate = false;
        Vector3 mDragPoint = {}; // where the mouse ray meets the plate's level

        PartIndex mHovered = NO_PART;
        PartIndex mSelected = NO_PART;
        Vector2 mPressPosition = {};
        bool mPressedInViewport = false;
    };
}
