#pragma once

#include <optional>
#include <vector>

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
        // Where the held parts would attach: a free stack node, or a spot on a part's side
        struct Snap
        {
            PartIndex parent = NO_PART;
            bool radial = false;
            Node node = Node::Bottom; // stack
            double angle = 0.0; // radial
            double height = 0.0; // radial, from the parent's centre
        };

        void updateHeld(Ray ray);
        void updateSelection(Ray ray);
        void updateShortcuts();
        void dropHeld();
        void drawPartList();
        void drawFooter();
        void drawHeldTooltip() const;

        bool mouseOverViewport() const;
        bool canStartRocket() const;
        std::optional<Snap> findSnap(Ray ray) const;
        std::optional<Snap> findStackSnap() const;
        std::optional<Snap> findRadialSnap(Ray ray) const;

        // Attaches the held parts at the snap and returns where each copy's root went
        std::vector<PartIndex> applySnap(RocketDesign& design, const Snap& snap) const;

        bool sameSymmetry(PartIndex a, PartIndex b) const;

        // Lifts the rocket so its lowest point rests on the plate
        Vector3 designOffset() const;

        PartIndex pickPart(Ray ray) const;

        void pushUndo();
        void undo();
        void redo();
        void deleteSelected();

        Camera3D& mCamera;
        const PartCatalog& mCatalog;
        RocketDesign& mDesign;
        OrbitCamera mOrbit;

        // Parts following the mouse: a new part from the panel, or a branch picked up off the rocket
        RocketDesign mHeld;
        bool mHeldFromPanel = false;
        std::optional<Snap> mSnap;
        bool mOverPlate = false;
        Vector3 mHeldPoint = {}; // where the held parts float when they aren't snapped

        PartIndex mHovered = NO_PART;
        PartIndex mSelected = NO_PART;
        PartIndex mPressedPart = NO_PART;
        Vector2 mPressPosition = {};
        bool mPressedInViewport = false;

        int mSymmetry = 1; // copies placed around the rocket for side-mounted parts

        std::vector<RocketDesign> mUndo;
        std::vector<RocketDesign> mRedo;
    };
}
