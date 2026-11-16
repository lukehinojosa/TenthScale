#pragma once

#include <optional>
#include <vector>

#include <raylib.h>

#include "flight/DesignStats.h"
#include "flight/PartCatalog.h"
#include "flight/RocketDesign.h"
#include "shared/OrbitCamera.h"
#include "shared/Scene.h"
#include "shared/SolarSystem.h"

namespace tenth
{
    // The assembly building: the rocket floats in the middle of the hall while parts are dragged onto it from the
    // panel on the left, with its stages and stats on the right
    class BuildScene : public Scene
    {
    public:
        BuildScene(Camera3D& camera, const PartCatalog& catalog, RocketDesign& design, const SolarSystem& solarSystem);

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
        void drawRocketPanel();
        void drawMarkers(Vector3 offset) const;

        bool mouseOverViewport() const;
        bool canStartRocket() const;
        std::optional<Snap> findSnap(Ray ray) const;
        std::optional<Snap> findStackSnap() const;
        std::optional<Snap> findRadialSnap(Ray ray) const;

        // Attaches the held parts at the snap and returns where each copy's root went
        std::vector<PartIndex> applySnap(RocketDesign& design, const Snap& snap) const;

        bool sameSymmetry(PartIndex a, PartIndex b) const;

        // Where the rocket's root sits in the hall
        Vector3 designOffset() const;
        void limitFocus();

        PartIndex pickPart(Ray ray) const;

        void pushUndo();
        void undo();
        void redo();
        void deleteSelected();

        Camera3D& mCamera;
        const PartCatalog& mCatalog;
        RocketDesign& mDesign;
        OrbitCamera mOrbit;
        double mSurfaceGravity = 9.81; // Earth's, for the launch pad TWR
        bool mShowMarkers = true;

        // Parts following the mouse: a new part from the panel, or a branch picked up off the rocket
        RocketDesign mHeld;
        bool mHeldFromPanel = false;
        std::optional<Snap> mSnap;
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
