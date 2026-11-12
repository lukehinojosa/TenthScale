#pragma once

#include <raylib.h>

#include "shared/OrbitCamera.h"
#include "shared/Scene.h"

namespace tenth
{
    // The build screen: a parts panel on the left and a build plate the camera orbits
    class BuildScene : public Scene
    {
    public:
        explicit BuildScene(Camera3D& camera);

        void onEnter() override;
        void onUpdate(double frameDt, double alpha) override;
        void onDraw3D() override;
        void onGui() override;

    private:
        Camera3D& mCamera;
        OrbitCamera mOrbit;
    };
}
