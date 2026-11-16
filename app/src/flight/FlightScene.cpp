#include "flight/FlightScene.h"

#include <imgui.h>

namespace tenth
{
    FlightScene::FlightScene(const RocketDesign& design, const SolarSystem& solarSystem) : mDesign(design), mSolarSystem(solarSystem)
    {
    }

    void FlightScene::onEnter()
    {
        // The rocket leaves the pad as built, standing on the equator with its tanks full
        const BodyId earth = mSolarSystem.findBody("Earth");
        const Vehicle vehicle = makeVehicle(mDesign);
        mTime = 0.0;
        mRocket.setVehicle(vehicle);
        mRocket.place(earth, { mSolarSystem.body(earth).radius + vehicle.groundClearance, 0.0, 0.0 }, {});
        mStats = computeStats(mDesign, mSolarSystem.surfaceGravity(earth));
    }

    void FlightScene::onFixedUpdate(double dt)
    {
        mRocket.step(dt, mSolarSystem, mTime);
        mTime += dt;
    }

    void FlightScene::onGui()
    {
        const BodyInfo& reference = mSolarSystem.body(mRocket.reference());
        const Vehicle& vehicle = mRocket.vehicle();
        const double altitude = mRocket.position().length() - reference.radius - vehicle.groundClearance;
        ImGui::Begin("Flight", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
        if (mRocket.onGround())
        {
            ImGui::Text("On the ground at %s", reference.name.c_str());
        }
        else
        {
            ImGui::Text("Near %s", reference.name.c_str());
        }
        ImGui::Text("Altitude: %.1f km", altitude / 1000.0);
        ImGui::Text("Speed: %.0f m/s", mRocket.velocity().length());
        ImGui::Text("Mission time: %.0f s", mTime);

        ImGui::SeparatorText("Rocket");
        ImGui::Text("Parts: %d   Stages: %d", vehicle.design.partCount(), vehicle.design.stageCount());
        ImGui::Text("Mass: %.2f t", vehicle.mass / 1000.0);
        ImGui::Text("Center of mass: %.1f m above the bottom", vehicle.groundClearance);
        ImGui::Text("Inertia: %.0f t m^2 pitch, %.1f t m^2 roll", vehicle.inertia.xx / 1000.0, vehicle.inertia.yy / 1000.0);
        ImGui::Text("Delta-v: %.0f m/s   Liftoff TWR: %.2f", mStats.totalDeltaVVacuum, mStats.liftoffTwr);
        ImGui::TextDisabled("Engines and the flight view come next");
        if (ImGui::Button("Back to assembly"))
        {
            requestScene(SceneId::Build);
        }
        ImGui::End();
    }
}
