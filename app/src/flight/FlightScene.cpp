#include "flight/FlightScene.h"

#include <cmath>

#include <imgui.h>

namespace tenth
{
    FlightScene::FlightScene(const SolarSystem& solarSystem) : mSolarSystem(solarSystem)
    {
    }

    void FlightScene::onEnter()
    {
        const BodyId earth = mSolarSystem.findBody("Earth");
        const BodyInfo& info = mSolarSystem.body(earth);
        const double radius = info.radius + 20.0e3;
        mTime = 0.0;
        mRocket.place(earth, { radius, 0.0, 0.0 }, { 0.0, 0.0, -std::sqrt(info.mu / radius) });
    }

    void FlightScene::onFixedUpdate(double dt)
    {
        mRocket.step(dt, mSolarSystem, mTime);
        mTime += dt;
    }

    void FlightScene::onGui()
    {
        const BodyInfo& reference = mSolarSystem.body(mRocket.reference());
        ImGui::Begin("Flight", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
        ImGui::Text("Orbiting: %s", reference.name.c_str());
        ImGui::Text("Altitude: %.1f km", (mRocket.position().length() - reference.radius) / 1000.0);
        ImGui::Text("Speed: %.0f m/s", mRocket.velocity().length());
        ImGui::Text("Mission time: %.0f s", mTime);
        ImGui::TextDisabled("The flight view comes later");
        if (ImGui::Button("Back to build"))
        {
            requestScene(SceneId::Build);
        }
        ImGui::End();
    }
}
