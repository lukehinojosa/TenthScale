#pragma once

#include <string>

namespace tenth
{
    enum class PartCategory
    {
        Pod,
        Tank,
        Engine,
        Coupling,
        Aero,
        Utility,
    };

    // Round parts are cylinders or cones along the rocket's axis; box parts stick out sideways, like fins and legs
    enum class ShapeKind
    {
        Round,
        Box,
    };

    // Sizes in metres. A part's origin is its centre.
    struct PartShape
    {
        ShapeKind kind = ShapeKind::Round;
        double height = 1.0;
        double topDiameter = 1.25; // round parts
        double bottomDiameter = 1.25; // round parts
        double span = 0.0; // box parts: how far it reaches out from the surface it's on
        double thickness = 0.0; // box parts
    };

    // A kind of part as it appears in the parts panel. Masses in kg, thrust in newtons, efficiency in seconds.
    struct PartDefinition
    {
        std::string id; // stable name used in saved designs
        std::string name;
        std::string description;
        PartCategory category = PartCategory::Tank;
        PartShape shape;

        double dryMass = 0.0;
        double fuelMass = 0.0;

        double thrustVacuum = 0.0; // engines only
        double ispVacuum = 0.0; // engines only: specific impulse in vacuum
        double ispSeaLevel = 0.0; // engines only: specific impulse at sea level, lower because of air pressure

        bool separates = false; // decouplers split the rocket into stages here
        bool hasParachute = false; // pods carry one built in, for a soft landing back on Earth
        bool popsOffAtPeak = false; // nose cones come away at the top of the climb, uncovering the pod's parachute
        bool fitsParentDiameter = false; // nose cones resize, keeping their proportions, to match what they sit on

        // Where it can connect: stack nodes on its top and bottom, or flat against another part's side
        bool hasTopNode = true;
        bool hasBottomNode = true;
        bool mountsRadially = false;
        bool acceptsRadial = false;

        double wetMass() const { return dryMass + fuelMass; }
        bool isEngine() const { return thrustVacuum > 0.0; }
    };

    const char* categoryName(PartCategory category);
}
