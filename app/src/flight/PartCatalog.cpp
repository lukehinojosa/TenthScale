#include "flight/PartCatalog.h"

#include <utility>

namespace tenth
{
    const char* categoryName(PartCategory category)
    {
        switch (category)
        {
        case PartCategory::Pod:
            return "Pods";
        case PartCategory::Tank:
            return "Fuel tanks";
        case PartCategory::Engine:
            return "Engines";
        case PartCategory::Coupling:
            return "Coupling";
        case PartCategory::Aero:
            return "Aerodynamics";
        case PartCategory::Utility:
            return "Utility";
        }
        return "Other";
    }

    void PartCatalog::add(PartDefinition definition)
    {
        mParts.push_back(std::move(definition));
    }

    const PartDefinition* PartCatalog::find(std::string_view id) const
    {
        for (const PartDefinition& part : mParts)
        {
            if (part.id == id)
            {
                return &part;
            }
        }
        return nullptr;
    }

    namespace
    {
        PartDefinition roundPart(const char* id, const char* name, PartCategory category, double height, double topDiameter,
                                 double bottomDiameter, double dryMass)
        {
            PartDefinition part;
            part.id = id;
            part.name = name;
            part.category = category;
            part.shape.height = height;
            part.shape.topDiameter = topDiameter;
            part.shape.bottomDiameter = bottomDiameter;
            part.dryMass = dryMass;
            part.acceptsRadial = true;
            return part;
        }

        PartDefinition tank(const char* id, const char* name, double height, double dryMass, double fuelMass)
        {
            PartDefinition part = roundPart(id, name, PartCategory::Tank, height, 1.25, 1.25, dryMass);
            part.fuelMass = fuelMass;
            part.description = "Liquid fuel and oxidizer for the engines below it.";
            return part;
        }

        PartDefinition engine(const char* id, const char* name, double height, double dryMass, double thrustVacuum, double ispVacuum,
                              double ispSeaLevel)
        {
            PartDefinition part = roundPart(id, name, PartCategory::Engine, height, 1.25, 0.9, dryMass);
            part.thrustVacuum = thrustVacuum;
            part.ispVacuum = ispVacuum;
            part.ispSeaLevel = ispSeaLevel;
            return part;
        }

        PartDefinition boxPart(const char* id, const char* name, PartCategory category, double height, double span, double thickness,
                               double dryMass)
        {
            PartDefinition part;
            part.id = id;
            part.name = name;
            part.category = category;
            part.shape.kind = ShapeKind::Box;
            part.shape.height = height;
            part.shape.span = span;
            part.shape.thickness = thickness;
            part.dryMass = dryMass;
            part.hasTopNode = false;
            part.hasBottomNode = false;
            part.mountsRadially = true;
            return part;
        }
    }

    // Real-size parts: only the planets are scaled down, so masses and thrusts are ordinary rocket numbers
    PartCatalog PartCatalog::standard()
    {
        PartCatalog catalog;

        PartDefinition pod = roundPart("pod", "Command Pod", PartCategory::Pod, 1.0, 0.6, 1.25, 800.0);
        pod.hasParachute = true;
        pod.description = "Seats the crew, steers with its reaction wheels, and carries a parachute for landing on Earth. "
                          "Every rocket is built around one.";
        catalog.add(pod);

        catalog.add(tank("tank-small", "Small Tank", 1.0, 125.0, 1000.0));
        catalog.add(tank("tank-medium", "Medium Tank", 2.0, 250.0, 2000.0));
        catalog.add(tank("tank-large", "Large Tank", 4.0, 500.0, 4000.0));

        PartDefinition lifter = engine("engine-lifter", "Lifter Engine", 1.6, 1250.0, 215000.0, 310.0, 280.0);
        lifter.description = "Strong at sea level. Gets a rocket off the pad.";
        catalog.add(lifter);

        PartDefinition vacuum = engine("engine-vacuum", "Vacuum Engine", 1.2, 500.0, 60000.0, 345.0, 85.0);
        vacuum.description = "Weak in thick air but efficient in space. For upper stages.";
        catalog.add(vacuum);

        PartDefinition decoupler = roundPart("decoupler", "Decoupler", PartCategory::Coupling, 0.2, 1.25, 1.25, 50.0);
        decoupler.separates = true;
        decoupler.acceptsRadial = false;
        decoupler.description = "Splits the rocket in two when staged, dropping everything below it.";
        catalog.add(decoupler);

        PartDefinition nose = roundPart("nose-cone", "Nose Cone", PartCategory::Aero, 0.8, 0.0, 1.25, 50.0);
        nose.hasTopNode = false;
        nose.acceptsRadial = false;
        nose.popsOffAtPeak = true;
        nose.fitsParentDiameter = true;
        nose.description = "Cuts drag on the way up, then pops off at peak altitude to uncover the pod's parachute.";
        catalog.add(nose);

        PartDefinition fin = boxPart("fin", "Fin", PartCategory::Aero, 0.8, 0.5, 0.05, 50.0);
        fin.description = "Keeps the rocket pointing forward in the air. Mount low and in symmetry.";
        catalog.add(fin);

        PartDefinition leg = boxPart("landing-leg", "Landing Leg", PartCategory::Utility, 1.2, 0.4, 0.15, 60.0);
        leg.description = "Takes the impact of a landing. Mount at least three in symmetry.";
        catalog.add(leg);

        return catalog;
    }
}
