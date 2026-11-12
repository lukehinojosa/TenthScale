#include "shared/SolarSystem.h"

namespace tenth
{
    BodyId SolarSystem::findBody(std::string_view name) const
    {
        for (BodyId id = 0; id < bodyCount(); id++)
        {
            if (body(id).name == name)
            {
                return id;
            }
        }
        return NO_BODY;
    }

    double SolarSystem::surfaceGravity(BodyId id) const
    {
        const BodyInfo& info = body(id);
        return info.mu / (info.radius * info.radius);
    }
}
