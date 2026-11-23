// Shared by the planet and the sky, and put ahead of each of their fragment shaders: sunlight scattered by the air
// between the eye and what it sees. The air is Earth's at a tenth of the size, so it scatters ten times as strongly.
uniform vec3 planetCentre;
uniform vec3 sunDirection;
uniform vec4 world; // planet radius, atmosphere thickness, camera height above the surface

const vec3 RAYLEIGH = vec3(5.8e-5, 1.35e-4, 3.31e-4);
const float RAYLEIGH_HEIGHT = 800.0;
const float MIE = 2.1e-4;
const float MIE_HEIGHT = 120.0;
const float MIE_G = 0.76;
const float SUN_POWER = 30.0;
const float PI = 3.14159265;
const int STEPS = 16;
const int LIGHT_STEPS = 4;

// Distances along a ray to where it enters and leaves a sphere round the planet's centre, entry past exit on a miss.
// c is the start's squared distance from the centre less the sphere's radius squared, worked out by the caller.
vec2 sphereHits(vec3 offset, vec3 dir, float c)
{
    float b = dot(offset, dir);
    float d = b * b - c;
    if (d < 0.0)
    {
        return vec2(1e20, -1e20);
    }
    float s = sqrt(d);
    return vec2(-b - s, -b + s);
}

float planetC(float height)
{
    return height * (height + 2.0 * world.x);
}

float airC(float height)
{
    return (height - world.y) * (height + 2.0 * world.x + world.y);
}

// How much of the sun's light gets down through the air to a point, given its offset from the centre and height
vec3 sunThrough(vec3 offset, float height)
{
    vec2 ground = sphereHits(offset, sunDirection, planetC(height));
    if (ground.x > 0.0 && ground.x < ground.y)
    {
        return vec3(0.0);
    }
    float span = max(sphereHits(offset, sunDirection, airC(height)).y, 0.0);
    float step = span / float(LIGHT_STEPS);
    float rayleigh = 0.0;
    float mie = 0.0;
    for (int i = 0; i < LIGHT_STEPS; i++)
    {
        float h = length(offset + sunDirection * (step * (float(i) + 0.5))) - world.x;
        rayleigh += exp(-h / RAYLEIGH_HEIGHT) * step;
        mie += exp(-h / MIE_HEIGHT) * step;
    }
    return exp(-(RAYLEIGH * rayleigh + MIE * 1.1 * mie));
}

// Light the air sends toward the eye along a stretch of a ray starting at offset from the centre, and the share of
// whatever lies beyond that still shows through
void scatter(vec3 offset, vec3 dir, float span, out vec3 light, out vec3 through)
{
    vec3 sumRayleigh = vec3(0.0);
    vec3 sumMie = vec3(0.0);
    float depthRayleigh = 0.0;
    float depthMie = 0.0;
    for (int i = 0; i < STEPS; i++)
    {
        // Steps lengthen with distance, where the detail matters less, and each is dimmed by the air up to its middle
        float near = span * pow(float(i) / float(STEPS), 2.0);
        float far = span * pow(float(i + 1) / float(STEPS), 2.0);
        float step = far - near;
        vec3 p = offset + dir * ((near + far) * 0.5);
        float h = length(p) - world.x;
        float rayleigh = exp(-h / RAYLEIGH_HEIGHT) * step;
        float mie = exp(-h / MIE_HEIGHT) * step;
        vec3 reach = sunThrough(p, h) * exp(-(RAYLEIGH * (depthRayleigh + rayleigh * 0.5) + MIE * 1.1 * (depthMie + mie * 0.5)));
        depthRayleigh += rayleigh;
        depthMie += mie;
        sumRayleigh += rayleigh * reach;
        sumMie += mie * reach;
    }
    float mu = dot(dir, sunDirection);
    float phaseRayleigh = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
    float g2 = MIE_G * MIE_G;
    float phaseMie = 3.0 / (8.0 * PI) * ((1.0 - g2) * (1.0 + mu * mu)) / ((2.0 + g2) * pow(1.0 + g2 - 2.0 * MIE_G * mu, 1.5));
    light = SUN_POWER * (phaseRayleigh * RAYLEIGH * sumRayleigh + phaseMie * MIE * sumMie);
    through = exp(-(RAYLEIGH * depthRayleigh + MIE * 1.1 * depthMie));
}

// The stretch of a ray from the camera that lies in the air before reaching a distance: where it starts, how long
vec2 airSpan(vec3 camera, vec3 dir, float limit)
{
    vec2 air = sphereHits(camera, dir, airC(world.z));
    float start = max(air.x, 0.0);
    float end = min(air.y, limit);
    return vec2(start, max(end - start, 0.0));
}
