// Follows atmosphere.frag. Alpha carries how much of the stars behind still shows, so the bright day sky hides them.
in vec3 viewDirection;

uniform vec3 cameraPosition;
uniform float sunDisc; // 0 where the Sun is drawn as a body instead
uniform vec4 discs[4]; // bodies drawn behind the air: the way to each, and the cosine of half its size across
uniform int discCount;

out vec4 finalColor;
void main()
{
    vec3 dir = normalize(viewDirection);
    vec3 camera = cameraPosition - planetCentre;
    vec2 ground = sphereHits(camera, dir, planetC(max(world.z, 0.0)));
    bool open = !(ground.x > 0.0 && ground.x < ground.y);
    vec2 span = airSpan(camera, dir, open ? 1e20 : ground.x);
    vec3 light = vec3(0.0);
    vec3 through = vec3(1.0);
    if (span.y > 0.0)
    {
        scatter(camera + dir * span.x, dir, span.y, light, through);
    }
    vec3 color = 1.0 - exp(-light);
    float brightness = dot(color, vec3(0.3, 0.5, 0.2));
    bool body = false;
    for (int i = 0; i < 4; i++)
    {
        if (i < discCount && dot(dir, discs[i].xyz) > discs[i].w)
        {
            body = true;
        }
    }
    if (open && !body)
    {
        float angle = acos(clamp(dot(dir, sunDirection), -1.0, 1.0));
        float disc = 1.0 - smoothstep(0.012, 0.016, angle);
        color = min(color + vec3(1.0, 0.95, 0.85) * through * disc * sunDisc * 3.0, vec3(1.0));
    }
    float stars = (through.r + through.g + through.b) / 3.0 * clamp(1.0 - brightness * 3.0, 0.0, 1.0);
    // A body behind shows through by however much light gets through the air, unlike the stars the day sky hides
    float behind = (through.r + through.g + through.b) / 3.0;
    finalColor = vec4(color, body ? behind : stars);
}
