// Follows atmosphere.frag, whose air it sees the ground through
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec3 fragPosition;

uniform sampler2D texture0; // day
uniform sampler2D texture1; // night lights
uniform sampler2D texture2; // relief, with east in red and south in green
uniform sampler2D texture3; // clouds
uniform sampler2D texture4; // water
uniform vec3 cameraPosition;
uniform vec3 sitePosition;
uniform vec3 localColor;
uniform vec3 pole; // the planet's north

out vec4 finalColor;

const float RELIEF = 2.5; // how strongly the relief map tilts the light
void main()
{
    vec3 up = normalize(fragNormal);
    vec3 toCamera = cameraPosition - fragPosition;
    float distance = length(toCamera);
    vec3 view = toCamera / distance;
    vec3 sun = sunDirection;

    vec3 day = texture(texture0, fragTexCoord).rgb;
    vec3 lights = texture(texture1, fragTexCoord).rgb;
    float cloud = texture(texture3, fragTexCoord).r;
    float water = texture(texture4, fragTexCoord).r;

    // Up close the maps are a blur, so the ground round the launch site takes a plain grassy colour instead
    float near = (1.0 - smoothstep(1500.0, 6000.0, length(fragPosition - sitePosition))) * (1.0 - smoothstep(15000.0, 60000.0, distance));
    day = mix(day, localColor, near);
    water *= 1.0 - near;

    // Clouds painted on the ground look like snow from underneath, so they only show from a distance
    cloud *= smoothstep(4000.0, 25000.0, distance);

    // Mountains catch the light on the sides facing the sun
    vec3 east = cross(pole, up);
    east = dot(east, east) > 1e-8 ? normalize(east) : vec3(1.0, 0.0, 0.0);
    vec3 south = cross(east, up);
    vec2 tilt = (texture(texture2, fragTexCoord).rg * 2.0 - 1.0) * RELIEF * (1.0 - near);
    vec3 n = normalize(up + east * tilt.x + south * tilt.y);

    // Sunlight reaching the ground, reddened by the air it came through
    vec3 offset = fragPosition - planetCentre;
    vec3 sunlight = sunThrough(offset, max(length(offset) - world.x, 1.0));
    float lit = smoothstep(-0.08, 0.2, dot(up, sun));
    vec3 color = day * (0.03 + 1.4 * max(dot(n, sun), 0.0) * sunlight);
    float glint = pow(max(dot(reflect(-sun, up), view), 0.0), 60.0) * water * (1.0 - cloud);
    color += sunlight * glint * 0.5;
    color = mix(color, vec3(0.03) + 1.3 * max(dot(up, sun), 0.0) * sunlight, cloud);
    color += pow(lights, vec3(1.3)) * vec3(1.0, 0.85, 0.6) * (1.0 - lit) * (1.0 - cloud * 0.85) * 1.5;

    // The air between the eye and the ground dims it and lays the sky's colour over it
    vec3 camera = cameraPosition - planetCentre;
    vec2 span = airSpan(camera, -view, distance);
    vec3 light = vec3(0.0);
    vec3 through = vec3(1.0);
    if (span.y > 0.0)
    {
        scatter(camera - view * span.x, -view, span.y, light, through);
    }
    finalColor = vec4(color * through + (1.0 - exp(-light)), 1.0);
}
