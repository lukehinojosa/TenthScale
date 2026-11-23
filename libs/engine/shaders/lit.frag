#define MAX_POINT_LIGHTS 8

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;
in vec4 fragPosLightSpace;

uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec3 viewPos;

uniform vec3 sunDirection;
uniform vec3 sunColor;
uniform float ambient;
uniform float specularStrength;
uniform float shininess;

uniform int pointLightCount;
uniform vec3 pointLightPosition[MAX_POINT_LIGHTS];
uniform vec3 pointLightColor[MAX_POINT_LIGHTS];
uniform float pointLightRange[MAX_POINT_LIGHTS];

uniform sampler2D shadowMap;
uniform int shadowsEnabled;
uniform float shadowTexel;

out vec4 finalColor;

float sunShadow(vec3 normal, vec3 lightDir)
{
    if (shadowsEnabled == 0)
    {
        return 0.0;
    }
    vec3 coords = fragPosLightSpace.xyz / fragPosLightSpace.w * 0.5 + 0.5;
    if (coords.z > 1.0 || coords.x < 0.0 || coords.x > 1.0 || coords.y < 0.0 || coords.y > 1.0)
    {
        return 0.0;
    }

    // Steeper light angles need more bias to avoid acne
    float bias = max(0.004 * (1.0 - dot(normal, lightDir)), 0.0008);
    float shadow = 0.0;
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            float closest = texture(shadowMap, coords.xy + vec2(float(x), float(y)) * shadowTexel).r;
            shadow += coords.z - bias > closest ? 1.0 : 0.0;
        }
    }
    return shadow / 9.0;
}

vec3 blinnPhong(vec3 normal, vec3 lightDir, vec3 viewDir, vec3 color)
{
    float diffuse = max(dot(normal, lightDir), 0.0);
    vec3 halfway = normalize(lightDir + viewDir);
    float specular = pow(max(dot(normal, halfway), 0.0), shininess) * specularStrength;
    return color * (diffuse + specular);
}

void main()
{
    vec4 base = texture(texture0, fragTexCoord) * colDiffuse * fragColor;
    vec3 normal = normalize(fragNormal);
    vec3 viewDir = normalize(viewPos - fragPosition);

    vec3 sunDir = normalize(sunDirection);
    vec3 light = ambient * sunColor;
    light += (1.0 - sunShadow(normal, sunDir)) * blinnPhong(normal, sunDir, viewDir, sunColor);

    for (int i = 0; i < MAX_POINT_LIGHTS; i++)
    {
        if (i >= pointLightCount)
        {
            break;
        }
        vec3 toLight = pointLightPosition[i] - fragPosition;
        float distance = length(toLight);
        // Smooth falloff that reaches zero at the light's range
        float falloff = clamp(1.0 - distance / pointLightRange[i], 0.0, 1.0);
        falloff *= falloff;
        light += falloff * blinnPhong(normal, toLight / distance, viewDir, pointLightColor[i]);
    }

    finalColor = vec4(base.rgb * light, base.a);
}
