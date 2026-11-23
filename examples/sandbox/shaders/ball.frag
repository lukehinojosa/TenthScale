in vec3 fragNormal;
in float fragHeight;
uniform vec3 sunDirection;
uniform vec2 heightRange;
out vec4 finalColor;
void main()
{
    float t = clamp((fragHeight - heightRange.x) / (heightRange.y - heightRange.x), 0.0, 1.0);
    vec3 base = mix(vec3(0.15, 0.45, 0.95), vec3(1.0, 0.55, 0.15), t);
    float light = 0.3 + 0.7 * max(dot(normalize(fragNormal), normalize(sunDirection)), 0.0);
    finalColor = vec4(base * light, 1.0);
}
