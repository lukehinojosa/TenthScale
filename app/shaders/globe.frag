// A moon or a star seen on the map: its map lit by the Sun from one side, or a star glowing by its own light
in vec2 fragTexCoord;
in vec3 fragNormal;

uniform sampler2D texture0;
uniform vec3 sunDirection;
uniform float glow; // 1 for a star

out vec4 finalColor;

void main()
{
    vec3 color = texture(texture0, fragTexCoord).rgb;
    float light = max(dot(normalize(fragNormal), sunDirection), 0.0);
    finalColor = vec4(color * mix(0.02 + 1.7 * light, 1.0, glow), 1.0);
}
