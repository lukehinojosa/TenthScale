// The Milky Way panorama, laid out in galactic coordinates: centred on the middle of the galaxy, with galactic
// longitude growing leftward. The planet's north is taken as the sky's north.
in vec3 viewDirection;

uniform sampler2D texture0;
uniform vec3 bodyEast;
uniform vec3 bodyUp;
uniform vec3 bodySouth;

out vec4 finalColor;

void main()
{
    vec3 d = normalize(viewDirection);
    vec3 body = bodyEast * d.x + bodyUp * d.y + bodySouth * d.z;
    vec3 equatorial = vec3(body.x, -body.z, body.y);
    vec3 galactic = vec3(dot(vec3(-0.0548755604, 0.4941094279, -0.8676661490), equatorial),
                         dot(vec3(-0.8734370902, -0.4448296300, -0.1980763734), equatorial),
                         dot(vec3(-0.4838350155, 0.7469822445, 0.4559837762), equatorial));
    float longitude = atan(galactic.y, galactic.x);
    float latitude = asin(clamp(galactic.z, -1.0, 1.0));
    vec2 spot = vec2(0.5 - longitude / 6.28318531, 0.5 - latitude / 3.14159265);
    finalColor = vec4(textureLod(texture0, spot, 0.0).rgb, 1.0);
}
