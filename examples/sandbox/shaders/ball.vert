// One sphere mesh drawn once per ball through instancing, coloured by height
in vec3 vertexPosition;
in vec3 vertexNormal;
in mat4 instanceTransform;
uniform mat4 mvp;
out vec3 fragNormal;
out float fragHeight;
void main()
{
    vec4 world = instanceTransform * vec4(vertexPosition, 1.0);
    fragNormal = mat3(instanceTransform) * vertexNormal;
    fragHeight = world.y;
    gl_Position = mvp * world;
}
