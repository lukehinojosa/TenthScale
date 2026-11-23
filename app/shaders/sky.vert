// A cube round the camera that turns with it but never moves, drawn on the far plane
in vec3 vertexPosition;

uniform mat4 matView;
uniform mat4 matProjection;

out vec3 viewDirection;

void main()
{
    viewDirection = vertexPosition;
    // Turned with the camera but never moved, and pushed out to the far plane
    vec4 clip = matProjection * mat4(mat3(matView)) * vec4(vertexPosition, 1.0);
    gl_Position = clip.xyww;
}
