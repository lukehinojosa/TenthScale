in vec3 vertexPosition;

uniform mat4 matView;
uniform mat4 matProjection;

out vec3 viewDirection;

void main()
{
    viewDirection = vertexPosition;
    // Rotation only, so the sky stays centered on the camera; z = w puts it on the far plane
    mat4 rotationOnly = mat4(mat3(matView));
    vec4 clip = matProjection * rotationOnly * vec4(vertexPosition, 1.0);
    gl_Position = clip.xyww;
}
