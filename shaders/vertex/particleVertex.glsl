#version 330 core

layout(location = 0) in vec3 aPos; // Quad vertices
layout(location = 1) in vec3 aInstancePos; // Particle world position
layout(location = 2) in float aSize; // Particle size
layout(location = 3) in float aAlpha; // Particle transparency

uniform mat4 view;
uniform mat4 projection;
uniform vec3 cameraPos;

out float alpha;
out vec2 texCoord;

void main()
{
    // Calculate billboard vectors
    vec3 worldPos = aInstancePos;
    vec3 toCam = normalize(cameraPos - worldPos);
    vec3 up = vec3(0.0, 1.0, 0.0);
    vec3 right = normalize(cross(up, toCam));
    up = cross(toCam, right);

    // Create billboard quad
    vec3 vertexPos = worldPos + (right * aPos.x + up * aPos.y) * aSize;

    gl_Position = projection * view * vec4(vertexPos, 1.0);

    alpha = aAlpha;
    texCoord = aPos.xy + 0.5; // Convert from [-0.5, 0.5] to [0, 1]
}
