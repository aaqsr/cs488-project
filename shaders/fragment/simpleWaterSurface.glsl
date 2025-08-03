#version 330 core

in vec3 worldPos;
in vec3 normal;
in vec2 texCoord;

out vec4 FragColour;

void main() {
    // simple water shading just assuming the light is at a constant place and very far away
    vec3 lightDir = normalize(vec3(1.0, 1.0, -1.0));
    float diff = max(dot(normal, lightDir), 0.0);

    vec3 waterColour = vec3(0.2, 0.6, 1.0);
    vec3 finalColour = waterColor * (0.3 + 0.7 * diff);

    FragColour = vec4(finalColour, 0.7);
}

