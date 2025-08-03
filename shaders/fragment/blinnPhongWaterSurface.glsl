#version 330 core

in vec3 worldPos;
in vec3 normal;
in vec2 texCoord;
in float roughness;

out vec4 FragColour;

uniform samplerCube skybox;
uniform vec3 cameraPos;

// Lighting
const vec3 lightDir = vec3(0.5773502691896258, 0.5773502691896258, -0.5773502691896258); // (1.0, 1.0, -1.0) pre-normalized

// Water appearance
const vec3 waterColour = vec3(0.2, 0.6, 1.0);
const float transparency = 0.7;

void main() {
    vec3 N = normalize(normal);
    vec3 V = normalize(cameraPos - worldPos);
    vec3 L = normalize(lightDir);
    vec3 H = normalize(L + V); // Blinn-Phong half vector

    // Diffuse and specular lighting
    float diffuse = max(dot(N, L), 0.0);
    float specular = pow(max(dot(N, H), 0.0), 32.0 / roughness);

    // Base water shading
    vec3 baseColour = waterColour * (0.3 + 0.7 * diffuse);
    vec3 litColour = baseColour + specular * vec3(1.0) * 0.8;

    // Reflection from environment
    vec3 reflected = reflect(-V, N);
    vec3 skyboxColour = texture(skybox, reflected).rgb;

    // Fresnel reflectance
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 2.0);

    // Blend reflection with lit water
    vec3 finalColour = mix(litColour, skyboxColour, fresnel);
    float finalAlpha = mix(transparency, 1.0, fresnel);

    FragColour
        = vec4(finalColour, finalAlpha);
}
