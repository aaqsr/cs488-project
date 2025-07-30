#version 330 core
in vec3 worldPos;
in vec3 normal;
in vec2 texCoords;
out vec4 FragColour;

struct Material {
    sampler2D diffuse;
    sampler2D specular;
    vec3 Kd;
    vec3 Ks;
    float Ns; // shininess (specular exponent)
};

uniform Material material;
uniform vec3 viewPos; // camera's position

// Sun properties
const vec3 sunDirection = vec3(0.5773502691896258, 0.5773502691896258, -0.5773502691896258); // equal to normalize(vec3(1.0, 1.0, -1.0))
const vec3 sunAmbient = vec3(0.8, 0.8, 0.8); // 0.1 of RGB 237, 232, 224
const vec3 sunDiffuse = vec3(1.0, 1.0, 1.0); // RGB 248, 246, 243
const vec3 sunSpecular = vec3(1.0, 1.0, 1.0);

vec3 getDiffuseColour() {
    return material.Kd * vec3(texture(material.diffuse, texCoords));
}

vec3 getSpecularColour() {
    return material.Ks * vec3(texture(material.specular, texCoords));
}

void main() {
    vec3 norm = normalize(normal);
    vec3 viewDir = normalize(viewPos - worldPos);
    vec3 lightDir = sunDirection; // sun is inf. dist away
    vec3 halfwayDir = normalize(lightDir + viewDir);
    
    // Ambient
    vec3 ambient = sunAmbient * getDiffuseColour();
    
    // Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = sunDiffuse * diff * getDiffuseColour();
    
    // Specular
    float spec = pow(max(dot(norm, halfwayDir), 0.0), material.Ns);
    vec3 specular = sunSpecular * spec * getSpecularColour();
    
    // Final result (no falloff for directional sun light)
    vec3 result = ambient + diffuse + specular;
    
    FragColour = vec4(result, 1.0);
}
