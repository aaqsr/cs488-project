#version 330 core

in vec3 worldPos;
in vec3 normal;
in vec2 texCoords;

out vec4 FragColour;

struct Material {
    bool isDiffuseTextured;
    bool isSpecularTextured;
    sampler2D diffuse;
    sampler2D specular;
    vec3 Kd;
    vec3 Ks;
    float Ns; // shininess (specular exponent)
};

// could do direction lights and spotlights ... but don't need them yet
struct Light {
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;

    float constantFalloff;
    float linearFalloff;
    float quadraticFalloff;
};

#define NUM_LIGHTS 1

uniform Material material;

uniform Light lights[NUM_LIGHTS];

uniform vec3 viewPos; // camera's position

float gamma = 2.2;

// Gamma correction howwwww
vec3 getDiffuseColour()
{
    // return material.isDiffuseTextured ? material.Kd * pow(texture(material.diffuse, texCoords).rgb, vec3(gamma)) : material.Kd;
    return material.isDiffuseTextured ? material.Kd * vec3(texture(material.diffuse, texCoords)) : material.Kd;
}

vec3 getSpecularColour()
{
    // return material.isSpecularTextured ? material.Ks * pow(texture(material.specular, texCoords).rgb, vec3(gamma)) : material.Ks;
    return material.isSpecularTextured ? material.Ks * vec3(texture(material.specular, texCoords)) : material.Ks;
}

// could use the BRDF stuff mentioned in lecture, but phong lighting looked easier to implement...
vec3 calcLight(Light light, vec3 normal, vec3 viewDir)
{
    vec3 lightDir = normalize(light.position - worldPos);

    float distance = length(light.position - worldPos);
    // instead of using a purely quadratic fall-off as in PBR,
    // to get less harsh fall-off we use this
    float falloff = 1.0 / (light.linearFalloff + light.linearFalloff * distance + light.quadraticFalloff * (distance * distance));

    // ambient
    vec3 ambient = light.ambient * getDiffuseColour();

    // diffuse
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * getDiffuseColour();

    // specular
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.Ns);
    vec3 specular = light.specular * spec * getSpecularColour();

    // phong lighting
    return falloff * (ambient + diffuse + specular);
}

void main()
{
    vec3 norm = normalize(normal);
    vec3 viewDir = normalize(viewPos - worldPos);

    vec3 result = vec3(0.0);

    for (int i = 0; i < NUM_LIGHTS; ++i) {
        result += calcLight(lights[i], norm, viewDir);
    }

    // Gamma correction howww??? this no work =(
    // result = pow(result, vec3(1.0 / gamma));
    FragColour = vec4(result, 1.0);
}
