#version 330 core

in float alpha;
in vec2 texCoord;

out vec4 FragColour;

void main()
{
    // Create circular particle
    float dist = length(texCoord - 0.5);
    if (dist > 0.5) discard;

    // Soft edges
    float edgeFade = 1.0 - smoothstep(0.3, 0.5, dist);

    // Water-like colour (light blue/white)
    vec3 colour = mix(vec3(0.3, 0.7, 1.0), vec3(0.7, 0.9, 1.0), edgeFade);

    float finalAlpha = alpha * edgeFade;
    FragColour = vec4(colour, finalAlpha);
}
