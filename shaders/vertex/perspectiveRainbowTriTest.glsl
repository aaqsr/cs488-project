#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColour;

uniform mat4 projection;
uniform mat4 view;
// uniform mat4 model; // TODO: what to do about model coords?

out vec4 vertexColour; // pipeline will then also interpolate colour

void main()
{
    gl_Position = projection * view * vec4(aPos, 1.0);
    vertexColour = aColour;
}
