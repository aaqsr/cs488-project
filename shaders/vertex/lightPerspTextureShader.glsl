#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;

uniform mat4 projection;
uniform mat4 view;
// uniform mat4 model; // TODO: what to do about model coords?

out vec3 worldPos;
out vec3 normal;
out vec2 texCoords;

void main()
{
    worldPos = aPos;
    // once we have a model transform. must take care when transforming normal!!
    normal = aNormal;
    texCoords = aTexCoord;

    gl_Position = projection * view * vec4(aPos, 1.0);
}
