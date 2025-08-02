#version 330 core

layout(location = 0) in vec3 aPos;

// instance data - model matrix (4 vec4s)
layout(location = 1) in vec4 aModel0;
layout(location = 2) in vec4 aModel1;
layout(location = 3) in vec4 aModel2;
layout(location = 4) in vec4 aModel3;

uniform mat4 view;
uniform mat4 projection;

void main()
{
    // Reconstruct model matrix from instance data
    mat4 model = mat4(aModel0, aModel1, aModel2, aModel3);

    gl_Position = projection * view * model * vec4(aPos, 1.0);
}

