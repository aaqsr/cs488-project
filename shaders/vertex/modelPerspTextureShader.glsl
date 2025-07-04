#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model; // TODO: what to do about model coords?

out vec2 texCoord;

void main()
{
    // worldPos = vec3(model * vec4(aPos, 1.0));
    // Computationally terrible.
    // Inverting a matrix is expensive on the GPU.
    // We invert *and* multiply matrices over and over for each vertex
    // when we could just be doing it once on the CPU.
    // So we should be computing the inverse of the model on CPU,
    // and we should be computing all of MVP on the CPU actually if possible...
    // normal = mat3(transpose(inverse(model))) * aNormal;

    texCoord = aTexCoord;

    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
