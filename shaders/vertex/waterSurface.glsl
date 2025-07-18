#version 330 core

layout(location = 0) in vec2 staticPos; // x,z pos
layout(location = 1) in float height; // y heights
layout(location = 2) in vec3 normal; // y heights

// TODO: just pass in MVP matrix instead
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec3 vertWorldPos;
out vec3 vertNormal;

void main() {
    mat4 mvpMatrix = projection * view * model;

    vertWorldPos = vec3(staticPos.x, height, staticPos.y);

    gl_Position = mvpMatrix * vec4(vertWorldPos, 1.0);

    // Computationally terrible.
    // Inverting a matrix is expensive on the GPU.
    // We invert *and* multiply matrices over and over for each vertex
    // when we could just be doing it once on the CPU.
    // So we should be computing the inverse of the model on CPU,
    // and we should be computing all of MVP on the CPU actually if possible...
    vertNormal = mat3(transpose(inverse(model))) * normal;
}
