#version 330 core

layout(location = 0) in vec2 staticPos; // x,z pos
layout(location = 1) in float height; // y heights

// TODO: just pass in MVP matrix instead
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec3 vertWorldPos;

void main() {
    mat4 mvpMatrix = projection * view * model;
    vertWorldPos = vec3(staticPos.x, height, staticPos.y);
    gl_Position = mvpMatrix * vec4(vertWorldPos, 1.0);
}
