#version 330 core

layout(lines_adjacency) in; // 4 vertices per primitive
layout(triangle_strip, max_vertices = 6) out;

in vec3 vertWorldPos[];

out vec3 worldPos;
out vec3 normal;
out vec2 texCoord;

void main() {
    vec4 p00 = gl_in[0].gl_Position; // Bottom-left
    vec4 p10 = gl_in[1].gl_Position; // Bottom-right
    vec4 p01 = gl_in[3].gl_Position; // Top-left
    vec4 p11 = gl_in[2].gl_Position; // Top-right

    // calc normal from quad
    vec3 edge1 = vertWorldPos[1] - vertWorldPos[0];
    vec3 edge2 = vertWorldPos[3] - vertWorldPos[0];
    vec3 quadNormal = normalize(cross(edge1, edge2));

    if (vertWorldPos[0].y + vertWorldPos[3].y > vertWorldPos[2].y + vertWorldPos[1].y) {
        // first triangle
        worldPos = vertWorldPos[0];
        normal = quadNormal;
        texCoord = vec2(0.0, 0.0);
        gl_Position = p00;
        EmitVertex();

        worldPos = vertWorldPos[1];
        normal = quadNormal;
        texCoord = vec2(1.0, 0.0);
        gl_Position = p10;
        EmitVertex();

        worldPos = vertWorldPos[3];
        normal = quadNormal;
        texCoord = vec2(0.0, 1.0);
        gl_Position = p01;
        EmitVertex();

        EndPrimitive();

        // other triangle
        worldPos = vertWorldPos[1];
        normal = quadNormal;
        texCoord = vec2(1.0, 1.0);
        gl_Position = p10;
        EmitVertex();

        worldPos = vertWorldPos[2];
        normal = quadNormal;
        texCoord = vec2(1.0, 1.0);
        gl_Position = p11;
        EmitVertex();

        worldPos = vertWorldPos[3];
        normal = quadNormal;
        texCoord = vec2(0.0, 1.0);
        gl_Position = p01;
        EmitVertex();

        EndPrimitive();
    } else {
        worldPos = vertWorldPos[0];
        normal = quadNormal;
        texCoord = vec2(0.0, 0.0);
        gl_Position = p00;
        EmitVertex();

        worldPos = vertWorldPos[1];
        normal = quadNormal;
        texCoord = vec2(1.0, 0.0);
        gl_Position = p10;
        EmitVertex();

        worldPos = vertWorldPos[2];
        normal = quadNormal;
        texCoord = vec2(1.0, 1.0);
        gl_Position = p11;
        EmitVertex();

        EndPrimitive();

        worldPos = vertWorldPos[0];
        normal = quadNormal;
        texCoord = vec2(0.0, 0.0);
        gl_Position = p00;
        EmitVertex();

        worldPos = vertWorldPos[2];
        normal = quadNormal;
        texCoord = vec2(1.0, 1.0);
        gl_Position = p11;
        EmitVertex();

        worldPos = vertWorldPos[3];
        normal = quadNormal;
        texCoord = vec2(0.0, 1.0);
        gl_Position = p01;
        EmitVertex();

        EndPrimitive();
    }
}
