#version 330 core

layout(lines_adjacency) in;
layout(triangle_strip, max_vertices = 6) out;

in vec3 vertWorldPos[];
in vec3 vertNormal[];

out vec3 worldPos;
out vec3 normal;
out vec2 texCoord;
out float roughness;

float computeTriangleRoughness(vec3 a, vec3 b, vec3 c) {
    float minY = min(min(a.y, b.y), c.y);
    float maxY = max(max(a.y, b.y), c.y);
    float heightDiff = maxY - minY;

    // Map height difference to roughness: 0 (flat) → 0.5 (choppy water)
    return clamp(heightDiff * 2.5, 0.05, 0.5); // adjust multiplier as needed
}

void emitVertex(int index, vec4 pos, vec2 uv, float r) {
    worldPos = vertWorldPos[index];
    normal = vertNormal[index];
    texCoord = uv;
    roughness = r;
    gl_Position = pos;
    EmitVertex();
}

void main() {
    vec4 p00 = gl_in[0].gl_Position;
    vec4 p10 = gl_in[1].gl_Position;
    vec4 p01 = gl_in[3].gl_Position;
    vec4 p11 = gl_in[2].gl_Position;

    // Compute roughness per triangle using height variation
    float roughA = computeTriangleRoughness(vertWorldPos[0], vertWorldPos[1], vertWorldPos[3]);
    float roughB = computeTriangleRoughness(vertWorldPos[1], vertWorldPos[2], vertWorldPos[3]);

    if (vertWorldPos[0].y + vertWorldPos[3].y > vertWorldPos[2].y + vertWorldPos[1].y) {
        emitVertex(0, p00, vec2(0.0, 0.0), roughA);
        emitVertex(1, p10, vec2(1.0, 0.0), roughA);
        emitVertex(3, p01, vec2(0.0, 1.0), roughA);
        EndPrimitive();

        emitVertex(1, p10, vec2(1.0, 1.0), roughB);
        emitVertex(2, p11, vec2(1.0, 1.0), roughB);
        emitVertex(3, p01, vec2(0.0, 1.0), roughB);
        EndPrimitive();
    } else {
        emitVertex(0, p00, vec2(0.0, 0.0), roughA);
        emitVertex(1, p10, vec2(1.0, 0.0), roughA);
        emitVertex(2, p11, vec2(1.0, 1.0), roughA);
        EndPrimitive();

        emitVertex(0, p00, vec2(0.0, 0.0), roughB);
        emitVertex(2, p11, vec2(1.0, 1.0), roughB);
        emitVertex(3, p01, vec2(0.0, 1.0), roughB);
        EndPrimitive();
    }
}
