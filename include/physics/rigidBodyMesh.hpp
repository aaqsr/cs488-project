#pragma once

#include <linalg.h>

// Not to be confused with frontend/mesh.hpp!
// This is world-space CPU triangles that represent the rigid body so that we
// can do physics on them. (In this case we primarily use them to compute forces
// on the body as a result of the fluid simulation and vice versa.)

struct SubTriangle;

struct Triangle
{
    linalg::aliases::float3 vertices[3];
    linalg::aliases::float3 normal;
    linalg::aliases::float3 centroid;
    float area;

    Triangle(const linalg::aliases::float3& v0,
             const linalg::aliases::float3& v1,
             const linalg::aliases::float3& v2);

    [[nodiscard]] std::vector<SubTriangle> subdivide(int subdivisions) const;
};

struct SubTriangle
{
    linalg::aliases::float3 centroid;
    linalg::aliases::float3 normal;
    float area;

    SubTriangle(const linalg::aliases::float3& cent,
                const linalg::aliases::float3& norm, float a);
};
