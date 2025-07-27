#include "physics/rigidBodyMesh.hpp"

Triangle::Triangle(const linalg::aliases::float3& v0,
                   const linalg::aliases::float3& v1,
                   const linalg::aliases::float3& v2)
{
    vertices[0] = v0;
    vertices[1] = v1;
    vertices[2] = v2;

    linalg::aliases::float3 edge1 = v1 - v0;
    linalg::aliases::float3 edge2 = v2 - v0;
    normal = linalg::normalize(linalg::cross(edge1, edge2));

    centroid = (v0 + v1 + v2) / 3.0F;

    area = 0.5F * linalg::length(linalg::cross(edge1, edge2));
}

SubTriangle::SubTriangle(const linalg::aliases::float3& cent,
                         const linalg::aliases::float3& norm, float a)
  : centroid(cent), normal(norm), area(a)
{
}

std::vector<SubTriangle> Triangle::subdivide(int subdivisions) const
{
    std::vector<SubTriangle> subTriangles;

    if (subdivisions <= 1) {
        subTriangles.emplace_back(centroid, normal, area);
        return subTriangles;
    }

    float subArea = area / static_cast<float>(subdivisions * subdivisions);

    float step = 1.0F / static_cast<float>(subdivisions);

    for (int i = 0; i < subdivisions; ++i) {
        for (int j = 0; j < subdivisions - i; ++j) {
            // barycentric coords for sub-triangle centroid
            float u = (static_cast<float>(i) + 1.0F / 3.0F) * step;
            float v = (static_cast<float>(j) + 1.0F / 3.0F) * step;
            float w = 1.0F - u - v;

            // skip if outside triangle
            if (w < 0) {
                continue;
            }

            linalg::aliases::float3 subCentroid =
              u * vertices[0] + v * vertices[1] + w * vertices[2];

            subTriangles.emplace_back(subCentroid, normal, subArea);
        }
    }

    return subTriangles;
}
