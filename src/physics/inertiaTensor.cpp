#include "physics/inertiaTensor.hpp"
#include "frontend/mesh.hpp"
#include "util/logger.hpp"
#include <sstream>

namespace
{

void calcTemps(const linalg::aliases::float3& w, float& f1, float& f2,
               float& f3)
{
    float tmp1 = w[0] + w[1];
    f1 = tmp1 + w[2];
    float tmp2 = w[0] * w[0];
    float tmp3 = tmp2 + (w[1] * tmp1);
    f2 = tmp3 + w[2] * f1;
    f3 = w[0] * tmp2 + w[1] * tmp3 + w[2] * f2;
}

bool isTriangleDegenerate(const linalg::aliases::float3& v0,
                          const linalg::aliases::float3& v1,
                          const linalg::aliases::float3& v2,
                          float epsilon = 1e-8F)
{
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;
    auto cross_product = linalg::cross(edge1, edge2);
    return linalg::length(cross_product) < epsilon;
}

} // namespace

Inertia::Inertia(const std::vector<Mesh>& bodyMeshes,
                 linalg::aliases::float3 scale, float density)
{
    size_t totalFaces = 0;
    size_t degenerateFaces = 0;

    for (const Mesh& mesh : bodyMeshes) {
        if (mesh.getNumFaceIndices() % 3 != 0) {
            Logger::GetInstance().log(
              "WARNING. Non-triangulated mesh detected!");
            continue;
        }

        for (size_t face_idx = 0; face_idx < mesh.getNumFaceIndices();
             face_idx += 3)
        {
            // TODO : refactor to add a face struct to make this safe and
            // cleaner
            const auto v0 = mesh.getVertexAtFaceIndex(face_idx + 0);
            const auto v1 = mesh.getVertexAtFaceIndex(face_idx + 1);
            const auto v2 = mesh.getVertexAtFaceIndex(face_idx + 2);

            // process faces
            auto scaled_v0 = linalg::cmul(v0.position, scale);
            auto scaled_v1 = linalg::cmul(v1.position, scale);
            auto scaled_v2 = linalg::cmul(v2.position, scale);

            if (isTriangleDegenerate(scaled_v0, scaled_v1, scaled_v2)) {
                degenerateFaces++;
                continue;
            }

            addFace(scaled_v0, scaled_v1, scaled_v2, density);
            totalFaces++;
        }
    }
    Logger::GetInstance().log("[Inertia Tensor Calc] Processed " +
                              std::to_string(totalFaces) + " faces, skipped " +
                              std::to_string(degenerateFaces) +
                              " degenerate faces");

    correctInertiaWithCOM(); // idk if to use this com or not

    // TODO: i hate this little guy. should we use doubles instead to fix this??
    std::stringstream ss;
    ss << "Before COM: " << com.x << ", " << com.y << ", " << com.z << "\n";
    cleanupNumericalNoise();
    ss << "After COM: " << com.x << ", " << com.y << ", " << com.z << "\n";
    Logger::GetInstance().log(ss.str());
}

void Inertia::calcIntegrals(const linalg::aliases::float3& x,
                            const linalg::aliases::float3& y,
                            const linalg::aliases::float3& z)
{
    // find surface integral using barycentric parametrisation
    linalg::aliases::float3 f1;
    linalg::aliases::float3 f2;
    linalg::aliases::float3 f3;

    // common temp values
    calcTemps(x, f1.x, f2.x, f3.x);
    calcTemps(y, f1.y, f2.y, f3.y);
    calcTemps(z, f1.z, f2.z, f3.z);

    // basic integrals
    // Int{0, 1} (Int{0, 1-v} (x0+e1.x*u+e2.x*v) du) dv == (x0+x1+x2)/6
    int_x = f1.x / 6.0F;
    // Int{0, 1} (Int{0, 1-v} ((x0+e1.x*u+e2.x*v)^2) du) dv
    int_x2 = f2.x / 12.0F;
    // Int{0, 1} (Int{0, 1-v} ((y0+e1.y*u+e2.y*v)^2) du) dv
    int_y2 = f2.y / 12.0F;
    // Int{0, 1} (Int{0, 1-v} ((z0+e1.z*u+e2.z*v)^2) du) dv
    int_z2 = f2.z / 12.0F;
    int_x3 = f3.x / 20.0F;
    int_y3 = f3.y / 20.0F;
    int_z3 = f3.z / 20.0F;

    int_x2y = linalg::dot(y, f2.x + linalg ::cmul(x, (x + f1.x))) / 60.0F;
    int_y2z = linalg::dot(z, f2.y + linalg ::cmul(y, (y + f1.y))) / 60.0F;
    int_z2x = linalg::dot(x, f2.z + linalg ::cmul(z, (z + f1.z))) / 60.0F;
}

void Inertia::addFace(const linalg::aliases::float3& v0,
                      const linalg::aliases::float3& v1,
                      const linalg::aliases::float3& v2, float density)
{

    linalg::aliases::float3 edge1 = v1 - v0;
    linalg::aliases::float3 edge2 = v2 - v0;
    linalg::aliases::float3 norm = (linalg::cross(edge1, edge2));

    float area = linalg::length(norm) * 0.5;
    if (area <= 1e-8) {
        Logger::GetInstance().log("WARNING. Degenerate face with small area");
        return;
    }

    calcIntegrals({v0.x, v1.x, v2.x}, {v0.y, v1.y, v2.y}, {v0.z, v1.z, v2.z});

    float old_mass = mass;
    auto face_com = linalg::aliases::float3{int_x2 * norm.x, int_y2 * norm.y,
                                            int_z2 * norm.z} /
                    2;
    volume += norm.x * int_x;
    mass += norm.x * int_x * density;

    if (mass >= 1e-8) {
        com = (com * old_mass + density * face_com) / mass;
    }

    // surface integral over triangle with outward normal n:
    // Ixx = ∫∫(y² + z²) dS, where dS = |n|/n.x dx dy for projection onto
    // yz-plane

    // diagonal terms: Ixx = ∫(y² + z²)dm, etc.
    inertia_xx += density * (int_y3 * norm.y + int_z3 * norm.z) / 3;
    inertia_yy += density * (int_x3 * norm.x + int_z3 * norm.z) / 3;
    inertia_zz += density * (int_x3 * norm.x + int_y3 * norm.y) / 3;

    // off-diagonal terms
    // Ixy = -∫xy dm (negative sign later when building the matrix)
    inertia_xy += density * int_x2y * norm.x / 2;
    inertia_yz += density * int_y2z * norm.y / 2;
    inertia_zx += density * int_z2x * norm.z / 2;
}

void Inertia::correctInertiaWithCOM()
{
    inertia_xx -= mass * (com.y * com.y + com.z * com.z);
    inertia_yy -= mass * (com.x * com.x + com.z * com.z);
    inertia_zz -= mass * (com.x * com.x + com.y * com.y);
    inertia_xy -= mass * com.x * com.y;
    inertia_yz -= mass * com.y * com.z;
    inertia_zx -= mass * com.z * com.x;
}

// Round off to kill floating point noise. Clean up:
// - very small off-diagonal terms relative to diagonal terms
// - center of mass
void Inertia::cleanupNumericalNoise(float threshold)
{
    float max_diagonal = std::max(
      {std::abs(inertia_xx), std::abs(inertia_yy), std::abs(inertia_zz)});
    float relative_threshold = threshold * max_diagonal;

    if (std::abs(inertia_xy) < relative_threshold) {
        inertia_xy = 0.0F;
    }
    if (std::abs(inertia_yz) < relative_threshold) {
        inertia_yz = 0.0F;
    }
    if (std::abs(inertia_zx) < relative_threshold) {
        inertia_zx = 0.0F;
    }

    if (std::abs(com.x) < threshold) {
        com.x = 0.0F;
    }
    if (std::abs(com.y) < threshold) {
        com.y = 0.0F;
    }
    if (std::abs(com.z) < threshold) {
        com.z = 0.0F;
    }
}

void Inertia::rescale(float scale)
{
    // Faster rescaling when uniform
    float scaleFactor = static_cast<float>(linalg::pow(scale, 5));
    inertia_xx *= scaleFactor;
    inertia_yy *= scaleFactor;
    inertia_zz *= scaleFactor;
    inertia_xy *= scaleFactor;
    inertia_yz *= scaleFactor;
    inertia_zx *= scaleFactor;
    volume *= static_cast<float>(linalg::pow(scale, 3));
    mass *= static_cast<float>(linalg::pow(scale, 3));
    com *= scale;
}

linalg::aliases::float3x3 Inertia::initialInertiaMatrix() const
{
    linalg::aliases::float3x3 m;
    m[0] = {inertia_xx, -inertia_xy, -inertia_zx};
    m[1] = {-inertia_xy, inertia_yy, -inertia_yz};
    m[2] = {-inertia_zx, -inertia_yz, inertia_zz};
    return m;
}

float Inertia::getMass() const
{
    return mass;
}

const linalg::aliases::float3& Inertia::getCentreOfMass() const
{
    return com;
}
