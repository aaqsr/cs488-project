#pragma once

#include <linalg.h>

class Mesh;

class Inertia
{
    float mass = 0.0F;
    float volume = 0.0F;
    linalg::aliases::float3 com{0.0F};

    float int_x{0.0F}, int_x2{0.0F}, int_y2{0.0F}, int_z2{0.0F}, int_x3{0.0F},
      int_y3{0.0F}, int_z3{0.0F}, int_x2y{0.0F}, int_y2z{0.0F}, int_z2x{0.0F};

    float inertia_xx{0.0F}, inertia_yy{0.0F}, inertia_zz{0.0F},
      inertia_xy{0.0F}, inertia_yz{0.0F}, inertia_zx{0.0F};

    void calcIntegrals(const linalg::aliases::float3& x,
                       const linalg::aliases::float3& y,
                       const linalg::aliases::float3& z);

    void correctInertiaWithCOM();

    // TODO: Needs tuning
    void cleanupNumericalNoise(float threshold = 5e-7);

  public:
    Inertia(const std::vector<Mesh>& bodyMeshes,
            linalg::aliases::float3 scale = {1.0F, 1.0F, 1.0F},
            float density = 1.0F);

    void addFace(const linalg::aliases::float3& v0,
                 const linalg::aliases::float3& v1,
                 const linalg::aliases::float3& v2, float density);

    void rescale(float scale);

    [[nodiscard]] linalg::aliases::float3x3 initialInertiaMatrix() const;
    [[nodiscard]] float getMass() const;
    [[nodiscard]] const linalg::aliases::float3& getCentreOfMass() const;
};
