#pragma once

#include "frontend/shader.hpp"
#include "physics/constants.hpp"
#include "util/math.hpp"
#include <GL/glew.h>

#include <functional>
#include <linalg.h>

#include <numbers>
#include <random>
#include <vector>

struct Particle;

/**
 * @class ParticleSystem
 * @ingroup frontend
 * @brief Simple particle system for water splash effects. Self-contained, using
 * simple physics.
 */
class ParticleSystem
{
    std::vector<Particle> particles;

    static constexpr size_t maxParticles = 2000;

  public:
    ParticleSystem();

    /**
     * @brief Emit splash particles at a collision point
     * @param worldPos Position where splash occurs
     * @param velocity Initial velocity of the colliding object
     * @param intensity How many particles to emit (0.0-1.0)
     */
    void emitSplash(const linalg::aliases::float3& worldPos,
                    const linalg::aliases::float3& velocity,
                    float intensity = 1.0F);

    /**
     * @brief Update all particles
     * @param deltaTime Time step
     * @param waterHeight Function to get water height at position
     */
    void update(float deltaTime,
                const std::function<float(const linalg::aliases::float3& pos)>&
                  getWaterHeight);

    /**
     * @brief Get all alive particles for rendering
     */
    [[nodiscard]] const std::vector<Particle>& getParticles() const;

    /**
     * @brief Get number of alive particles
     */
    [[nodiscard]] size_t getAliveCount() const;

    /**
     * @brief Clear the particle vector
     */
    void clear();
};

class Camera;
struct InstanceData;

/**
 * @class ParticleRenderer
 * @ingroup frontend
 * @brief Simple renderer for particles using instanced rendering
 */
class ParticleRenderer
{
    uint32_t VAO = 0;
    uint32_t VBO = 0;
    uint32_t instanceVBO = 0;

    std::vector<InstanceData> instanceData;

    Shader particleShader{
      std::filesystem::path{"shaders/vertex/particleVertex.glsl"},
      std::filesystem::path{"shaders/fragment/particleFragment.glsl"},
      {"view", "projection", "cameraPos"}
    };

  public:
    ParticleRenderer();

    ~ParticleRenderer();

    /**  @brief Draw the particles with the shader */
    void draw(const ParticleSystem& particles, const Camera& cam);
};
