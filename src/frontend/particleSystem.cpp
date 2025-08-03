#include "frontend/particleSystem.hpp"
#include "frontend/camera.hpp"

struct Particle
{
    linalg::aliases::float3 position;
    linalg::aliases::float3 velocity;
    float life = 1.0F;    // 0.0 = dead, 1.0 = full life
    float maxLife = 1.0F; // initial life value
    float size = 0.02F;   // particle size

    [[nodiscard]] bool isAlive() const
    {
        return life > 0.0F;
    }
};

struct InstanceData
{
    linalg::aliases::float3 position;
    float size;
    float alpha;
};

ParticleSystem::ParticleSystem()
{
    particles.reserve(maxParticles);
}

void ParticleSystem::emitSplash(const linalg::aliases::float3& worldPos,
                                const linalg::aliases::float3& velocity,
                                float intensity)
{
    int numParticles = static_cast<int>((intensity * 30.0F) + 5.0F);

    for (int i = 0; i < numParticles && particles.size() < maxParticles; ++i) {
        Particle p;
        p.position = worldPos;
        p.position.y += 0.02F; // Start slightly above water surface

        // Random radial direction
        float angle = CS488Math::randInRange(0.0F, 2.0F * std::numbers::pi);
        float speed = CS488Math::randInRange(0.5F, 3.0F);
        float upward = CS488Math::randInRange(0.5F, 2.0F);
        float size = CS488Math::randInRange(0.008F, 0.02F);
        float life = CS488Math::randInRange(1.2F, 2.5F);

        p.velocity.x = std::cos(angle) * speed;
        p.velocity.z = std::sin(angle) * speed;
        p.velocity.y = upward;

        // Add some of the object's velocity
        p.velocity += velocity * 0.3F;

        p.maxLife = p.life = life;
        p.size = size;

        particles.push_back(p);
    }
}

const std::vector<Particle>& ParticleSystem::getParticles() const
{
    return particles;
}

size_t ParticleSystem::getAliveCount() const
{
    return std::count_if(particles.begin(), particles.end(),
                         [](const Particle& p) { return p.isAlive(); });
}

void ParticleSystem::clear()
{
    particles.clear();
}

ParticleRenderer::ParticleRenderer()
{
    instanceData.reserve(2000);

    // Simple quad vertices for billboard particles
    float vertices[] = {-0.5F, -0.5F, 0.0F, 0.5F,  -0.5F, 0.0F,
                        0.5F,  0.5F,  0.0F, -0.5F, 0.5F,  0.0F};

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &instanceVBO);

    glBindVertexArray(VAO);

    // Vertex positions
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          (void*)0);

    // Instance data
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    // Position (per instance)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                          (void*)0);
    glVertexAttribDivisor(1, 1);
    // Size (per instance)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                          (void*)(3 * sizeof(float)));
    glVertexAttribDivisor(2, 1);
    // Alpha (per instance)
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(InstanceData),
                          (void*)(4 * sizeof(float)));
    glVertexAttribDivisor(3, 1);

    glBindVertexArray(0);
}

void ParticleSystem::update(
  float deltaTime,
  const std::function<float(const linalg::aliases::float3& pos)>&
    getWaterHeight)
{
    for (auto it = particles.begin(); it != particles.end();) {
        Particle& p = *it;

        if (!p.isAlive()) {
            it = particles.erase(it);
            continue;
        }

        // simple physics update
        p.velocity.y +=
          -Physics::gravitationalAccelerationMagnitude * deltaTime;
        p.position += p.velocity * deltaTime;

        // check collision with water surface
        float waterSurfaceHeight = getWaterHeight(p.position);
        if (p.position.y <= waterSurfaceHeight) {
            // Particle hits water - kill it or make it bounce slightly
            if (p.velocity.y < -0.5F) // Fast downward velocity
            {
                p.life = 0.0F; // Kill particle
            } else {
                // Small bounce
                p.position.y = waterSurfaceHeight + 0.001F;
                p.velocity.y = std::abs(p.velocity.y) * 0.3F;
                p.velocity.x *= 0.7F;
                p.velocity.z *= 0.7F;
            }
        }

        // age particle
        p.life -= deltaTime;

        ++it;
    }
}

ParticleRenderer::~ParticleRenderer()
{
    if (VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &instanceVBO);
    }
}

void ParticleRenderer::draw(const ParticleSystem& particles, const Camera& cam)

{
    auto shader = particleShader.bind();

    cam.setUniforms(shader);
    shader.setUniform("cameraPos", cam.getPosition());

    instanceData.clear();

    for (const auto& particle : particles.getParticles()) {
        if (!particle.isAlive()) {
            continue;
        }

        InstanceData data;
        data.position = particle.position;
        data.size = particle.size;
        data.alpha = particle.life / particle.maxLife; // Fade out over time

        instanceData.push_back(data);
    }

    if (instanceData.empty()) {
        return;
    }

    // Update instance buffer
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, instanceData.size() * sizeof(InstanceData),
                 instanceData.data(), GL_DYNAMIC_DRAW);

    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE); // Don't write to depth buffer plz

    // Render
    glBindVertexArray(VAO);
    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4,
                          static_cast<GLsizei>(instanceData.size()));

    // Restore state
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}
