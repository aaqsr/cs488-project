#pragma once

#include <linalg.h>
#include <GL/glew.h>

#include <filesystem>
#include <string>

class Shader
{
  public:
    Shader(const std::string& vertexSource, const std::string& fragmentSource,
           const std::vector<std::string>& requiredUniforms = {});

    Shader(const std::filesystem::path& vertexPath,
           const std::filesystem::path& fragmentPath,
           const std::vector<std::string>& requiredUniforms = {});

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&&) = delete;
    Shader& operator=(Shader&&) = delete;

    ~Shader();

    void loadFromSource(const std::string& vertexSource,
                        const std::string& fragmentSource,
                        const std::vector<std::string>& requiredUniforms = {});

    void loadFromFile(const std::filesystem::path& vertexPath,
                      const std::filesystem::path& fragmentPath,
                      const std::vector<std::string>& requiredUniforms = {});

    void use() const;

    [[nodiscard]] uint32_t getId() const;
    [[nodiscard]] bool hasUniform(const std::string& name) const;

    struct UniformInfo
    {
        GLint location;
        GLenum type;
        std::string name;
    };

    [[nodiscard]] const UniformInfo&
    getUniformInfo(const std::string& name) const;

    // Man I wish these were templatable
    void setUniform(const std::string& name, float value);
    void setUniform(const std::string& name, const linalg::aliases::float2& value);
    void setUniform(const std::string& name, const linalg::aliases::float3& value);
    void setUniform(const std::string& name, const linalg::aliases::float4& value);
    void setUniform(const std::string& name, const linalg::aliases::float4x4& value);

  private:
    void linkProgram(uint32_t vertexShader, uint32_t fragmentShader);
    void validateUniforms(const std::vector<std::string>& requiredUniforms);
    void validateUniformExists(const std::string& name) const;

    uint32_t programId{};
    std::unordered_map<std::string, UniformInfo> validUniforms;
};
