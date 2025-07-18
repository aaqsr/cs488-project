#pragma once

#include "GL/glew.h"
#include "linalg.h"

#include <filesystem>
#include <string>

class Shader
{
    friend class BindObject;

    void linkProgram(uint32_t vertexShader, uint32_t geoShader, uint32_t fragmentShader);
    void validateUniforms(const std::vector<std::string>& requiredUniforms);

    uint32_t programId{};

    bool isBound = false;

  public:
    struct UniformInfo
    {
        GLint location;
        GLenum type;
        std::string name;
    };

  private:
    std::unordered_map<std::string, UniformInfo> validUniforms;

    void loadFromSource(const std::string& vertexSource,
                        const std::string& geoSource,
                        const std::string& fragmentSource,
                        const std::vector<std::string>& requiredUniforms = {});

    void loadFromFile(const std::filesystem::path& vertexPath,
                      const std::filesystem::path& geoPath,
                      const std::filesystem::path& fragmentPath,
                      const std::vector<std::string>& requiredUniforms = {});

  public:
    // needs a better name tbh
    class BindObject
    {
        friend class Shader;

        uint32_t programId;
        Shader& shader;

        BindObject(uint32_t programId, Shader& shader);

      public:
        BindObject(const BindObject&) = delete;
        BindObject(BindObject&&) = delete;
        BindObject& operator=(const BindObject&) = delete;
        BindObject& operator=(BindObject&&) = delete;

        ~BindObject();

        // Man I wish these were templatable
        // TODO: Better way to structure this? Need better way to check if
        // uniform is valid
        void setUniformInt(const std::string& name, int value);
        void setUniform(const std::string& name, float value);
        void setUniform(const std::string& name,
                        const linalg::aliases::float2& value);
        void setUniform(const std::string& name,
                        const linalg::aliases::float3& value);
        void setUniform(const std::string& name,
                        const linalg::aliases::float4& value);
        void setUniform(const std::string& name,
                        const linalg::aliases::float4x4& value);
        [[nodiscard]] const UniformInfo&
        getUniformInfo(const std::string& name) const;
        [[nodiscard]] bool hasUniform(const std::string& name) const;
    };

    Shader(const std::string& vertexSource, const std::string& fragmentSource,
           const std::vector<std::string>& requiredUniforms = {});

    Shader(const std::filesystem::path& vertexPath,
           const std::filesystem::path& fragmentPath,
           const std::vector<std::string>& requiredUniforms = {});

    Shader(const std::filesystem::path& vertexPath,
           const std::filesystem::path& geoPath,
           const std::filesystem::path& fragmentPath,
           const std::vector<std::string>& requiredUniforms = {});

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&&) = delete;
    Shader& operator=(Shader&&) = delete;

    ~Shader();

    [[nodiscard]] BindObject bind();
    [[nodiscard]] bool validateUniformExists(const std::string& name) const;
};
