#pragma once

#include <filesystem>
#include <string>

class Shader
{
  public:
    Shader(const std::string& vertexSource, const std::string& fragmentSource);

    Shader(const std::filesystem::path& vertexPath,
           const std::filesystem::path& fragmentPath);

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&&) = delete;
    Shader& operator=(Shader&&) = delete;

    ~Shader();

    void loadFromSource(const std::string& vertexSource,
                        const std::string& fragmentSource);

    void loadFromFile(const std::filesystem::path& vertexPath,
                      const std::filesystem::path& fragmentPath);

    void use() const;

    [[nodiscard]] uint32_t getId() const
    {
        return programId;
    }

  private:
    void linkProgram(uint32_t vertexShader, uint32_t fragmentShader);
    uint32_t programId{};
};
