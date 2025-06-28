#include "shader.hpp"

#include <GL/glew.h>

#include <fstream>
#include <sstream>

namespace
{
[[nodiscard]] uint32_t compileShader(const std::string& source, GLenum type)
{
    GLuint shader = glCreateShader(type);
    const char* sourceCStr = source.c_str();

    glShaderSource(shader, 1, &sourceCStr, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

    if (success == 0) {
        std::array<GLchar, 1024> infoLog{};
        glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog.begin());

        const char* shaderType =
          (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT";

        glDeleteShader(shader);

        throw std::runtime_error{std::string{"ERROR: "} + shaderType +
                                 " shader compilation failed:\n" +
                                 infoLog.begin()};
    }

    return shader;
}

std::string readFile(const std::filesystem::path& filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error{"ERROR: Could not open file: " +
                                 filePath.string()};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    return buffer.str();
}
} // namespace

Shader::Shader(const std::string& vertexSource,
               const std::string& fragmentSource)
{
    loadFromSource(vertexSource, fragmentSource);
}

Shader::Shader(const std::filesystem::path& vertexPath,
               const std::filesystem::path& fragmentPath)
{
    loadFromFile(vertexPath, fragmentPath);
}

Shader::~Shader()
{
    if (programId != 0) {
        glDeleteProgram(programId);
    }
}

void Shader::loadFromSource(const std::string& vertexSource,
                            const std::string& fragmentSource)
{
    GLuint vertexShader = compileShader(vertexSource, GL_VERTEX_SHADER);
    if (vertexShader == 0) {
        throw std::runtime_error{"Vertex shader failed to compile"};
    }

    GLuint fragmentShader = compileShader(fragmentSource, GL_FRAGMENT_SHADER);
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        throw std::runtime_error{"Fragment shader failed to compile"};
    }

    try {
        linkProgram(vertexShader, fragmentShader);
    } catch (...) {
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        throw;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Shader::loadFromFile(const std::filesystem::path& vertexPath,
                          const std::filesystem::path& fragmentPath)
{
    std::string vertexSource = readFile(vertexPath);
    std::string fragmentSource = readFile(fragmentPath);

    if (vertexSource.empty() || fragmentSource.empty()) {
        throw std::runtime_error{"Failed to read shader file"};
    }

    loadFromSource(vertexSource, fragmentSource);
}

void Shader::use() const
{
    glUseProgram(programId);
}

void Shader::linkProgram(GLuint vertexShader, GLuint fragmentShader)
{
    programId = glCreateProgram();

    glAttachShader(programId, vertexShader);
    glAttachShader(programId, fragmentShader);
    glLinkProgram(programId);

    GLint success = 0;
    glGetProgramiv(programId, GL_LINK_STATUS, &success);

    if (success == 0) {
        std::array<GLchar, 1024> infoLog{};
        glGetProgramInfoLog(programId, sizeof(infoLog), nullptr,
                            infoLog.begin());

        glDeleteProgram(programId);
        programId = 0;

        throw std::runtime_error{
          std::string{"ERROR: Shader program linking failed:\n"} +
          infoLog.begin()};
    }
}
