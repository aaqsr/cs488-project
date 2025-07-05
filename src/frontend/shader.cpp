#include "frontend/shader.hpp"
#include "util/error.hpp"
#include "util/logger.hpp"

#include <fstream>
#include <sstream>
#include <string>

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
        glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog.data());

        const char* shaderType =
          (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT";

        glDeleteShader(shader);

        throw IrrecoverableError{std::string{"ERROR: "} + shaderType +
                                 " shader compilation failed:\n" +
                                 infoLog.data()};
    }

    return shader;
}

std::string readFile(const std::filesystem::path& filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw IrrecoverableError{"ERROR: Could not open file: " +
                                 filePath.string()};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    return buffer.str();
}
} // namespace

Shader::Shader(const std::string& vertexSource,
               const std::string& fragmentSource,
               const std::vector<std::string>& requiredUniforms)
{
    loadFromSource(vertexSource, fragmentSource, requiredUniforms);
}

Shader::Shader(const std::filesystem::path& vertexPath,
               const std::filesystem::path& fragmentPath,
               const std::vector<std::string>& requiredUniforms)
{
    loadFromFile(vertexPath, fragmentPath, requiredUniforms);
}

Shader::~Shader()
{
    if (programId != 0) {
        glDeleteProgram(programId);
    }
}

void Shader::loadFromSource(const std::string& vertexSource,
                            const std::string& fragmentSource,
                            const std::vector<std::string>& requiredUniforms)
{
    GLuint vertexShader = compileShader(vertexSource, GL_VERTEX_SHADER);
    if (vertexShader == 0) {
        throw IrrecoverableError{"Vertex shader failed to compile"};
    }

    GLuint fragmentShader = compileShader(fragmentSource, GL_FRAGMENT_SHADER);
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        throw IrrecoverableError{"Fragment shader failed to compile"};
    }

    try {
        linkProgram(vertexShader, fragmentShader);
        validateUniforms(requiredUniforms);
    } catch (IrrecoverableError& e) {
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        if (programId != 0) {
            glDeleteProgram(programId);
            programId = 0;
        }
        throw e;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Shader::loadFromFile(const std::filesystem::path& vertexPath,
                          const std::filesystem::path& fragmentPath,
                          const std::vector<std::string>& requiredUniforms)
{
    std::string vertexSource = readFile(vertexPath);
    std::string fragmentSource = readFile(fragmentPath);

    if (vertexSource.empty() || fragmentSource.empty()) {
        throw IrrecoverableError{"Failed to read shader file"};
    }

    loadFromSource(vertexSource, fragmentSource, requiredUniforms);
}

Shader::BindObject Shader::bind()
{
    if (isBound) {
        throw IrrecoverableError{"Attempt to bind shader twice"};
    }

    isBound = true;

    return BindObject{programId, *this};
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
                            infoLog.data());

        glDeleteProgram(programId);
        programId = 0;

        throw IrrecoverableError{
          std::string{"ERROR: Shader program linking failed:\n"} +
          infoLog.data()};
    }
}

void Shader::validateUniforms(const std::vector<std::string>& requiredUniforms)
{
    validUniforms.clear();

    // For each required uniform, check if it exists in the compiled shader
    for (const auto& uniformName : requiredUniforms) {
        GLint location = glGetUniformLocation(programId, uniformName.c_str());

        if (location == -1) {
            throw IrrecoverableError{std::string{"Required uniform '"} +
                                     uniformName +
                                     "' not found in shader program"};
        }

        // Get the uniform's type info
        GLint uniformCount = 0;
        glGetProgramiv(programId, GL_ACTIVE_UNIFORMS, &uniformCount);

        GLenum uniformType = GL_FLOAT; // Default fallback
        bool found = false;

        for (GLint i = 0; i < uniformCount && !found; ++i) {
            std::array<char, 256> name{};
            GLsizei length = 0;
            GLint size = 0;
            GLenum type = 0;

            glGetActiveUniform(programId, static_cast<GLuint>(i), name.size(),
                               &length, &size, &type, name.data());

            if (length > 0) {
                std::string activeUniformName(name.data(), length);
                if (activeUniformName == uniformName) {
                    uniformType = type;
                    found = true;
                }
            }
        }

        UniformInfo info{location, uniformType, uniformName};
        validUniforms[uniformName] = info;
    }
}

bool Shader::validateUniformExists(const std::string& name) const
{

    if (!validUniforms.contains(name)) {
        // Logger::GetInstance().log("ERROR: Uniform '" + name +
        //                           "' does not exist in shader program");
        return false;
    }
    return true;
}

const Shader::UniformInfo&
Shader::BindObject::getUniformInfo(const std::string& name) const
{
    try {
        return shader.validUniforms.at(name);
    } catch (...) {
        throw IrrecoverableError{"Uniform '" + name +
                                 "' does not exist in shader program"};
    }
}

void Shader::BindObject::setUniformInt(const std::string& name, int value)
{
    if (!shader.validateUniformExists(name)) {
        return;
    }
    const UniformInfo& info = shader.validUniforms.at(name);
    glUniform1i(info.location, value);
}

void Shader::BindObject::setUniform(const std::string& name, float value)
{
    if (!shader.validateUniformExists(name)) {
        return;
    }
    const UniformInfo& info = shader.validUniforms.at(name);
    glUniform1f(info.location, value);
}

void Shader::BindObject::setUniform(const std::string& name,
                                    const linalg::aliases::float2& value)
{
    if (!shader.validateUniformExists(name)) {
        return;
    }
    const UniformInfo& info = shader.validUniforms.at(name);
    glUniform2f(info.location, value.x, value.y);
}

void Shader::BindObject::setUniform(const std::string& name,
                                    const linalg::aliases::float3& value)
{
    if (!shader.validateUniformExists(name)) {
        return;
    }
    const UniformInfo& info = shader.validUniforms.at(name);
    glUniform3f(info.location, value.x, value.y, value.z);
}

void Shader::BindObject::setUniform(const std::string& name,
                                    const linalg::aliases::float4& value)
{
    if (!shader.validateUniformExists(name)) {
        return;
    }
    const UniformInfo& info = shader.validUniforms.at(name);
    glUniform4f(info.location, value.x, value.y, value.z, value.w);
}

void Shader::BindObject::setUniform(const std::string& name,
                                    const linalg::aliases::float4x4& value)
{
    if (!shader.validateUniformExists(name)) {
        return;
    }
    const UniformInfo& info = shader.validUniforms.at(name);
    // TODO: Is memory layout correct here??
    glUniformMatrix4fv(info.location, 1, GL_FALSE, &value.x.x);
}

Shader::BindObject::BindObject(uint32_t programId, Shader& shader)
  : programId{programId}, shader{shader}
{
    glUseProgram(programId);
}

Shader::BindObject::~BindObject()
{
    shader.isBound = false;
    glUseProgram(0);
}
