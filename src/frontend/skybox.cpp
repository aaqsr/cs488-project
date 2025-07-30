#include "frontend/skybox.hpp"

#include "frontend/camera.hpp"
#include "frontend/shader.hpp"
#include "linalg.h"
#include "util/error.hpp"
#include "util/logger.hpp"

#include <stb_image.h>
#include <string>

namespace
{
} // namespace

Skybox::Skybox()
{
    {
        // textures
        glGenTextures(1, &textureId);
        glBindTexture(GL_TEXTURE_CUBE_MAP, textureId);
        // prevent seams
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S,
                        GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T,
                        GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R,
                        GL_CLAMP_TO_EDGE);
        // TODO:
        // try GL_TEXTURE_CUBE_MAP_SEAMLESS but only if can find a way to make
        // it cross platform
        // flip texture vertically for OpenGL
        stbi_set_flip_vertically_on_load(0);

        int i = -1;
        for (const auto& path : skyboxTexturePaths) {
            ++i;
            int width = 0;
            int height = 0;
            int channels = 0;
            unsigned char* data =
              stbi_load(path.string().c_str(), &width, &height, &channels, 0);

            if (data == nullptr) {
                stbi_image_free(data);
                throw IrrecoverableError(
                  "Failed to load skybox texture: " + path.string() + " - " +
                  std::string(stbi_failure_reason()));
            }

            // fine format based on channels
            GLenum format = GL_RGB;
            GLenum internalFormat = GL_RGB;

            switch (channels) {
                case 1: format = internalFormat = GL_RED; break;
                case 2: format = internalFormat = GL_RG; break;
                case 3:
                    format = GL_RGB;
                    internalFormat = GL_SRGB;
                    break;
                case 4:
                    format = GL_RGBA;
                    internalFormat = GL_SRGB_ALPHA;
                    break;
                default:
                    stbi_image_free(data);
                    throw IrrecoverableError(
                      "Unsupported Skybox texture format at " + path.string());
            }

            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, internalFormat,
                         width, height, 0, format, GL_UNSIGNED_BYTE, data);

            stbi_image_free(data);
        }
    }

    {
        // VAO, VBO, EBO
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * skyboxVertices.size(),
                     skyboxVertices.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     sizeof(int) * skyboxIndices.size(), skyboxIndices.data(),
                     GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                              nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }

    {
        Shader::BindObject boundShader = skyboxShader.bind();
        setSkyboxSamplerUniform(boundShader);
    }
}

Skybox::~Skybox()
{
    if (VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
    }
    if (textureId != 0) {
        glDeleteTextures(1, &textureId);
    }
}

void Skybox::setSkyboxSamplerUniform(Shader::BindObject& shader)
{
    shader.setUniformInt("skybox", 0);
}

void Skybox::setUniformsAndDraw(const Camera& mainCamera)
{
    // depth buffer fixed at 1.0 for skybox
    // so need to make sure the skybox passes the depth tests with values less
    // than or equal to the depth buffer
    glDepthFunc(GL_LEQUAL);

    // glEnable(GL_FRAMEBUFFER_SRGB);

    linalg::aliases::float4x4 viewMatRaw = mainCamera.getViewMatrix();
    linalg::aliases::float3x3 viewMatCropped{
      viewMatRaw.x.xyz(), viewMatRaw.y.xyz(), viewMatRaw.z.xyz()};
    linalg::aliases::float4x4 viewMat{
      {viewMatCropped.x, 0.0F},
      {viewMatCropped.y, 0.0F},
      {viewMatCropped.z, 0.0F},
      {0.0F, 0.0F, 0.0F, 1.0F}
    };

    if (textureId == 0) {
        Logger::GetInstance().log("Skybox texture not loaded");
        return;
    }

    linalg::aliases::float4x4 perspMat = mainCamera.getPerspectiveMatrix();

    {
        Shader::BindObject boundShader = skyboxShader.bind();
        boundShader.setUniform("view", viewMat);
        boundShader.setUniform("projection", perspMat);

        glBindVertexArray(VAO);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, textureId);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(skyboxIndices.size()),
                       GL_UNSIGNED_INT, nullptr);
    }

    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        Logger::GetInstance().log("Skybox Draw Error Code " +
                                  std::to_string(error));
    }

    // Clean up
    glBindVertexArray(0);
    glDepthFunc(GL_LESS);
}
