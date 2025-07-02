#pragma once

#include "shader.hpp"

#include <filesystem>
#include <string>

// Does not actually store the image file inside!
// It gets copied over to the GPU memory.
class Texture
{
    uint32_t textureId = 0;
    int width = 0;
    int height = 0;
    int channels = 0;
    std::string filePath;

    void loadFromFile(const std::filesystem::path& path);
    void loadFromData(const unsigned char* data, int dataWidth, int dataHeight,
                      GLenum format, GLenum internalFormat);

  public:
    Texture() = default;
    explicit Texture(const std::filesystem::path& path);

    // `data` is borrowed here, caller must clean-up after the function.
    // We assume internal format and format of `GL_RGB`.
    Texture(int width, int height, const unsigned char* data);

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    ~Texture();

    // USER IS RESPONSIBLE FOR HAVING BOUND THE SHADER
    // TODO: Need to make a convention for this or something
    void bind(Shader& shader, const std::string& textureUniformName,
              GLuint textureUnit) const;
    void unbind(GLuint textureUnit = 0) const;

    [[nodiscard]] uint32_t getId() const
    {
        return textureId;
    }
    [[nodiscard]] int getWidth() const
    {
        return width;
    }
    [[nodiscard]] int getHeight() const
    {
        return height;
    }
    [[nodiscard]] int getChannels() const
    {
        return channels;
    }
    [[nodiscard]] const std::string& getFilePath() const
    {
        return filePath;
    }
    [[nodiscard]] bool isValid() const
    {
        return textureId != 0;
    }
};
