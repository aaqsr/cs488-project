#pragma once

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

  public:
    Texture() = default;
    explicit Texture(const std::filesystem::path& path);

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    Texture(Texture&& other) noexcept;
    Texture& operator=(Texture&& other) noexcept;

    ~Texture();

    void loadFromFile(const std::filesystem::path& path);
    void bind(uint32_t textureUnit = 0) const;
    void unbind() const;

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
