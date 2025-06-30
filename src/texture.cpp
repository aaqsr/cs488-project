#include "texture.hpp"
#include "stb_image.h"

#include <gl/glew.h>
#include <math.h>

#include "error.hpp"

namespace
{
void setTextureParameters()
{
    // wrapping params
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // filtering params
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // anisotropic filtering ((if available))
    if (glewIsSupported("GL_EXT_texture_filter_anisotropic") != 0) {
        GLfloat maxAnisotropy = NAN;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAnisotropy);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT,
                        maxAnisotropy);
    }
}
} // namespace

Texture::Texture(const std::filesystem::path& path)
{
    loadFromFile(path);
}

// whyyyyy did I write this ( i had to :( )
Texture::Texture(Texture&& other) noexcept
  : textureId(other.textureId), width(other.width), height(other.height),
    channels(other.channels), filePath(std::move(other.filePath))
{
    other.textureId = 0;
    other.width = 0;
    other.height = 0;
    other.channels = 0;
}

Texture& Texture::operator=(Texture&& other) noexcept
{
    // TODO: wait what why am i doing this, i should just be swapping the two...
    if (this != &other) {
        if (textureId != 0) {
            glDeleteTextures(1, &textureId);
        }

        textureId = other.textureId;
        width = other.width;
        height = other.height;
        channels = other.channels;
        filePath = std::move(other.filePath);

        other.textureId = 0;
        other.width = 0;
        other.height = 0;
        other.channels = 0;
    }
    return *this;
}

Texture::~Texture()
{
    if (textureId != 0) {
        glDeleteTextures(1, &textureId);
    }
}

void Texture::loadFromFile(const std::filesystem::path& path)
{
    if (textureId != 0) {
        glDeleteTextures(1, &textureId);
        textureId = 0;
    }

    filePath = path.string();

    // flip texture vertically for OpenGL
    stbi_set_flip_vertically_on_load(1);

    unsigned char* data =
      stbi_load(filePath.c_str(), &width, &height, &channels, 0);
    if (data == nullptr) {
        throw IrrecoverableError("Failed to load texture: " + filePath + " - " +
                                 std::string(stbi_failure_reason()));
    }

    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);

    // fine format based on channels
    GLenum format = GL_RGB;
    GLenum internalFormat = GL_RGB;

    switch (channels) {
        case 1: format = internalFormat = GL_RED; break;
        case 2: format = internalFormat = GL_RG; break;
        case 3: format = internalFormat = GL_RGB; break;
        case 4: format = internalFormat = GL_RGBA; break;
        default:
            stbi_image_free(data);
            throw IrrecoverableError("Unsupported texture format with " +
                                     std::to_string(channels) +
                                     " channels: " + filePath);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, format,
                 GL_UNSIGNED_BYTE, data);

    setTextureParameters();

    glGenerateMipmap(GL_TEXTURE_2D); // mipmaps!

    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::bind(GLuint textureUnit) const
{
    if (!isValid()) {
        return;
    }

    // texture units allow us to use more than 1 texture in our shaders.
    // minimum of 16.
    // texture unit number tells openGL which sampler uniform to set when you
    // bind the texture.
    glActiveTexture(GL_TEXTURE0 + textureUnit);
    glBindTexture(GL_TEXTURE_2D, textureId);
}

void Texture::unbind() const
{
    glBindTexture(GL_TEXTURE_2D, 0);
}
