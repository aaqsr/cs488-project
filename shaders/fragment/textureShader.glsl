#version 330 core

in vec2 texCoord;
out vec4 FragColour;

// built-in data-type for texture objects called a sampler that
// takes as a postfix the texture type we want. if we bind the
// texture before calling glDrawElements, it will automatically
// assign the texture to the fragment shader's sampler.
uniform sampler2D theTexture;

void main()
{
    FragColour = texture(theTexture, texCoord);
}
