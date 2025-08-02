#version 330 core

uniform vec4 colour;
out vec4 FragColour;

void main()
{
    FragColour = colour;
}
