#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdexcept>

int main()
{
    if (glfwInit() == 0) {
        throw std::runtime_error{"Failed to initialize GLFW."};
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "CS488", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        throw std::runtime_error{"Failed to create GLFW window"};
    }

    glfwMakeContextCurrent(window);

    glewExperimental = 1;
    if (glewInit() != GLEW_OK) {
        glfwTerminate();
        throw std::runtime_error{"Failed to initialize GLEW."};
    }

    while (glfwWindowShouldClose(window) == GL_FALSE) {
        // Double buffering.
        // The front buffer contains the final output image that is
        // shown at the screen. Whilst all the rendering commands
        // draw to the back buffer. We swap the back buffer to the
        // front buffer so the image can be displayed without still
        // being rendered to.
        glfwSwapBuffers(window);

        glfwPollEvents();
    }
}
