#include "window.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

Window::Window()
{
    if (glfwInit() == 0) {
        throw std::runtime_error{"Failed to initialize GLFW."};
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for apple

    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        throw std::runtime_error{"Failed to create GLFW window"};
    }

    glfwMakeContextCurrent(window);

    // TODO: what is glew for??
    glewExperimental = 1;
    if (glewInit() != GLEW_OK) {
        glfwTerminate();
        throw std::runtime_error{"Failed to initialize GLEW."};
    }

    // Enable VSync
    // glfwSwapInterval(1);

    // Callback when window resizes
    // glfwSetFramebufferSizeCallback(
    //   window, [](GLFWwindow* window, int width, int height) {
    //   });

    // Use framebuffer size, not window size for viewport.
    // Why? bcs they might differ...sigh (example: on HiDPI or Retina
    // displays, framebuffer is typically 2x the window size)
    int framebufferWidth = width;
    int framebufferHeight = height;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    glViewport(0, 0, framebufferWidth, framebufferHeight);
}

Window::~Window()
{
    if (window != nullptr) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

bool Window::shouldClose() const
{
    return glfwWindowShouldClose(window) == GL_TRUE;
}

void Window::swapBuffers()
{
    glfwSwapBuffers(window);
}
