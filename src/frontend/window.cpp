#include "frontend/window.hpp"
#include "util/error.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

Window::Window()
{
    if (glfwInit() == 0) {
        throw IrrecoverableError{"Failed to initialize GLFW."};
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for apple
#endif

    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        throw IrrecoverableError{"Failed to create GLFW window"};
    }

    glfwMakeContextCurrent(window);

    glewExperimental = 1;
    if (glewInit() != GLEW_OK) {
        glfwTerminate();
        throw IrrecoverableError{"Failed to initialize GLEW."};
    }

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
