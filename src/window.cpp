#include "window.hpp"
#include "error.hpp"

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

    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for apple

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

    //
    // OpenGL configuration
    //
    glEnable(GL_CULL_FACE);  // don't draw back faces
    glEnable(GL_DEPTH_TEST); // Depth buffer
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE); // allow writing to depth buffer
    glDepthRange(0.0, 1.0);
    glClearDepth(1.0); // clear depth buffer to 1.0 (far plane)

    // Disable VSync for them FPS wooooo (TODO: good idea??)
    glfwSwapInterval(0);

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
