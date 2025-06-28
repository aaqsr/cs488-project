#pragma once

#include "singleton.hpp"

struct GLFWwindow;

class Window : public Singleton<Window>
{
    static constexpr int width = 800;
    static constexpr int height = 600;
    inline static const char* title = "CS488";

    GLFWwindow* window;

  public:
    Window();

    Window(const Window&) = delete;
    Window(Window&&) = delete;
    Window& operator=(const Window&) = delete;
    Window& operator=(Window&&) = delete;

    ~Window();

    GLFWwindow* getWindow()
    {
        return window;
    }

    [[nodiscard]] bool shouldClose() const;

    void swapBuffers();
};
