#pragma once

#include "util/singleton.hpp"

struct GLFWwindow;

class Window : public Singleton<Window>
{
    friend class Singleton<Window>;

    GLFWwindow* window;

    Window();

  public:
    inline static int width = 800;
    inline static int height = 600;

    inline static const char* title = "CS488";

    Window(const Window&) = delete;
    Window(Window&&) = delete;
    Window& operator=(const Window&) = delete;
    Window& operator=(Window&&) = delete;

    ~Window() override;

    GLFWwindow* getWindow()
    {
        return window;
    }

    [[nodiscard]] bool shouldClose() const;

    void swapBuffers();
};
