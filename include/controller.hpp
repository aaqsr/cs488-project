#pragma once

#include "camera.hpp"
#include "singleton.hpp"

struct GLFWwindow;

class Controller : public Singleton<Controller>
{
    friend class Singleton<Controller>;

    GLFWwindow* window;
    // TODO: Make this safe for use after free...
    Camera* camera = nullptr;

    // Mouse state
    bool inputCaptured = false;
    double lastMouseX = 0.0;
    double lastMouseY = 0.0;
    bool firstMouse = true;

    // Movement settings
    float moveSpeed = 3.0F;
    float mouseSensitivity = 0.1F;

    // Static callbacks
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action,
                                    int mods);
    static void keyCallback(GLFWwindow* window, int key, int scancode,
                            int action, int mods);

    Controller();

  public:
    ~Controller() override = default;

    void setMainCamera(Camera* cam)
    {
        camera = cam;
    }
    void update(double deltaTime);
    void captureMouse();
    void releaseMouse();
};
