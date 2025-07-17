#pragma once

#include "util/singleton.hpp"
#include <atomic>

struct GLFWwindow;
class WaterSimulation;
class Camera;

class Controller : public Singleton<Controller>
{
    friend class Singleton<Controller>;

    GLFWwindow* window;
    // TODO: Make this safe for use after free...
    Camera* camera = nullptr;
    std::atomic<std::atomic<bool>*> isPlaying_ptr;

    // Mouse state
    bool inputCaptured = false;
    double lastMouseX = 0.0;
    double lastMouseY = 0.0;
    bool firstMouse = true;

    // but, but, we used quaternions?? well dear reader, this is here to
    // prevent you from being able to look around the world. We do dispatch out
    // to quaternions to still prevent gimbal lock though...
    float pitch = 0.0F; // in radians

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

    void setMainCamera(Camera* cam);
    void setIsPlayingBoolPtr(std::atomic<bool>* isPlayingBool_ptr);

    void toggleIsPlaying();

    void captureMouse();
    void releaseMouse();

    void update(double deltaTime);
};
