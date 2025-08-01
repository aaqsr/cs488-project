#pragma once

#include "util/singleton.hpp"

#include <linalg.h>

#include <atomic>

struct GLFWwindow;
class WaterSimulation;
class Camera;
template <typename T>
class Sender;
struct PhysicsEngineReceiverData;

class Controller : public Singleton<Controller>
{
    friend class Singleton<Controller>;

    GLFWwindow* window;
    // TODO: Make this safe for use after free...
    Camera* camera = nullptr;
    std::atomic<std::atomic<bool>*> isPlaying_ptr;

    // Channel for sending physics commands (like throwing bottles [whaaa no
    // way])
    Sender<std::vector<PhysicsEngineReceiverData>>* physCmdChannel = nullptr;

    // Mouse state
    bool inputCaptured = false;
    double lastMouseX = 0.0;
    double lastMouseY = 0.0;
    bool firstMouse = true;
    float mouseSensitivity = 0.1F;

    // but, but, we used quaternions?? well dear reader, this is here to
    // prevent you from being able to look around the world. We do dispatch out
    // to quaternions to still prevent gimbal lock though...
    float pitch = 0.0F; // in radians

    // Movement settings
    struct
    {
        bool forward = false;
        bool backward = false;
        bool left = false;
        bool right = false;
        bool up = false;
        bool down = false;
    } moveKeyState;
    linalg::aliases::float3 moveVelocity{0.0F, 0.0F, 0.0F};
    constexpr static float moveAccel = 15.0F;
    constexpr static float moveDeAccel = 10.0F;
    constexpr static float moveMaxSpeed = 4.5F;
    constexpr static float moveMomentumDamping = 0.9F;

    void updateKeyboardState();
    linalg::aliases::float3 calculateDesiredMovement() const;
    void
    updateVelocityWithMomentum(const linalg::aliases::float3& desiredDirection,
                               double deltaTime);

    // Bottle throwing settings
    constexpr static float throwSpeed = 4.0F;
    constexpr static float throwSpinSpeed = 3.0F;
    void throwBottle();

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
    void setPhysicsCommandChannel(
      Sender<std::vector<PhysicsEngineReceiverData>>* channel);

    void toggleIsPlaying();

    void captureMouse();
    void releaseMouse();

    void update(double deltaTime);
};
