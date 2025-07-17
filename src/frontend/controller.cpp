#include "frontend/controller.hpp"

#include "frontend/camera.hpp"
#include "frontend/window.hpp"
#include "util/logger.hpp"
#include "util/quaternion.hpp"

#include <GLFW/glfw3.h>
#include <atomic>
#include <sstream>

Controller::Controller() : window{Window::GetInstance().getWindow()}
{
    // we can get to this class in the callback
    glfwSetWindowUserPointer(window, this);
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetKeyCallback(window, keyCallback);
}

void Controller::update(double deltaTime)
{
    if (!inputCaptured || (camera == nullptr)) {
        return;
    }

    const Quaternion& orientation = camera->getOrientation();

    linalg::aliases::float3 forwardOrientation = orientation.forward();

    // Extract yaw from quaternion for horizontal movement
    float yaw = atan2(forwardOrientation.x, -forwardOrientation.z);

    // Calculate movement direction using yaw rotation
    linalg::aliases::float3 forward{sin(yaw), 0.0F, -cos(yaw)};
    linalg::aliases::float3 right{cos(yaw), 0.0F, sin(yaw)};
    linalg::aliases::float3 up{0.0F, 1.0F, 0.0F};

    bool moveForward = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    bool moveBackward = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
    bool moveLeft = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    bool moveRight = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
    bool moveUp = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    bool moveDown = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;

    float velocity = moveSpeed * deltaTime;
    linalg::aliases::float3 movement{0.0F, 0.0F, 0.0F};

    // movement input (my god this is terrible, surely I can do
    // better...SURELY)
    if (moveForward) {
        movement += forward * velocity;
    }
    if (moveBackward) {
        movement -= forward * velocity;
    }
    if (moveLeft) {
        movement -= right * velocity;
    }
    if (moveRight) {
        movement += right * velocity;
    }
    if (moveUp) {
        movement += up * velocity;
    }
    if (moveDown) {
        movement -= up * velocity;
    }

    // normalise diagonal movement
    float horizontalMagnitude =
      ((movement.x * movement.x) + (movement.z * movement.z));
    if (horizontalMagnitude > velocity * velocity) {
        float scale = velocity / std::sqrtf(horizontalMagnitude);
        movement.x *= scale;
        movement.z *= scale;
    }

    if (movement.x != 0.0F || movement.y != 0.0F || movement.z != 0.0F) {
        camera->move(movement);
    }
}

void Controller::captureMouse()
{
    inputCaptured = true;
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    firstMouse = true;
}

void Controller::releaseMouse()
{
    inputCaptured = false;
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
}

void Controller::mouseCallback(GLFWwindow* window, double xpos, double ypos)
{
    auto* controller =
      static_cast<Controller*>(glfwGetWindowUserPointer(window));

    if (!controller->inputCaptured || (controller->camera == nullptr)) {
        return;
    }

    if (controller->firstMouse) {
        controller->lastMouseX = xpos;
        controller->lastMouseY = ypos;
        controller->firstMouse = false;
        return; // skip this frame to prevent choppy look
    }

    double xoffset = xpos - controller->lastMouseX;
    double yoffset = ypos - controller->lastMouseY;

    controller->lastMouseX = xpos;
    controller->lastMouseY = ypos;

    xoffset *= controller->mouseSensitivity;
    yoffset *= controller->mouseSensitivity;

    constexpr static float degToRadCoeff = 0.0174533F;
    const float yawDelta =
      static_cast<float>(xoffset) * degToRadCoeff; // degrees to radians
    float pitchDelta = static_cast<float>(yoffset) * degToRadCoeff;

    const float oldPitch = controller->pitch;
    controller->pitch += pitchDelta;
    controller->pitch = std::clamp(controller->pitch, -89.0F * degToRadCoeff,
                                   89.0F * degToRadCoeff);
    // ugly way to move cam only in correct zone
    pitchDelta = controller->pitch - oldPitch;

    // apply rotations
    Camera* camera = controller->camera;
    camera->rotateAroundAxis({0.0F, 1.0F, 0.0F}, -yawDelta);

    camera->rotateAroundAxis(controller->camera->getRight(), -pitchDelta);
}

void Controller::mouseButtonCallback(GLFWwindow* window, int button, int action,
                                     int mods)
{
    auto* controller =
      static_cast<Controller*>(glfwGetWindowUserPointer(window));

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        controller->captureMouse();
    }
}

// To detect only single pulses resulting from presses.
// To do behaviour with held down key, see Controller::update
void Controller::keyCallback(GLFWwindow* window, int key, int scancode,
                             int action, int mods)
{
    auto* controller =
      static_cast<Controller*>(glfwGetWindowUserPointer(window));

    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE: controller->releaseMouse(); break;
            case GLFW_KEY_P: controller->toggleIsPlaying(); break;
            case GLFW_KEY_I:
                if (controller->camera != nullptr) {
                    std::stringstream ss;
                    const auto& camPos = controller->camera->getPosition();
                    const auto& camOri =
                      controller->camera->getOrientation().data();
                    ss << "Camera pos " << camPos.x << ", " << camPos.y << ", "
                       << camPos.z << "\n"
                       << "Camera orientation quat. " << camOri.x << " + "
                       << camOri.y << "i + " << camOri.z << "j + " << camOri.w
                       << "k\n";
                    Logger::GetInstance().log(ss.str());
                }
                break;
            default: break;
        }
    }
}
void Controller::setMainCamera(Camera* cam)
{
    camera = cam;
    pitch = cam->getOrientation().toEulerAngles().z;
}

void Controller::setIsPlayingBoolPtr(std::atomic<bool>* isPlayingBool_ptr)
{
    isPlaying_ptr.store(isPlayingBool_ptr);
}

void Controller::toggleIsPlaying()
{
    std::atomic<bool>* ptr = isPlaying_ptr.load(std::memory_order_acquire);
    if (ptr != nullptr) {
        bool old = ptr->load(std::memory_order_relaxed);
        bool desired = !old;

        // WHYY CPP STANDARD DOES NOT DEFINE FETCH_XOR FOR BOOLEANS WHEN IT DOES
        // FOR ALL OTHER INTEGRAL TYPES
        while (!ptr->compare_exchange_strong(old, desired)) {
            desired = !old;
        }
    }
}
