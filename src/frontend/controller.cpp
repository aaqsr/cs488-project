#include "frontend/controller.hpp"
#include "frontend/window.hpp"

#include "GLFW/glfw3.h"

Controller::Controller() : window{Window::GetInstance().getWindow()}
{
    glfwSetWindowUserPointer(window,
                             this); // we can get to this class in the callback
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetKeyCallback(window, keyCallback);
}

void Controller::update(double deltaTime)
{
    if (camera == nullptr) {
        return;
    }

    if (!inputCaptured) {
        return;
    }

    float velocity = moveSpeed * deltaTime;
    linalg::aliases::float3 movement{0.0F, 0.0F, 0.0F};

    // movement input. (my god this is terrible, surely I can do
    // better...SURELY)
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        movement += camera->getFront() * velocity;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        movement -= camera->getFront() * velocity;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        movement -= camera->getRight() * velocity;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        movement += camera->getRight() * velocity;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        movement += camera->getUp() * velocity;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        movement -= camera->getUp() * velocity;

    if (movement.x != 0.0f || movement.y != 0.0f || movement.z != 0.0f) {
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
    Controller* controller =
      static_cast<Controller*>(glfwGetWindowUserPointer(window));

    if (!controller->inputCaptured || (controller->camera == nullptr)) {
        return;
    }

    if (controller->firstMouse) {
        controller->lastMouseX = xpos;
        controller->lastMouseY = ypos;
        controller->firstMouse = false;
    }

    double xoffset = xpos - controller->lastMouseX;
    double yoffset = ypos - controller->lastMouseY;

    controller->lastMouseX = xpos;
    controller->lastMouseY = ypos;

    xoffset *= controller->mouseSensitivity;
    yoffset *= controller->mouseSensitivity;

    // Convert to radians
    float yawDelta =
      static_cast<float>(xoffset) * 0.0174533F; // degrees to radians
    float pitchDelta = static_cast<float>(yoffset) * 0.0174533F;

    // Apply rotations
    controller->camera->rotateAroundAxis({0.0F, 1.0F, 0.0F},
                                         -yawDelta); // Yaw (Y-axis)
    controller->camera->rotateAroundAxis(
      controller->camera->getRight(),
      -pitchDelta); // Pitch (local right axis)
}

void Controller::mouseButtonCallback(GLFWwindow* window, int button, int action,
                                     int mods)
{
    Controller* controller =
      static_cast<Controller*>(glfwGetWindowUserPointer(window));

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        controller->captureMouse();
    }
}

void Controller::keyCallback(GLFWwindow* window, int key, int scancode,
                             int action, int mods)
{
    Controller* controller =
      static_cast<Controller*>(glfwGetWindowUserPointer(window));

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        controller->releaseMouse();
    }
}
