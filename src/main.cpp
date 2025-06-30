#include "error.hpp"
#include "renderer.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>

namespace
{

void run()
{
    Renderer& renderer = Renderer::GetInstance();

    try {
        renderer.loop();
    } catch (IrrecoverableError& e) {
        std::cout << e.msg << "\n" << e.what() << "\n";
    }
}

} // namespace

int main()
{
    run();
}
