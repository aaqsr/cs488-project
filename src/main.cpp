#include "frontend/renderer.hpp"
#include "util/error.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>

namespace
{

void run()
{
    Renderer& renderer = Renderer::GetInstance();

    try {
        renderer.init();
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
