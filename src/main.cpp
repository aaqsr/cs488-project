#include "frontend/renderer.hpp"
#include "sim/waterSimulation.hpp"
#include "util/channel.hpp"
#include "util/error.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

namespace
{

void run()
{
    // Do not reorder these sigh
    Renderer& renderer = Renderer::GetInstance();

    // Attach channels
    TripleBufferedChannel<
      HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>>
      channel; // very heavy class yum!
    renderer.attachReceiverChannel(&(channel.getReceiver()));

    // aaaaaaand awayyyyy we go!
    std::atomic<bool> appShouldExit{false};

    std::jthread simulationThread{
      [](Sender<HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>>*
           channel,
         const std::atomic<bool>& appShouldExit) {
          WaterSimulation sim;
          sim.attachSenderChannel(channel);
          Controller::GetInstance().setSim(&sim);

          auto lastUpdateTime = std::chrono::high_resolution_clock::now();
          // TODO: The bound here is wonky. I think we want it such that
          // deltaT is less than the time between rendered frames??
          // I'm not sure. See section 2.3 of R. Bridson for more
          // We also more importantly want it so that the simulation frames that take longer 
          // For now this is an *arbitrary* amount.
          const auto targetFrameTime = std::chrono::microseconds(350);

          while (!appShouldExit.load()) {
              auto frameStart = std::chrono::high_resolution_clock::now();

              sim.update();

              auto frameEnd = std::chrono::high_resolution_clock::now();
              auto frameDuration = frameEnd - frameStart;

              if (frameDuration < targetFrameTime) {
                  std::this_thread::sleep_for(targetFrameTime - frameDuration);
              } else {
                  // Logger::GetInstance().log(
                  //   "Not meeting timing, frameDuration is " +
                  //   std::to_string(
                  //     std::chrono::duration_cast<std::chrono::microseconds>(
                  //       frameDuration)
                  //       .count()) +
                  //   " microseconds");
              }
          }
      },
      &(channel.getSender()), std::cref(appShouldExit)};

    try {
        renderer.init();
        renderer.loop();
        appShouldExit.store(true);
    } catch (IrrecoverableError& e) {
        // TODO: App class to make this safer?
        appShouldExit.store(true);
        std::cout << e.msg << "\n" << e.what() << "\n";
    }
}

} // namespace

int main()
{
    run();
}
