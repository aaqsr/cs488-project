#include "bridgeChannelData.hpp"
#include "frontend/renderer.hpp"
#include "physics/physicsEngine.hpp"
#include "sim/waterSimulation.hpp"
#include "util/channel.hpp"
#include "util/error.hpp"
#include "util/perf.hpp"
#include "util/queueChannel.hpp"
#include "util/tripleBufferedChannel.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

namespace
{

void physicsAndSimulationThread(
  Sender<BridgeChannelData>& bridgeChannel,
  Receiver<std::vector<PhysicsEngineReceiverData>>& physCmdChannel,
  const std::atomic<bool>& appShouldExit)
{
    WaterSimulation sim;
    PhysicsEngine phys{physCmdChannel};

    std::atomic<bool> isPlaying{false};
    Controller::GetInstance().setIsPlayingBoolPtr(&isPlaying);

    {
        // Initial conditions
        auto msg = bridgeChannel.createMessage();
        WaterSimulation::setInitConditions(msg.getWriteBuffer().waterHeights);
    }

    // TODO: The bound here is wonky. I think we want it such that
    // deltaT is less than the time between rendered frames??
    // I'm not sure. See section 2.3 of R. Bridson for more
    // We also more importantly want it so that the simulation frames that take
    // longer For now this is an *arbitrary* amount.
    constexpr auto targetFrameTime = std::chrono::microseconds(50);

    IterationsPerSecondCounter msPerUpdate{"UPS", "update"};

    while (!appShouldExit.load()) {
        auto frameStart = std::chrono::high_resolution_clock::now();

        // TODO: Should be condition variable rather than polling? ehhhhh
        if (isPlaying) {
            msPerUpdate.tick();

            {
                auto msg = bridgeChannel.createMessage();

                // Must be in this order:
                // Heightfield simulation
                // Solid simulation
                // Two-way coupling of heightfield and solids
                // (optional) particle generation & simulation
                // Rendering (done implicitly once data sent through the
                // channel)

                // TODO: split sim.update in half. A function that needs the
                // channel to be open and one that does not. So we can send
                // message as fast as possible.
                sim.update(msg.getWriteBuffer().waterHeights,
                           msg.getPreviousWriteBuffer().waterHeights);

                phys.updateRigidBodies(msg.getWriteBuffer().physicsObjects, msg.getPreviousWriteBuffer().physicsObjects);
            }
        }

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
};

void run()
{
    // Do not reorder these sigh
    Renderer& renderer = Renderer::GetInstance();

    // Attach channels
    TripleBufferedChannel<BridgeChannelData>
      bridgeChannel; // very heavy class yum!
    renderer.attachBridgeChannel(&(bridgeChannel.getReceiver()));

    MPSCQueueChannel<PhysicsEngineReceiverData> physCmdChannel;
    renderer.attachPhysicsEngineCommandsChannel(&(physCmdChannel.getSender()));

    // aaaaaaand awayyyyy we go!
    std::atomic<bool> appShouldExit{false};

    std::jthread simulationThread{
      physicsAndSimulationThread, std::ref(bridgeChannel.getSender()),
      std::ref(physCmdChannel.getReceiver()), std::cref(appShouldExit)};

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
