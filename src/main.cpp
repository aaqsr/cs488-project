#include "bridgeChannelData.hpp"
#include "frontend/renderer.hpp"
#include "physics/constants.hpp"
#include "physics/physicsEngine.hpp"
#include "sim/waterSimulation.hpp"
#include "util/channel.hpp"
#include "util/error.hpp"
#include "util/logger.hpp"
#include "util/perf.hpp"
#include "util/queueChannel.hpp"
#include "util/tripleBufferedChannel.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace
{

void physicsAndSimulationThread(
  Sender<BridgeChannelData>& bridgeChannel,
  Receiver<std::vector<PhysicsEngineReceiverData>>& physCmdChannel,
  const std::atomic<bool>& appShouldExit, bool waterEnabled,
  float waterInitHumpSz)
{
    WaterSimulation sim;
    PhysicsEngine phys{physCmdChannel};

    std::atomic<bool> isPlaying{false};
    Controller::GetInstance().setIsPlayingBoolPtr(&isPlaying);

    if (waterEnabled) {
        // Initial conditions
        auto msg = bridgeChannel.createMessage();
        WaterSimulation::setInitConditions(msg.getWriteBuffer().waterHeights,
                                           waterInitHumpSz);
    }

    // we want it such that deltaT is less than the time between rendered frames
    constexpr auto targetFrameTime =
      std::chrono::duration<double>{Physics::WaterSim::deltaT * 0.8F};

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

                if (waterEnabled) {
                    sim.update(msg.getWriteBuffer().waterHeights,
                               msg.getPreviousWriteBuffer().waterHeights);
                }

                phys.updateRigidBodies(
                  msg.getWriteBuffer().physicsObjects,
                  msg.getPreviousWriteBuffer().physicsObjects, waterEnabled);

                if (waterEnabled) {
                    sim.coupleWithRigidBodies(
                      msg.getWriteBuffer().physicsObjects,
                      msg.getWriteBuffer().waterHeights);
                }
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

void printUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --scene <path>    Path to scene JSON file\n";
    std::cout << "  --help           Show this help message\n";
    std::cout
      << "\nIf no scene file is provided, a default scene will be used.\n";
}

void run(int argc, const char* argv[])
{
    // Parse command line arguments
    std::filesystem::path sceneFilePath;

    bool loggingEnabled = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return;
        } else if (arg == "--scene") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --scene option requires a file path\n";
                printUsage(argv[0]);
                return;
            }
            sceneFilePath = argv[++i];

            if (!std::filesystem::exists(sceneFilePath)) {
                std::cerr << "Error: Scene file does not exist: "
                          << sceneFilePath << "\n";
                return;
            }
        } else if (arg == "--log") {
            loggingEnabled = true;
        } else {
            std::cerr << "Error: Unknown option '" << arg << "'\n";
            printUsage(argv[0]);
            return;
        }
    }

    if (loggingEnabled) {
        Logger::GetInstance().enable();
    } else {
        Logger::GetInstance().disable();
    }

    // Do not reorder these sigh
    Renderer& renderer = Renderer::GetInstance();

    if (!sceneFilePath.empty()) {
        std::cout << "Loading scene from: " << sceneFilePath << "\n";
        renderer.setSceneFile(sceneFilePath);
    } else {
        std::cout << "No scene file provided, using default scene\n";
    }

    // Attach channels
    TripleBufferedChannel<BridgeChannelData>
      bridgeChannel; // very heavy class yum!
    renderer.attachBridgeChannel(&(bridgeChannel.getReceiver()));

    MPSCQueueChannel<PhysicsEngineReceiverData> physCmdChannel;
    renderer.attachPhysicsEngineCommandsChannel(&(physCmdChannel.getSender()));

    renderer.loadSceneData();

    // aaaaaaand awayyyyy we go!
    std::atomic<bool> appShouldExit{false};

    std::jthread simulationThread{physicsAndSimulationThread,
                                  std::ref(bridgeChannel.getSender()),
                                  std::ref(physCmdChannel.getReceiver()),
                                  std::cref(appShouldExit),
                                  renderer.getSceneData().waterEnabled,
                                  renderer.getSceneData().waterInitialHumpSize};

    Logger::GetInstance().log(
      "AAAA" + std::to_string(renderer.getSceneData().waterInitialHumpSize));

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

int main(int argc, const char* argv[])
{
    run(argc, argv);
}
