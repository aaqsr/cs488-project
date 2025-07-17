#pragma once

#include "util/logger.hpp"

#include <cstdint>
#include <format>

// Counts any metric you want over a second every second and logs the result
// Just remember to call tick()...
class IterationsPerSecondCounter
{
    uint32_t numFrames = 0;
    std::chrono::steady_clock::time_point timeOfLastPrint;
    std::string iterationUnit;
    std::string millisecondsPerIterationUnit;

  public:
    IterationsPerSecondCounter(std::string iterationUnit,
                               std::string secondsPerIterationUnit)
      : timeOfLastPrint(std::chrono::steady_clock::now()),
        iterationUnit(std::move(iterationUnit)),
        millisecondsPerIterationUnit(std::move(secondsPerIterationUnit))
    {
    }

    void tick()
    {
        numFrames++;
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSeconds = now - timeOfLastPrint;
        if (elapsedSeconds.count() >= 1.0) {
            double msPerIteration =
              (elapsedSeconds.count() * 1000.0) / numFrames;
            Logger::GetInstance().log(
              std::format("{:<6}{}\t{:<9.5f}ms/{}", numFrames, iterationUnit,
                          msPerIteration, millisecondsPerIterationUnit));

            numFrames = 0;
            timeOfLastPrint = now;
        }
    }
};
