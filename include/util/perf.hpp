#pragma once

#include "util/logger.hpp"

#include <chrono>
#include <cstdint>
#include <format>
#include <string>
#include <utility>

/**
 * @class IterationsPerSecondCounter
 * @brief A utility to measure and log a rate of events over one-second
 * intervals.
 * @ingroup util
 *
 * @details This class provides a simple way to track performance metrics such
 * as frames per second (FPS) or physics updates per second. It counts the
 * number of times `tick()` is called and, once per second, logs the total count
 * and the average time per tick to the asynchronous `Logger`.
 *
 * @section Usage
 * 1.  Instantiate the counter with descriptive units.
 * 2.  Call the `tick()` method once per iteration of the loop you want to
 * measure.
 *
 * @example
 * ```cpp
 * // In an initialisation block:
 * IterationsPerSecondCounter fpsCounter("FPS", "frame");
 *
 * // In the main game loop:
 * while(running) {
 * // ... game logic ...
 * fpsCounter.tick();
 * }
 * ```
 */
class IterationsPerSecondCounter
{
    /**
     * @brief A counter for the number of ticks since the last log.
     */
    uint32_t numFrames = 0;

    /**
     * @brief The timestamp of the last time a log was printed.
     */
    std::chrono::steady_clock::time_point timeOfLastPrint;

    /**
     * @brief A string describing the unit of the rate (e.g., "FPS", "Updates").
     */
    std::string iterationUnit;

    /**
     * @brief A string describing the unit of a single iteration (e.g., "frame",
     * "update").
     */
    std::string millisecondsPerIterationUnit;

  public:
    /**
     * @brief Constructs a new performance counter.
     * @param iterationUnit The string to display for the rate (e.g., "FPS").
     * @param secondsPerIterationUnit The string to display for a single
     * iteration's time (e.g., "frame").
     */
    IterationsPerSecondCounter(std::string iterationUnit,
                               std::string secondsPerIterationUnit)
      : timeOfLastPrint(std::chrono::steady_clock::now()),
        iterationUnit(std::move(iterationUnit)),
        millisecondsPerIterationUnit(std::move(secondsPerIterationUnit))
    {
    }

    /**
     * @brief Increments the iteration counter and logs the performance if a
     * second has elapsed.
     * @details This method should be called once per event or frame being
     * measured. It checks the elapsed time since the last log. If it is one
     * second or more, it calculates the rate and average time, logs them, and
     * resets the counter and timer.
     */
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
