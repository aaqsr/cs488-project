#pragma once

#include "singleton.hpp"
#include "util/queueChannel.hpp"

#include <linalg.h>

#include <array>
#include <ostream>
#include <sstream>
#include <string>
#include <thread>

/**
 * @class Logger
 * @brief A thread-safe, asynchronous, singleton logger.
 * @ingroup util
 *
 * @details This class provides a global point of access for logging messages
 * throughout the application. It is designed to be highly performant by
 * offloading the slow I/O operations of writing log messages to a dedicated
 * background thread.
 *
 * @section Architecture
 * The logger is implemented using several key patterns:
 * - **Singleton:** It inherits from a `Singleton<Logger>` template, ensuring
 * that only one instance of the logger exists globally. This provides a
 * convenient, centralized point for all logging activities.
 * - **Asynchronous Processing:** Calls to the `log()` methods do not write
 * directly to the console. Instead, they push the log message into a
 * thread-safe queue and return immediately. A dedicated background thread
 * (`loggingThread`) is responsible for consuming messages from this queue and
 * performing the actual I/O.
 *
 * @section Technicality
 * The core of the asynchronous mechanism is the `MPSCQueueChannel`, a
 * Multi-Producer, Single-Consumer queue. This data structure is specifically
 * chosen for this use case:
 * - **Multi-Producer:** Many different threads (e.g., physics, rendering,
 * input) can safely and concurrently push log messages into the queue.
 * - **Single-Consumer:** A single, dedicated background thread is the sole
 * consumer, which simplifies the logic for reading from the queue and writing
 * to standard output.
 *
 * @section Performance
 * This design significantly improves performance by decoupling the main
 * application logic from slow I/O operations. The main simulation loop is not
 * blocked waiting for logs to be written to disk or the console, which prevents
 * stuttering and maintains a smooth frame rate.
 *
 * @section Caveats
 * The implementation relies on a specific construction order: the `logChannel`
 * member must be initialized before the `loggingThread` is created. This is
 * necessary to ensure the thread has a valid channel to work with upon
 * starting. The destructor handles a graceful shutdown by signalling the
 * channel and joining the thread, ensuring all pending messages are flushed
 * before the program exits.
 */
class Logger : public Singleton<Logger>
{
    friend class Singleton<Logger>;

    /**
     * @brief The communication channel used to buffer log messages. It allows
     * multiple producer threads to send messages to a single consumer thread.
     */
    MPSCQueueChannel<std::string> logChannel;

    /**
     * @brief The dedicated background thread that processes and writes log
     * messages.
     */
    std::thread loggingThread;

    /** @brief Whether logging in enabled (disabled by default). */
    std::atomic<bool> loggingEnabled{false};

    /**
     * @brief Private constructor to enforce the singleton pattern. It spawns
     * the logging thread.
     */
    Logger();

    /**
     * @brief The main function executed by the `loggingThread`. It runs in a
     * loop, consuming messages from the `logChannel` and writing them to
     * `std::cout`.
     */
    void processMessages();

  public:
    // Delete copy/move constructors and assignment operators to prevent
    // duplication.
    Logger(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger& operator=(Logger&&) = delete;

    /**
     * @brief Destructor that ensures a graceful shutdown of the logging thread.
     * @details It signals the channel to shut down, then joins the thread,
     * waiting for it to finish processing any remaining messages in the queue.
     */
    ~Logger() override;

    /**
     * @brief Enqueues a string message to be logged asynchronously.
     * @param str The message to log.
     */
    void log(std::string str, bool force = false);

    /**
     * @brief A template utility function to log the contents of a 2D grid or
     * array.
     * @details This function is useful for debugging grid-based data
     * structures, such as those found in the water simulation, by
     * pretty-printing them to the console.
     * @tparam T The type of data in the grid.
     * @tparam sz The total size of the grid array.
     * @param msg A descriptive message to print before the grid data.
     * @param grid The `std::array` containing the grid data.
     * @param numRows The number of rows to use for formatting the output.
     * @param numCols The number of columns to use for formatting the output.
     */
    template <typename T, size_t sz>
    void log(const std::string& msg, const std::array<T, sz>& grid,
             int numRows = 1, int numCols = sz);

    /** @brief Enable log messages being registered. */
    void enable();
    /** @brief Disable log messages being registered. */
    void disable();
};

/**
 * @brief Overloads the stream insertion operator for `linalg::aliases::float2`.
 * @details Allows for easy and readable printing of 2D vectors to any
 * `std::ostream`.
 * @param s The output stream.
 * @param vec The vector to print.
 * @return A reference to the output stream.
 */
inline std::ostream& operator<<(std::ostream& s,
                                const linalg::aliases::float2& vec)
{
    s << "[" << vec.x << ", " << vec.y << "]";
    return s;
}

// Template implementation must be in the header file.
template <typename T, size_t sz>
void Logger::log(const std::string& msg, const std::array<T, sz>& grid,
                 int numRows, int numCols)
{
    std::stringstream ss;
    ss << "\n" << msg << "\n";
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numCols; ++j) {
            ss << grid[i * numCols + j] << " ";
        }
        ss << "\n";
    }
    log(ss.str());
}
