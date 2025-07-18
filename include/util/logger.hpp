#pragma once

#include "linalg.h"
#include "singleton.hpp"

#include <array>
#include <ostream>
#include <queue>
#include <string>
#include <sstream>
#include <thread>

class Logger : public Singleton<Logger>
{
    friend class Singleton<Logger>;

    // TODO: I am not brave enough to test my lock-free queue implementation in
    // practice... but it's tempting...
    std::queue<std::string> messageQueue;
    std::mutex queueMutex;
    std::condition_variable condVar;
    std::atomic<bool> exitFlag = false;
    std::thread loggingThread{[this]() {
        this->processMessages();
    }};

    Logger() = default;

    void processMessages();

  public:
    Logger(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger& operator=(Logger&&) = delete;

    ~Logger() override;

    void log(std::string str);

    template <typename T, size_t sz>
    void log(const std::string& msg, const std::array<T, sz>& grid, int numRows = 1,
             int numCols = sz);
};

inline std::ostream& operator<<(std::ostream& s, const linalg::aliases::float2& vec)
{
    s << "[" << vec.x << ", " << vec.y << "]";
    return s;
}

template <typename T, size_t sz>
void Logger::log(const std::string& msg, const std::array<T, sz>& grid,
                 int numRows, int numCols)
{
    std::stringstream ss;
    ss << "\n" << msg << "\n";
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numCols; ++j) {
            ss << grid[(i * numCols) + j] << " ";
        }
        ss << "\n";
    }
    log(ss.str());
}
