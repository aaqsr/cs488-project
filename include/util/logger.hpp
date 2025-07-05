#pragma once

#include "singleton.hpp"

#include <queue>
#include <string>
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
};
