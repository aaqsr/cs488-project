#include "util/logger.hpp"

#include <iostream>

void Logger::processMessages()
{
    std::unique_lock lock(queueMutex);

    // so wake me up when it's all over
    // when i'm wiser & i'm older etc.
    condVar.wait(lock, [this] { return exitFlag || !messageQueue.empty(); });

    while (true) {
        while (!messageQueue.empty()) {
            auto msg = std::move(messageQueue.front());
            messageQueue.pop();
            // unlock while printing to reduce contention for expensive
            // operation
            lock.unlock();
            std::cout << msg << '\n';
            lock.lock();
        }

        if (exitFlag) {
            // bye bye!
            break;
        }

        condVar.wait(lock,
                     [this] { return exitFlag || !messageQueue.empty(); });
    }
}

Logger::~Logger()
{
    {
        std::lock_guard<std::mutex> lock{queueMutex};
        exitFlag = true;
    }
    condVar.notify_one();
    loggingThread.join(); // wait for logging to finish
}

void Logger::log(std::string str)
{
    {
        std::lock_guard<std::mutex> lock{queueMutex};
        messageQueue.push(std::move(str));
    }
    condVar.notify_one();
}
