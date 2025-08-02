#include "util/logger.hpp"
#include "util/queueChannel.hpp"

#include <iostream>
#include <string>

Logger::Logger()
{
    loggingThread = std::thread([this]() { this->processMessages(); });
}

void Logger::processMessages()
{
    MPSCQueueChannel<std::string>::ChannelReceiver& receiver =
      logChannel.getReceiver();

    while (!logChannel.isShutdownRequested()) {

        if (!receiver.isMessageReadyElseSleep()) {
            continue;
        }

        const auto& messages = receiver.getBuffer();

        if (messages.empty()) {
            continue;
        }

        for (const auto& message : messages) {
            std::cout << message << "\n";
        }
    }
}

Logger::~Logger()
{
    logChannel.shutdown();
    if (loggingThread.joinable()) {
        loggingThread.join();
    }
}

void Logger::log(std::string str, bool force)
{
    if (!force && !loggingEnabled.load()) {
        return;
    }
    logChannel.sendSingle(std::move(str));
}

void Logger::enable()
{
    loggingEnabled.store(true);
}

void Logger::disable()
{
    loggingEnabled.store(false);
}
