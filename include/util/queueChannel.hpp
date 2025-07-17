#pragma once

#include "channel.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <vector>

// A multi-producer, single-consumer queue wrapped up in a channel.
// Threads can call into the channel to register items required.
// The consumer thread can either choose to sleep until new items exist, or poll
// the channel.
// Once there are items in the queue, they are batched up and given to the
// consumer at once when requested.
//
// See the Logger class for a simple class to use this channel.
//
// Notes:
// - Items should be as small as possible as they will need to be frequently
// allocated/de-allocated & copied.
// - This implementation is not lock-free yet sadly (I am not brave enough to
// use my own lock-free queue implementation).
template <typename T>
class MPSCQueueChannel
{
    std::queue<T> messageQueue;
    std::mutex queueMutex;
    std::condition_variable condVar;
    std::atomic<bool> exitFlag{false};

    // Buffers for the channel abstraction
    std::vector<T> writeBuffer;
    std::vector<T> previousWriteBuffer;
    std::vector<T> readBuffer;

    void drainQueue(const std::unique_lock<std::mutex>& m);
    // (into the read buffer)

  public:
    class ChannelSender : public Sender<std::vector<T>>
    {
        friend class MPSCQueueChannel<T>;
        MPSCQueueChannel<T>& channel;

        explicit ChannelSender(MPSCQueueChannel<T>& ch);

      public:
        void send() override;
        std::vector<T>& getWriteBuffer() override;
        std::vector<T>& getPreviousWriteBuffer() override;
    };

    class ChannelReceiver : public Receiver<std::vector<T>>
    {
        friend class MPSCQueueChannel<T>;
        MPSCQueueChannel<T>& channel;

        explicit ChannelReceiver(MPSCQueueChannel<T>& ch);

      public:
        const std::vector<T>& getBuffer() override;

        // non-blocking version for polling threads
        bool isMessageReady() override;

        // blocking version. waits indefinitely for messages until the cond.
        // var. wakes it with fresh messages
        bool isMessageReadyElseSleep();

        // timeout version. waits up to a specified duration
        // not used yet
        template <typename Rep, typename Period>
        bool
        isMessageReadyFor(const std::chrono::duration<Rep, Period>& timeout);
    };

  private:
    ChannelSender sender{*this};
    ChannelReceiver receiver{*this};

    void sendImpl();
    bool isMessageReadyCondVarImpl();
    bool isMessageReadyNonBlockingImpl();

    template <typename Rep, typename Period>
    bool isMessageReadyTimeoutImpl(
      const std::chrono::duration<Rep, Period>& timeout);

  public:
    MPSCQueueChannel() = default;
    MPSCQueueChannel(const MPSCQueueChannel&) = delete;
    MPSCQueueChannel(MPSCQueueChannel&&) = delete;
    MPSCQueueChannel& operator=(const MPSCQueueChannel&) = delete;
    MPSCQueueChannel& operator=(MPSCQueueChannel&&) = delete;
    ~MPSCQueueChannel();

    ChannelSender& getSender();

    ChannelReceiver& getReceiver();

    // convenience method for single item sends
    void sendSingle(const T& item);
    void sendSingle(T&& item);

    // signal shutdown to all threads sleeping on us
    void shutdown();

    bool isShutdownRequested() const;
};

template <typename T>
inline void MPSCQueueChannel<T>::drainQueue(
  const std::unique_lock<std::mutex>& /*m*/) // (into the read buffer)
{
    readBuffer.clear();
    while (!messageQueue.empty()) {
        readBuffer.push_back(std::move(messageQueue.front()));
        messageQueue.pop();
    }
}

template <typename T>
inline bool MPSCQueueChannel<T>::isShutdownRequested() const
{
    return exitFlag.load();
}

template <typename T>
inline void MPSCQueueChannel<T>::shutdown()
{
    exitFlag.store(true);
    condVar.notify_all();
}

template <typename T>
inline void MPSCQueueChannel<T>::sendSingle(const T& item)
{
    std::unique_lock<std::mutex> lock(queueMutex);
    messageQueue.push(item);
    condVar.notify_one();
}

template <typename T>
void MPSCQueueChannel<T>::sendSingle(T&& item)
{
    std::unique_lock<std::mutex> lock(queueMutex);
    messageQueue.push(std::move(item));
    condVar.notify_one();
}

template <typename T>
inline typename MPSCQueueChannel<T>::ChannelReceiver&
MPSCQueueChannel<T>::getReceiver()
{
    return receiver;
}

template <typename T>
inline typename MPSCQueueChannel<T>::ChannelSender&
MPSCQueueChannel<T>::getSender()
{
    return sender;
}

template <typename T>
inline MPSCQueueChannel<T>::~MPSCQueueChannel()
{
    shutdown();
}

template <typename T>
template <typename Rep, typename Period>
inline bool MPSCQueueChannel<T>::isMessageReadyTimeoutImpl(
  const std::chrono::duration<Rep, Period>& timeout)
{
    std::unique_lock<std::mutex> lock(queueMutex);

    if (condVar.wait_for(lock, timeout, [this] {
            return !messageQueue.empty() || exitFlag.load();
        }))
    {
        if (exitFlag.load()) {
            return false;
        }

        drainQueue(lock);
        return !readBuffer.empty();
    }

    return false;
}

template <typename T>
inline bool MPSCQueueChannel<T>::isMessageReadyNonBlockingImpl()
{
    std::unique_lock<std::mutex> lock(queueMutex);

    if (exitFlag.load()) {
        return false;
    }

    if (!messageQueue.empty()) {
        drainQueue(lock);
        return !readBuffer.empty();
    }

    return false;
}

template <typename T>
inline bool MPSCQueueChannel<T>::isMessageReadyCondVarImpl()
{
    std::unique_lock<std::mutex> lock(queueMutex);
    condVar.wait(lock,
                 [this] { return !messageQueue.empty() || exitFlag.load(); });

    if (exitFlag.load()) {
        return false;
    }

    drainQueue(lock);
    return !readBuffer.empty();
}

template <typename T>
inline void MPSCQueueChannel<T>::sendImpl()
{
    if (!writeBuffer.empty()) {
        std::unique_lock<std::mutex> lock(queueMutex);
        for (auto& item : writeBuffer) {
            messageQueue.push(std::move(item));
        }
        previousWriteBuffer = std::move(writeBuffer);
        writeBuffer.clear();
        condVar.notify_one();
    }
}

template <typename T>
template <typename Rep, typename Period>
inline bool MPSCQueueChannel<T>::ChannelReceiver::isMessageReadyFor(
  const std::chrono::duration<Rep, Period>& timeout)
{
    return channel.isMessageReadyTimeoutImpl(timeout);
}

template <typename T>
inline bool MPSCQueueChannel<T>::ChannelReceiver::isMessageReadyElseSleep()
{
    return channel.isMessageReadyCondVarImpl();
}

template <typename T>
inline bool MPSCQueueChannel<T>::ChannelReceiver::isMessageReady()
{
    return channel.isMessageReadyNonBlockingImpl();
}

template <typename T>
inline const std::vector<T>& MPSCQueueChannel<T>::ChannelReceiver::getBuffer()
{
    return channel.readBuffer;
}

template <typename T>
inline MPSCQueueChannel<T>::ChannelReceiver::ChannelReceiver(
  MPSCQueueChannel<T>& ch)
  : channel(ch)
{
}

template <typename T>
inline std::vector<T>&
MPSCQueueChannel<T>::ChannelSender::getPreviousWriteBuffer()
{
    return channel.previousWriteBuffer;
}

template <typename T>
inline std::vector<T>& MPSCQueueChannel<T>::ChannelSender::getWriteBuffer()
{
    return channel.writeBuffer;
}

template <typename T>
inline void MPSCQueueChannel<T>::ChannelSender::send()
{
    channel.sendImpl();
}

template <typename T>
inline MPSCQueueChannel<T>::ChannelSender::ChannelSender(
  MPSCQueueChannel<T>& ch)
  : channel(ch)
{
}
