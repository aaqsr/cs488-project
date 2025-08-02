#pragma once

#include "channel.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <vector>

/**
 * @class MPSCQueueChannel
 * @brief A thread-safe, blocking, Multi-Producer, Single-Consumer (MPSC)
 * communication channel.
 * @ingroup util
 * @tparam T The type of data to be sent through the queue.
 *
 * @details This class implements a classic MPSC channel using a standard
 * library queue protected by a mutex. It is designed for scenarios where
 * multiple producer threads need to reliably send data to a single consumer
 * thread. A primary use case is the `Logger` system, where various application
 * threads submit log messages to a dedicated I/O thread. Unlike the
 * `TripleBufferedChannel`, this channel guarantees that every message sent is
 * eventually delivered in order.
 *
 * @section Technicality
 * The channel's design is centered around a `std::queue` which acts as the
 * shared buffer.
 * - **Synchronization:** To prevent race conditions, all access to the
 * `messageQueue` is guarded by a `std::mutex`. This ensures that only one
 * thread can modify the queue at any given time.
 * - **Producer Logic:** Producers can add single items via `sendSingle` or
 * batch items using the `Sender::Message` RAII wrapper. In both cases, the
 * calling thread acquires a lock on `queueMutex`, pushes the data, releases the
 * lock, and then notifies the consumer.
 * - **Consumer Logic:** The single consumer thread uses a
 * `std::condition_variable` (`condVar`) to wait efficiently. The
 * `isMessageReadyElseSleep` method allows the consumer to relinquish its CPU
 * time slice and sleep until a producer signals that new data is available.
 * This is vastly more efficient than busy-waiting in a tight loop (polling).
 * - **Batching Advantage:** When the consumer wakes, it drains the *entire*
 * queue into its local `readBuffer` while holding the lock. This batching
 * strategy is a key performance optimization. It minimizes the frequency and
 * duration of lock contention, as the lock is acquired once per batch of
 * messages rather than once per message.
 *
 * @section Performance and Guarantees
 * - **Blocking and Thread-Safe:** This is a fully thread-safe blocking queue.
 * The use of a mutex provides strong correctness guarantees at the cost of
 * performance under high contention.
 * - **FIFO Delivery:** The channel guarantees First-In, First-Out delivery of
 * all messages. No data is ever dropped.
 * - **Contention:** As a lock-based structure, performance can degrade if many
 * producers attempt to send data simultaneously, as they will be serialised by
 * the mutex. However, for typical logging or event-handling workloads, this
 * design is robust and sufficiently performant.
 */
template <typename T>
class MPSCQueueChannel
{
    /** @brief The internal queue holding messages from producers, shared
     * between threads. */
    std::queue<T> messageQueue;
    /** @brief A mutex to protect all access to `messageQueue` from concurrent
     * modification. */
    std::mutex queueMutex;
    /** @brief A condition variable to allow the consumer thread to sleep
     * efficiently until data arrives or shutdown is signalled. */
    std::condition_variable condVar;
    /** @brief An atomic flag to signal the consumer thread to shut down
     * gracefully. */
    std::atomic<bool> exitFlag{false};

    /** @brief A buffer where producers accumulate data before sending a batch
     * via the `Sender::Message` interface. */
    std::vector<T> writeBuffer;
    /** @brief A buffer holding a copy of the previously sent batch of data. */
    std::vector<T> previousWriteBuffer;
    /** @brief A buffer where the consumer receives a batch of data from the
     * queue after it has been drained. */
    std::vector<T> readBuffer;

    /**
     * @brief Drains all items from the internal `messageQueue` into the
     * `readBuffer`.
     * @details This is a private utility called by the receiver logic. It must
     * be called while the `queueMutex` is held.
     * @param m A reference to the unique_lock holding the mutex, ensuring
     * precondition is met.
     */
    void drainQueue(const std::unique_lock<std::mutex>& m);

  public:
    /**
     * @class ChannelSender
     * @brief The concrete sending end for the `MPSCQueueChannel`.
     * @ingroup util
     */
    class ChannelSender : public Sender<std::vector<T>>
    {
        friend class MPSCQueueChannel<T>;
        /** @brief A reference back to the parent channel. */
        MPSCQueueChannel<T>& channel;

        /**
         * @brief Private constructor to ensure it is only created by the parent
         * channel.
         * @param ch A reference to the parent `MPSCQueueChannel`.
         */
        explicit ChannelSender(MPSCQueueChannel<T>& ch);

      public:
        /** @brief Commits the batched `writeBuffer` to the queue. */
        void send() override;
        /** @brief Gets the buffer to which batched data should be written. */
        std::vector<T>& getWriteBuffer() override;
        /** @brief Gets the buffer that was written in the previous batched send
         * operation. */
        std::vector<T>& getPreviousWriteBuffer() override;
    };

    /**
     * @class ChannelReceiver
     * @brief The concrete receiving end for the `MPSCQueueChannel`.
     * @ingroup util
     */
    class ChannelReceiver : public Receiver<std::vector<T>>
    {
        friend class MPSCQueueChannel<T>;
        /** @brief A reference back to the parent channel. */
        MPSCQueueChannel<T>& channel;

        /**
         * @brief Private constructor to ensure it is only created by the parent
         * channel.
         * @param ch A reference to the parent `MPSCQueueChannel`.
         */
        explicit ChannelReceiver(MPSCQueueChannel<T>& ch);

      public:
        /** @brief Gets the buffer containing the received batch of data. */
        std::vector<T>& getBuffer() override;

        /**
         * @brief Checks if a message is ready without blocking (polls the
         * queue).
         * @return `true` if data is available, `false` otherwise.
         */
        bool isMessageReady() override;

        /**
         * @brief Blocks and sleeps the thread until a message is ready.
         * @details This is the most efficient way for a consumer to wait for
         * data, as it consumes no CPU cycles while idle. The underlying
         * condition variable is woken by a producer.
         * @return `true` if data is ready, `false` if shutdown was requested.
         */
        bool isMessageReadyElseSleep();

        /**
         * @brief Checks if a message is ready, waiting for a specified timeout.
         * @return `true` if data becomes available, `false` if the timeout is
         * reached.
         */
        template <typename Rep, typename Period>
        bool
        isMessageReadyFor(const std::chrono::duration<Rep, Period>& timeout);
    };

  private:
    /** @brief The sender instance associated with this channel. */
    ChannelSender sender{*this};
    /** @brief The receiver instance associated with this channel. */
    ChannelReceiver receiver{*this};

    /**
     * @brief The internal implementation for the batched send operation.
     */
    void sendImpl();
    /**
     * @brief Internal implementation for an indefinite, blocking wait on the
     * condition variable.
     * @return `true` if a message is ready, `false` if shutdown was signalled.
     */
    bool isMessageReadyCondVarImpl();
    /**
     * @brief Internal implementation for a non-blocking check for messages.
     * @return `true` if a message is ready, `false` otherwise.
     */
    bool isMessageReadyNonBlockingImpl();

    /**
     * @brief Internal implementation for a timed wait on the condition
     * variable.
     * @tparam Rep The representation type of the duration (e.g., int, long).
     * @tparam Period The period type of the duration (e.g., std::milli,
     * std::ratio<1>).
     * @param timeout The maximum duration to wait.
     * @return `true` if a message is ready, `false` if the timeout expired.
     */
    template <typename Rep, typename Period>
    bool isMessageReadyTimeoutImpl(
      const std::chrono::duration<Rep, Period>& timeout);

  public:
    MPSCQueueChannel() = default;

    // Deleted copy/move since there should only ever be one channel
    MPSCQueueChannel(const MPSCQueueChannel&) = delete;
    MPSCQueueChannel(MPSCQueueChannel&&) = delete;
    MPSCQueueChannel& operator=(const MPSCQueueChannel&) = delete;
    MPSCQueueChannel& operator=(MPSCQueueChannel&&) = delete;

    ~MPSCQueueChannel();

    /** @brief Gets a reference to the sender interface. */
    ChannelSender& getSender();

    /** @brief Gets a reference to the receiver interface. */
    ChannelReceiver& getReceiver();

    /**
     * @brief Sends a single item by copying it into the queue.
     * @param item The item to send.
     */
    void sendSingle(const T& item);
    /**
     * @brief Sends a single item by moving it into the queue.
     * @param item The item to send.
     */
    void sendSingle(T&& item);

    // signal shutdown to all threads sleeping on us
    /** @brief Signals the channel to shut down, waking up the receiver if it is
     * sleeping. */
    void shutdown();

    /** @brief Checks if a shutdown has been requested. */
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
inline std::vector<T>& MPSCQueueChannel<T>::ChannelReceiver::getBuffer()
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
