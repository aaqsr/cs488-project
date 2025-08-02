#pragma once

#include "util/channel.hpp"

#include <atomic>

/**
 * @class TripleBufferedChannel
 * @brief A single-slot, wait-free, single-producer single-consumer (SPSC)
 * communication channel.
 * @ingroup util
 * @tparam T The type of data being synchronized.
 *
 * @details This channel implements the triple buffering pattern to provide
 * high-frequency state synchronisation between exactly two threads (e.g., the
 * physics thread updating our render thread). Its primary design goal is to
 * ensure that neither the producer nor the consumer is ever blocked waiting for
 * the other. This is achieved at the cost of dropping intermediate states if
 * the producer is faster than the consumer; only the most recently completed
 * state is ever delivered.
 *
 * @section Technicality
 * The "triple buffering" algorithm uses three distinct instances (`T`) of the
 * data, managed by atomic pointers, to eliminate contention and the need for
 * locks.
 * 1.  **`inprogress`:** A buffer exclusively owned by the producer for
 * preparing the next state. The consumer never touches this.
 * 2.  **`present`:** A buffer exclusively owned by the consumer for reading the
 * current state. The producer never touches this.
 * 3.  **`ready`:** A buffer that acts as a transfer slot. The producer places a
 * completed buffer here, and the consumer picks it up.
 *
 * The synchronisation is orchestrated through atomic pointer exchanges:
 * - **`send()`:** The producer finishes writing to `inprogress`. It then
 * atomically swaps its `inprogress` pointer with the `ready` pointer. The old
 * `ready` buffer now becomes the new `inprogress` buffer for the next frame,
 * and the producer can begin writing to it immediately without blocking.
 * - **`getBuffer()` (on receive):** The consumer checks an atomic flag
 * (`isNewDataAvailable`). If true, it atomically swaps its `present` pointer
 * with the `ready` pointer. The old `present` buffer now becomes the new
 * `ready` buffer (freeing it up for the producer), and the consumer can
 * immediately begin reading from its new `present` buffer.
 *
 * @section Performance and Guarantees
 * - **Wait-Free:** Both `send()` and `receive()` operations are guaranteed to
 * complete in a finite number of steps, regardless of the execution speed or
 * state of the other thread. This makes the channel suitable for real-time
 * systems where thread stalls are unacceptable.
 * - **Linearisability:** The operations are linearisable. If my math is
 * correct... (which is may not be)
 * - **Data Loss by Design:** If the producer completes two `send` operations
 * before the consumer performs a `receive`, the data from the first `send` will
 * be overwritten at the `ready` slot and will never be seen by the consumer.
 * This is the intended behaviour for state synchronisation, where only the
 * latest state matters (e.g., rendering the most up-to-date object positions).
 * - **Memory Efficient:** The buffers do not need to be allocated and freed
 * other than at first instantiations and final destruction. This lets us pass
 * huge chunks of data between the producer and consumer thread effficiently.
 */
template <typename T>
class TripleBufferedChannel
{
    /** @brief Atomic pointer to the buffer the consumer is currently reading
     * from. */
    std::atomic<T*> present;
    /** @brief Atomic pointer to the buffer holding the latest completed data
     * from the producer. */
    std::atomic<T*> ready;
    /** @brief Atomic pointer to the buffer the producer is currently writing
     * to. */
    std::atomic<T*> inprogress;

    /** @brief Atomic pointer to the buffer from the previous producer write,
     * for stateful updates (e.g., Verlet integration). */
    std::atomic<T*> previousWriteBuffer;
    /** @brief An atomic flag indicating if new data is in the `ready` buffer
     * for the consumer to pick up. */
    std::atomic<bool> isNewDataAvailable;

    /**
     * @class ChannelSender
     * @brief The concrete wait-free sender for this channel.
     * @ingroup util
     */
    class ChannelSender : public Sender<T>
    {
        /** @brief A reference back to the parent channel. */
        TripleBufferedChannel<T>& channel;

        /** @brief Commits the `inprogress` buffer by swapping it with the
         * `ready` buffer. */
        void send() override
        {
            channel.send();
        }

        /** @brief Gets the producer's exclusive write buffer (`inprogress`). */
        T& getWriteBuffer() override
        {
            return channel.getWriteBuffer();
        }

        /** @brief Gets the buffer that was written in the previous send
         * operation. */
        T& getPreviousWriteBuffer() override
        {
            return channel.getPreviousWriteBuffer();
        }

      public:
        /**
         * @brief Constructs the sender.
         * @param channel A reference to the parent `TripleBufferedChannel`.
         */
        explicit ChannelSender(TripleBufferedChannel<T>& channel)
          : channel{channel}
        {
        }
    };

    /**
     * @class ChannelReceiver
     * @brief The concrete wait-free receiver for this channel.
     * @ingroup util
     */
    class ChannelReceiver : public Receiver<T>
    {
        /** @brief A reference back to the parent channel. */
        TripleBufferedChannel<T>& channel;

        /** @brief Gets the consumer's exclusive read buffer (`present`),
         * potentially swapping it with the `ready` buffer if new data is
         * available. */
        T& getBuffer() override
        {
            return channel.getBuffer();
        }

        /** @brief Checks if new data is available from the producer. This is a
         * non-blocking check. */
        bool isMessageReady() override
        {
            return channel.isNewDataAvailable.load();
        }

      public:
        /**
         * @brief Constructs the receiver.
         * @param channel A reference to the parent `TripleBufferedChannel`.
         */
        explicit ChannelReceiver(TripleBufferedChannel<T>& channel)
          : channel{channel}
        {
        }
    };

    /** @brief The sender instance associated with this channel. */
    ChannelSender sender;
    /** @brief The receiver instance associated with this channel. */
    ChannelReceiver receiver;

    /**
     * @brief The core producer-side logic. Makes the `inprogress` buffer
     * available to the consumer.
     */
    void send()
    {
        T* writeBuf = inprogress.load();
        previousWriteBuffer.store(writeBuf);

        T* old_ready = ready.exchange(writeBuf);
        inprogress.store(old_ready);

        isNewDataAvailable.store(true);
    }

    /**
     * @brief Provides the producer with its exclusive write buffer.
     * @return A reference to the `inprogress` buffer.
     */
    T& getWriteBuffer()
    {
        return *inprogress.load();
    }

    /**
     * @brief Provides the producer with the buffer from the previous frame.
     * @return A reference to the `previousWriteBuffer`.
     */
    T& getPreviousWriteBuffer()
    {
        T* ptr = previousWriteBuffer.load();
        if (ptr == nullptr) {
            ptr = inprogress.load();
        }
        return *ptr;
    }

    /**
     * @brief A placeholder method, currently unused.
     */
    void consumed()
    {
    }

    /**
     * @brief The core consumer-side logic. Acquires the latest data if
     * available.
     * @return A reference to the consumer's `present` buffer, which will
     * contain the latest data if a swap occurred.
     */
    T& getBuffer()
    {
        bool expected = true;
        if (isNewDataAvailable.compare_exchange_strong(expected, false)) {
            T* old_ready = ready.exchange(present.load());
            present.store(old_ready);
        }
        // return old buffer if no new data, new one else
        return *present.load();
    }

  public:
    /**
     * @brief Constructs the channel, allocating the three buffers on the heap
     * and initializing the sender/receiver pair.
     */
    TripleBufferedChannel()
      : present(new T{}), ready(new T{}), inprogress(new T{}),
        previousWriteBuffer(nullptr), isNewDataAvailable(false), sender(*this),
        receiver(*this)
    {
    }

    /** @brief Deleted copy constructor to prevent copying of the channel state.
     */
    TripleBufferedChannel(const TripleBufferedChannel&) = delete;
    /** @brief Deleted move constructor. */
    TripleBufferedChannel(TripleBufferedChannel&&) = delete;
    /** @brief Deleted copy assignment operator. */
    TripleBufferedChannel& operator=(const TripleBufferedChannel&) = delete;
    /** @brief Deleted move assignment operator. */
    TripleBufferedChannel& operator=(TripleBufferedChannel&&) = delete;

    /**
     * @brief Destructor that safely deallocates the three buffers.
     */
    ~TripleBufferedChannel()
    {
        delete present.load();
        delete ready.load();
        delete inprogress.load();
    }

    /** @brief Gets a reference to the sender interface. */
    Sender<T>& getSender()
    {
        return sender;
    }

    /** @brief Gets a reference to the receiver interface. */
    Receiver<T>& getReceiver()
    {
        return receiver;
    }
};
