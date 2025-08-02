#pragma once

#include <atomic>

/**
 * @class Sender
 * @brief An abstract base class defining the sending end of a communication
 * channel.
 * @ingroup util
 * @tparam T The type of data being sent through the channel.
 *
 * @details This class defines the interface for sending messages. Concrete
 * channel implementations (like `TripleBufferedChannel` or `MPSCQueueChannel`)
 * will provide specific `Sender` types. The `Message` inner class uses RAII to
 * ensure that a send operation is automatically committed when the `Message`
 * object goes out of scope.
 */
template <typename T>
class Sender
{
    friend class Message;
    /** @brief Pure virtual function to commit the send operation. */
    virtual void send() = 0;
    /** @brief Pure virtual function to get a reference to the buffer being
     * written to. */
    virtual T& getWriteBuffer() = 0;
    /** @brief Pure virtual function to get a reference to the previously
     * written buffer. */
    virtual T& getPreviousWriteBuffer() = 0;

  public:
    /**
     * @class Message
     * @brief A RAII wrapper for a send operation.
     * @ingroup util
     * @details When a `Message` object is created, it represents the start of a
     * send operation. When it is destructed (at the end of its scope), its
     * destructor automatically calls the `sender.send()` method, committing the
     * data to the channel. This makes the channel API safer and less
     * error-prone.
     */
    class Message
    {
        Sender<T>& sender;

      public:
        explicit Message(Sender<T>& sender) : sender{sender}
        {
        }
        Message(const Message&) = delete;
        Message(Message&&) = delete;
        Message& operator=(const Message&) = delete;
        Message& operator=(Message&&) = delete;

        /**
         * @brief Destructor that automatically commits the send operation.
         */
        ~Message()
        {
            sender.send();
        }

        /** @brief Gets the buffer to which data should be written. */
        T& getWriteBuffer()
        {
            return sender.getWriteBuffer();
        }

        /** @brief Gets the buffer that was written in the previous send
         * operation. */
        T& getPreviousWriteBuffer()
        {
            return sender.getPreviousWriteBuffer();
        }
    };

    /**
     * @brief Creates a `Message` object to begin a send operation.
     * @return A `Message` RAII wrapper.
     */
    [[nodiscard]] Message createMessage()
    {
        return Message{*this};
    }

    virtual ~Sender() = default;
};

/**
 * @class Receiver
 * @brief An abstract base class defining the receiving end of a communication
 * channel.
 * @ingroup util
 * @tparam T The type of data being received.
 */
template <typename T>
class Receiver
{
    friend class Message;

    /** @brief Pure virtual function to get a reference to the received data
     * buffer. */
    virtual T& getBuffer() = 0;

  public:
    /**
     * @class Message
     * @brief A RAII wrapper for a receive operation.
     * @ingroup util
     * @details This class provides a scope-based handle to received data.
     */
    class Message
    {
        Receiver<T>& receiver;

      public:
        explicit Message(Receiver<T>& receiver) : receiver{receiver}
        {
        }
        Message(const Message&) = delete;
        Message(Message&&) = delete;
        Message& operator=(const Message&) = delete;
        Message& operator=(Message&&) = delete;

        ~Message() = default;

        /** @brief Gets a const reference to the received data buffer. */
        const T& getBuffer()
        {
            return receiver.getBuffer();
        }

        /**
         * @brief Gets a mutable reference to the received data buffer.
         * @warning This is marked as dangerous. In many channel implementations
         * (like triple buffering), the read buffer is a live buffer. Modifying
         * it can lead to race conditions or undefined behaviour. Its use is a
         * concession to specific needs of the physics engine and should be
         * avoided in general.
         */
        T& getBuffer_MutableDangerous()
        {
            return receiver.getBuffer();
        }
    };

    /**
     * @brief Creates a `Message` object to begin a receive operation.
     * @return A `Message` RAII wrapper.
     */
    [[nodiscard]] Message receive()
    {
        return Message{*this};
    }

    virtual ~Receiver() = default;

    /**
     * @brief Pure virtual function to check if a message is ready to be
     * received.
     * @return `true` if data is available, `false` otherwise.
     */
    virtual bool isMessageReady() = 0;
};
